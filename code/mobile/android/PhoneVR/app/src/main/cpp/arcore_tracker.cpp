#include "arcore_tracker.h"

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES3/gl3.h>

// needs the GL types from gl3.h above (GL_TEXTURE_EXTERNAL_OES)
#include <GLES2/gl2ext.h>

#include <android/log.h>
#include <chrono>
#include <cmath>
#include <ctime>

#include "arcore_c_api.h"

// utils.h defines its log() in the header, so it can only be included by alvr_main.cpp.
#define ALOGI(...) __android_log_print(ANDROID_LOG_INFO, "PhoneVR6DoF", __VA_ARGS__)
#define ALOGE(...) __android_log_print(ANDROID_LOG_ERROR, "PhoneVR6DoF", __VA_ARGS__)

namespace {

int64_t bootTimeNs() {
    timespec t{};
    clock_gettime(CLOCK_BOOTTIME, &t);
    return int64_t(t.tv_sec) * 1000000000LL + t.tv_nsec;
}

const char *failureText(int reason) {
    switch (reason) {
    case AR_TRACKING_FAILURE_REASON_BAD_STATE: return "bad state";
    case AR_TRACKING_FAILURE_REASON_INSUFFICIENT_LIGHT: return "too dark";
    case AR_TRACKING_FAILURE_REASON_EXCESSIVE_MOTION: return "moving too fast";
    case AR_TRACKING_FAILURE_REASON_INSUFFICIENT_FEATURES: return "nothing to track (blank wall?)";
    case AR_TRACKING_FAILURE_REASON_CAMERA_UNAVAILABLE: return "camera in use by another app";
    default: return "starting";
    }
}

// Hidden 1x1 GL context for ARCore's camera texture; the stream's own context is untouched.
struct HiddenGl {
    EGLDisplay display = EGL_NO_DISPLAY;
    EGLContext context = EGL_NO_CONTEXT;
    EGLSurface surface = EGL_NO_SURFACE;
    GLuint cameraTexture = 0;

    bool create() {
        display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
        if (display == EGL_NO_DISPLAY || !eglInitialize(display, nullptr, nullptr)) return false;
        const EGLint configAttribs[] = {EGL_RENDERABLE_TYPE, EGL_OPENGL_ES3_BIT_KHR, EGL_SURFACE_TYPE,
                                        EGL_PBUFFER_BIT, EGL_NONE};
        EGLConfig config;
        EGLint count = 0;
        if (!eglChooseConfig(display, configAttribs, &config, 1, &count) || count < 1) return false;
        const EGLint contextAttribs[] = {EGL_CONTEXT_CLIENT_VERSION, 3, EGL_NONE};
        context = eglCreateContext(display, config, EGL_NO_CONTEXT, contextAttribs);
        const EGLint surfaceAttribs[] = {EGL_WIDTH, 1, EGL_HEIGHT, 1, EGL_NONE};
        surface = eglCreatePbufferSurface(display, config, surfaceAttribs);
        if (context == EGL_NO_CONTEXT || surface == EGL_NO_SURFACE) return false;
        if (!eglMakeCurrent(display, surface, surface, context)) return false;
        glGenTextures(1, &cameraTexture);
        glBindTexture(GL_TEXTURE_EXTERNAL_OES, cameraTexture);
        glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        return true;
    }

    void destroy() {
        if (display == EGL_NO_DISPLAY) return;
        if (cameraTexture) glDeleteTextures(1, &cameraTexture);
        eglMakeCurrent(display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
        if (surface != EGL_NO_SURFACE) eglDestroySurface(display, surface);
        if (context != EGL_NO_CONTEXT) eglDestroyContext(display, context);
        display = EGL_NO_DISPLAY;
    }
};

}   // namespace

bool ArcoreTracker::configure(bool planeFinding) {
    ArConfig *config = nullptr;
    ArConfig_create(session_, &config);
    // Keep ARCore as light as possible: we only need the device pose (+ the floor once).
    ArConfig_setUpdateMode(session_, config, AR_UPDATE_MODE_BLOCKING);   // worker sleeps until a frame
    ArConfig_setLightEstimationMode(session_, config, AR_LIGHT_ESTIMATION_MODE_DISABLED);
    ArConfig_setDepthMode(session_, config, AR_DEPTH_MODE_DISABLED);
    ArConfig_setInstantPlacementMode(session_, config, AR_INSTANT_PLACEMENT_MODE_DISABLED);
    ArConfig_setPlaneFindingMode(
        session_, config, planeFinding ? AR_PLANE_FINDING_MODE_HORIZONTAL : AR_PLANE_FINDING_MODE_DISABLED);
    ArStatus st = ArSession_configure(session_, config);
    ArConfig_destroy(config);
    planeFindingOn_ = planeFinding;
    return st == AR_SUCCESS;
}

bool ArcoreTracker::resume(JNIEnv *env, jobject activity, int displayRotation, int width, int height) {
    std::lock_guard<std::mutex> lock(sessionMutex_);
    displayRotation_ = displayRotation;
    width_ = width;
    height_ = height;

    if (!session_) {
        ArAvailability availability = AR_AVAILABILITY_UNKNOWN_CHECKING;
        ArCoreApk_checkAvailability(env, activity, &availability);
        if (availability == AR_AVAILABILITY_UNSUPPORTED_DEVICE_NOT_CAPABLE) {
            ALOGE("ARCore: this phone isn't supported; staying in 3DoF");
            return false;
        }
        ArInstallStatus install = AR_INSTALL_STATUS_INSTALLED;
        if (ArCoreApk_requestInstall(env, activity, !installRequested_, &install) != AR_SUCCESS) {
            ALOGE("ARCore: install/update of Google Play Services for AR failed or was declined");
            return false;
        }
        if (install == AR_INSTALL_STATUS_INSTALL_REQUESTED) {
            installRequested_ = true;   // Android shows the Play Store prompt; we come back via onResume
            return false;
        }
        if (ArSession_create(env, activity, &session_) != AR_SUCCESS) {
            ALOGE("ARCore: session could not be created");
            session_ = nullptr;
            return false;
        }
        // Prefer the cheapest camera mode: 30 fps, no depth sensor, smallest image.
        ArCameraConfigFilter *filter = nullptr;
        ArCameraConfigFilter_create(session_, &filter);
        ArCameraConfigFilter_setTargetFps(session_, filter, AR_CAMERA_CONFIG_TARGET_FPS_30);
        ArCameraConfigFilter_setDepthSensorUsage(session_, filter,
                                                 AR_CAMERA_CONFIG_DEPTH_SENSOR_USAGE_DO_NOT_USE);
        ArCameraConfigList *list = nullptr;
        ArCameraConfigList_create(session_, &list);
        ArSession_getSupportedCameraConfigsWithFilter(session_, filter, list);
        int32_t n = 0;
        ArCameraConfigList_getSize(session_, list, &n);
        ArCameraConfig *best = nullptr;
        int32_t bestPixels = 0;
        for (int32_t i = 0; i < n; i++) {
            ArCameraConfig *c = nullptr;
            ArCameraConfig_create(session_, &c);
            ArCameraConfigList_getItem(session_, list, i, c);
            int32_t w = 0, h = 0;
            ArCameraConfig_getImageDimensions(session_, c, &w, &h);
            if (!best || w * h < bestPixels) {
                if (best) ArCameraConfig_destroy(best);
                best = c;
                bestPixels = w * h;
            } else {
                ArCameraConfig_destroy(c);
            }
        }
        if (best) {
            ArSession_setCameraConfig(session_, best);
            ArCameraConfig_destroy(best);
        }
        ArCameraConfigList_destroy(list);
        ArCameraConfigFilter_destroy(filter);

        ArFrame_create(session_, &frame_);
        if (!configure(true)) ALOGE("ARCore: configure failed");
        ALOGI("ARCore: session ready (%d camera modes at 30 fps, using %d pixels)", n, bestPixels);
    }

    ArSession_setDisplayGeometry(session_, displayRotation_, width_, height_);
    ArStatus st = ArSession_resume(session_);
    if (st != AR_SUCCESS) {
        ALOGE("ARCore: resume failed (%d)%s", st,
              st == AR_ERROR_CAMERA_PERMISSION_NOT_GRANTED ? " - camera permission not granted" : "");
        return false;
    }

    if (!running_.exchange(true)) {
        JavaVM *vm = nullptr;
        env->GetJavaVM(&vm);
        thread_ = std::thread(&ArcoreTracker::loop, this, vm);
    }
    return true;
}

void ArcoreTracker::pause() {
    if (running_.exchange(false) && thread_.joinable()) thread_.join();   // <= one camera frame (33 ms)
    std::lock_guard<std::mutex> lock(sessionMutex_);
    if (session_) ArSession_pause(session_);
    ArcoreSample s = latest();
    s.tracking = false;
    publish(s);
}

void ArcoreTracker::destroy() {
    pause();
    std::lock_guard<std::mutex> lock(sessionMutex_);
    if (frame_) ArFrame_destroy(frame_);
    if (session_) ArSession_destroy(session_);
    frame_ = nullptr;
    session_ = nullptr;
}

void ArcoreTracker::publish(const ArcoreSample &s) {
    seq_.fetch_add(1, std::memory_order_acq_rel);   // odd: write in progress
    std::atomic_thread_fence(std::memory_order_release);
    sample_ = s;
    std::atomic_thread_fence(std::memory_order_release);
    seq_.fetch_add(1, std::memory_order_acq_rel);   // even: stable
}

ArcoreSample ArcoreTracker::latest() const {
    ArcoreSample copy;
    for (;;) {
        uint32_t before = seq_.load(std::memory_order_acquire);
        if (before & 1) continue;
        std::atomic_thread_fence(std::memory_order_acquire);
        copy = sample_;
        std::atomic_thread_fence(std::memory_order_acquire);
        if (seq_.load(std::memory_order_acquire) == before) return copy;
    }
}

void ArcoreTracker::findFloor(ArcoreSample &s) {
    // Floor = lowest upward-facing plane clearly below the phone. Once it has been seen consistently,
    // plane finding is switched off to save CPU.
    ArTrackableList *planes = nullptr;
    ArTrackableList_create(session_, &planes);
    ArSession_getAllTrackables(session_, AR_TRACKABLE_PLANE, planes);
    int32_t n = 0;
    ArTrackableList_getSize(session_, planes, &n);
    bool found = false;
    float lowest = 0;
    ArPose *pose = nullptr;
    ArPose_create(session_, nullptr, &pose);
    for (int32_t i = 0; i < n; i++) {
        ArTrackable *t = nullptr;
        ArTrackableList_acquireItem(session_, planes, i, &t);
        ArTrackingState ts;
        ArTrackable_getTrackingState(session_, t, &ts);
        ArPlane *plane = ArAsPlane(t);
        ArPlane *subsumedBy = nullptr;
        ArPlane_acquireSubsumedBy(session_, plane, &subsumedBy);
        ArPlaneType type;
        ArPlane_getType(session_, plane, &type);
        if (ts == AR_TRACKING_STATE_TRACKING && !subsumedBy && type == AR_PLANE_HORIZONTAL_UPWARD_FACING) {
            float raw[7];
            ArPlane_getCenterPose(session_, plane, pose);
            ArPose_getPoseRaw(session_, pose, raw);
            if (raw[5] < s.position[1] - 0.5f && (!found || raw[5] < lowest)) {
                lowest = raw[5];
                found = true;
            }
        }
        if (subsumedBy) ArTrackable_release(ArAsTrackable(subsumedBy));
        ArTrackable_release(t);
    }
    ArPose_destroy(pose);
    ArTrackableList_destroy(planes);

    if (found) {
        if (floorSightings_ > 0 && std::fabs(lowest - floorCandidate_) < 0.05f)
            floorSightings_++;
        else
            floorSightings_ = 1;
        floorCandidate_ = lowest;
        if (floorSightings_ >= 3) {
            s.floorKnown = true;
            s.floorY = lowest;
            ALOGI("ARCore: floor found %.2f m below the start point; plane finding off", -lowest);
            std::lock_guard<std::mutex> lock(sessionMutex_);
            configure(false);
        }
    }
}

void ArcoreTracker::loop(JavaVM *vm) {
    JNIEnv *env = nullptr;
    vm->AttachCurrentThread(&env, nullptr);   // ARCore calls back into Java internally
    HiddenGl gl;
    if (!gl.create()) {
        ALOGE("ARCore: hidden GL context failed; staying in 3DoF");
        running_ = false;
        gl.destroy();
        vm->DetachCurrentThread();
        return;
    }
    {
        std::lock_guard<std::mutex> lock(sessionMutex_);
        ArSession_setCameraTextureName(session_, gl.cameraTexture);
    }

    ArcoreSample s = latest();
    int64_t lastTs = 0, statsStart = bootTimeNs(), lastLog = statsStart;
    int frames = 0;
    while (running_.load()) {
        ArStatus st;
        {
            std::lock_guard<std::mutex> lock(sessionMutex_);
            st = ArSession_update(session_, frame_);   // blocks until the next camera image
        }
        if (st != AR_SUCCESS) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }
        int64_t ts = 0;
        ArFrame_getTimestamp(session_, frame_, &ts);
        if (ts == lastTs) continue;   // same image as last time
        lastTs = ts;
        frames++;

        ArCamera *camera = nullptr;
        ArFrame_acquireCamera(session_, frame_, &camera);
        ArTrackingState state;
        ArCamera_getTrackingState(session_, camera, &state);
        s.timestampNs = ts;
        s.tracking = state == AR_TRACKING_STATE_TRACKING;
        if (s.tracking) {
            ArPose *pose = nullptr;
            ArPose_create(session_, nullptr, &pose);
            ArCamera_getDisplayOrientedPose(session_, camera, pose);
            float raw[7];
            ArPose_getPoseRaw(session_, pose, raw);   // qx qy qz qw tx ty tz
            ArPose_destroy(pose);
            for (int i = 0; i < 4; i++) s.orientation[i] = raw[i];
            for (int i = 0; i < 3; i++) s.position[i] = raw[4 + i];
            s.failureReason = 0;
        } else {
            ArTrackingFailureReason reason;
            ArCamera_getTrackingFailureReason(session_, camera, &reason);
            s.failureReason = reason;
        }
        ArCamera_release(camera);

        if (s.tracking && planeFindingOn_ && ts - lastFloorCheckNs_ > 500000000LL) {
            lastFloorCheckNs_ = ts;
            findFloor(s);
        }

        int64_t now = bootTimeNs();
        if (now - statsStart > 1000000000LL) {
            s.cameraFps = frames * 1e9f / float(now - statsStart);
            frames = 0;
            statsStart = now;
        }
        publish(s);

        if (now - lastLog > 5000000000LL) {   // M3: log only, position not used yet
            lastLog = now;
            if (s.tracking)
                ALOGI("ARCore %.0f fps tracking pos (%.2f, %.2f, %.2f) floor %s %.2f, age %.0f ms",
                      s.cameraFps, s.position[0], s.position[1], s.position[2],
                      s.floorKnown ? "at" : "unknown", s.floorY, (now - ts) / 1e6);
            else
                ALOGI("ARCore %.0f fps NOT tracking: %s", s.cameraFps, failureText(s.failureReason));
        }
    }
    gl.destroy();
    vm->DetachCurrentThread();
}
