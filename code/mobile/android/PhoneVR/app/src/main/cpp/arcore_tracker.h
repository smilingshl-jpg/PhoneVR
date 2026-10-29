// ARCore head-position tracker for PhoneVR 6DoF.
//
// Runs ARCore on its own thread with its own hidden EGL context, so the camera work never touches the
// stream's render thread. The camera picture is never drawn; ARCore only uses it to work out where the
// phone is. Readers (the 180 Hz tracking thread) get the newest sample through a lock-free seqlock.
#pragma once

#include <atomic>
#include <cstdint>
#include <jni.h>
#include <mutex>
#include <thread>

struct ArSession_;
struct ArFrame_;

struct ArcoreSample {
    int64_t timestampNs = 0;   // camera capture time, CLOCK_BOOTTIME (same clock ALVR uses)
    float orientation[4] = {0, 0, 0, 1};   // x, y, z, w: display-oriented camera, world from camera
    float position[3] = {0, 0, 0};          // metres, ARCore world (origin = where the session started)
    bool tracking = false;
    int failureReason = 0;   // ArTrackingFailureReason when not tracking
    bool floorKnown = false;
    float floorY = 0;        // ARCore-world height of the floor plane
    float cameraFps = 0;
};

class ArcoreTracker {
  public:
    // From onResume (Java thread). Returns false if ARCore can't run (unsupported, not installed yet,
    // no camera permission); the app then simply stays in 3DoF.
    bool resume(JNIEnv *env, jobject activity, int displayRotation, int width, int height);
    void pause();     // from onPause
    void destroy();   // from onDestroy

    // Newest sample (any thread, never blocks).
    ArcoreSample latest() const;
    bool running() const { return running_.load(); }

  private:
    void loop(JavaVM *vm);
    void publish(const ArcoreSample &s);
    void findFloor(ArcoreSample &s);
    bool configure(bool planeFinding);

    ArSession_ *session_ = nullptr;
    ArFrame_ *frame_ = nullptr;
    bool installRequested_ = false;
    bool planeFindingOn_ = false;
    int displayRotation_ = 1, width_ = 0, height_ = 0;

    std::thread thread_;
    std::atomic<bool> running_{false};
    std::mutex sessionMutex_;   // resume/pause vs. the worker's ArSession_update

    mutable std::atomic<uint32_t> seq_{0};
    ArcoreSample sample_;

    // floor detection bookkeeping (worker thread only)
    int64_t lastFloorCheckNs_ = 0;
    int floorSightings_ = 0;
    float floorCandidate_ = 0;
};
