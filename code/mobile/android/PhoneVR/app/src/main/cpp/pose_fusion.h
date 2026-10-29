// Head position fusion for PhoneVR 6DoF: ARCore position (30 Hz, laggy, absolute) + Cardboard rotation
// (1 kHz, predicted, but its heading drifts and has its own "forward"). Pure maths, unit tested on the PC.
//
// Output space = Cardboard's world (what ALVR already receives): gravity-aligned, Y up, metres,
// X/Z origin = where the head was at start/recenter, Y = height above the floor.
#pragma once

#include <cmath>
#include <cstdint>

#include "pose_math.h"

namespace pvr6 {

struct FusionConfig {
    // From the rear camera to the point between the eyes, in the camera's display-oriented frame
    // (+X right, +Y up, +Z towards the wearer; the camera looks along -Z). Per phone, see presets.
    Vec3 cameraToEyes{0.0f, 0.0f, 0.07f};
    float defaultEyeHeight = 1.5f;   // used until ARCore has found the floor (same as upstream FLOOR_HEIGHT)
    float maxPredictionS = 0.10f;    // never extrapolate further than this
    float yawTimeConstantS = 2.0f;   // how fast the ARCore->Cardboard heading alignment follows
    float blendTimeConstantS = 0.15f;   // how fast jumps (relocalization, floor found) are smoothed out
    float lostVelocityDecayS = 0.10f;   // after tracking loss the head glides to a stop this quickly
};

struct ArInput {
    int64_t timestampNs = 0;
    Quat orientation;   // ARCore world from (display-oriented) camera
    Vec3 position;      // ARCore world, metres
    bool tracking = false;
    bool floorKnown = false;
    float floorY = 0;
};

class HeadPositionFusion {
  public:
    explicit HeadPositionFusion(FusionConfig cfg = {}) : cfg_(cfg) {}

    void setConfig(const FusionConfig &cfg) { cfg_ = cfg; }
    bool valid() const { return haveOrigin_; }

    // Recenter: the current spot becomes X/Z = 0 again (height is kept).
    void recenter() { recenterRequested_ = true; }

    // New ARCore sample + the Cardboard head orientation at (about) the same moment.
    void addSample(const ArInput &in, Quat cardboardAtSample) {
        if (in.timestampNs <= lastTs_) return;
        float dt = lastTs_ ? float(in.timestampNs - lastTs_) * 1e-9f : 0.0f;
        lastTs_ = in.timestampNs;

        if (!in.tracking) {
            tracking_ = false;
            return;
        }

        // 1. Eye centre in ARCore world.
        Vec3 eyes = add(in.position, rotate(in.orientation, cfg_.cameraToEyes));

        // 2. Heading alignment ARCore -> Cardboard (both gravity aligned, so yaw only), low-passed so
        //    Cardboard's slow drift is absorbed without making the position wobble.
        float yawDiff = wrapAngle(yawOf(cardboardAtSample) - yawOf(in.orientation));
        if (!haveYaw_) {
            yawOffset_ = yawDiff;
            haveYaw_ = true;
        } else if (dt > 0) {
            float a = std::fmin(1.0f, dt / cfg_.yawTimeConstantS);
            yawOffset_ = wrapAngle(yawOffset_ + a * wrapAngle(yawDiff - yawOffset_));
        }

        // 3. Origin: first sample (or recenter) defines X/Z = 0.
        if (!haveOrigin_ || recenterRequested_) {
            originXZ_ = {eyes.x, 0, eyes.z};
            startY_ = eyes.y;
            if (haveOrigin_) {   // recenter: jump on purpose, no smoothing, fresh velocity
                blendOffset_ = {};
                velocity_ = {};
                haveRaw_ = false;
            }
            haveOrigin_ = true;
            recenterRequested_ = false;
        }

        // 4. Into Cardboard world: rotate horizontal offset by the yaw alignment, height above the floor.
        Vec3 rel = sub(eyes, originXZ_);
        Vec3 horiz = rotate(yawOnly(yawOffset_), Vec3{rel.x, 0, rel.z});
        float height = in.floorKnown ? eyes.y - in.floorY : eyes.y - startY_ + cfg_.defaultEyeHeight;
        Vec3 raw{horiz.x, height, horiz.z};

        // 5. Continuity: a sudden jump (ARCore relocalized, tracking regained, floor found) is blended in.
        if (haveRaw_) {
            Vec3 expected = predictRaw(in.timestampNs);
            Vec3 jump = sub(raw, expected);
            bool regained = !tracking_;
            bool jumped = regained || length(jump) > 0.08f;
            if (jumped) {
                // hide the jump now, fade the hiding out from this moment on
                blendOffset_ = sub(blendAt(in.timestampNs), jump);
                blendTs_ = in.timestampNs;
            }
            if (regained) {
                velocity_ = {};
            } else if (dt > 0 && !jumped) {   // a jump is not head motion: don't let it become speed
                Vec3 v = scale(sub(raw, lastRaw_), 1.0f / dt);
                velocity_ = add(scale(velocity_, 0.5f), scale(v, 0.5f));
            }
        }
        lastRaw_ = raw;
        rawTs_ = in.timestampNs;
        haveRaw_ = true;
        tracking_ = true;
    }

    // Head (eye centre) position at targetNs, in Cardboard world. Call from the tracking thread.
    Vec3 positionAt(int64_t targetNs) const {
        if (!haveRaw_) return {0, cfg_.defaultEyeHeight, 0};
        return add(predictRaw(targetNs), blendAt(targetNs));
    }

    Vec3 velocity() const { return tracking_ ? velocity_ : Vec3{}; }
    bool tracking() const { return tracking_; }

  private:
    // What's left of a hidden jump at time t (fades out with blendTimeConstantS).
    Vec3 blendAt(int64_t t) const {
        float since = float(t - blendTs_) * 1e-9f;
        if (since <= 0) return blendOffset_;
        return scale(blendOffset_, std::exp(-since / cfg_.blendTimeConstantS));
    }

    Vec3 predictRaw(int64_t targetNs) const {
        float ahead = float(targetNs - rawTs_) * 1e-9f;
        if (ahead < 0) ahead = 0;
        if (tracking_) {
            ahead = std::fmin(ahead, cfg_.maxPredictionS);
            return add(lastRaw_, scale(velocity_, ahead));
        }
        // lost: glide to a stop instead of flying off, then hold
        float tau = cfg_.lostVelocityDecayS;
        return add(lastRaw_, scale(velocity_, tau * (1.0f - std::exp(-ahead / tau))));
    }

    FusionConfig cfg_;
    int64_t lastTs_ = 0, rawTs_ = 0, blendTs_ = 0;
    bool haveYaw_ = false, haveOrigin_ = false, haveRaw_ = false, tracking_ = false;
    bool recenterRequested_ = false;
    float yawOffset_ = 0, startY_ = 0;
    Vec3 originXZ_, lastRaw_, velocity_, blendOffset_;
};

}   // namespace pvr6
