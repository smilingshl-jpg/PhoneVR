// Head position fusion for PhoneVR 6DoF: ARCore position (30 Hz, laggy, absolute) + Cardboard rotation
// (1 kHz, predicted, but its heading drifts and has its own "forward"). Pure maths, unit tested on the PC.
//
// Output space = Cardboard's world (what ALVR already receives): gravity-aligned, Y up, metres,
// X/Z origin = where the neck was at start/recenter, Y = height above the floor.
//
// Jitter control: ARCore only tracks the neck (which moves slowly: leaning, stepping), smoothed by a
// One-Euro filter. Turning the head swings the eyes around the neck using Cardboard's fast predicted
// rotation, so head turns never wait for ARCore. Prediction is switched off while the neck is still (it only
// amplified ARCore's noise), and every new sample is blended in instead of stepping at 30 Hz.
#pragma once

#include <cmath>
#include <cstdint>

#include "pose_math.h"

namespace pvr6 {

struct FusionConfig {
    // From the rear camera to the point between the eyes, in the camera's display-oriented frame
    // (+X right, +Y up, +Z towards the wearer; the camera looks along -Z). Per phone, see cameraOffsetFor.
    Vec3 cameraToEyes{0.0f, 0.0f, 0.07f};
    // From the neck pivot to the point between the eyes, in the head frame (usual neck model: up, forward).
    Vec3 neckToEyes{0.0f, 0.075f, -0.08f};
    float defaultEyeHeight = 1.5f;   // used until ARCore has found the floor (same as upstream FLOOR_HEIGHT)
    float maxPredictionS = 0.12f;    // never extrapolate the neck further than this
    float yawTimeConstantS = 2.0f;   // how fast the ARCore->Cardboard heading alignment follows
    float yawUpdateMaxRate = 0.5f;   // rad/s: only align headings while the head turns slower than this
    float blendTimeConstantS = 0.15f;   // how fast jumps (relocalization, floor found) are smoothed out
    float stepBlendTimeConstantS = 0.04f;   // how fast each ordinary 30 Hz sample is blended in
    float jumpThreshold = 0.08f;     // metres: more than this off the prediction counts as a jump
    float lostVelocityDecayS = 0.10f;   // after tracking loss the head glides to a stop this quickly
    // One-Euro filter on the neck: smooth when still, follows quickly when moving.
    float minCutoffHz = 1.5f;
    float beta = 4.0f;               // cutoff rises by this many Hz per m/s
    float velocityCutoffHz = 3.0f;
    // Prediction fades in between these neck speeds (m/s); below the first the head is treated as still.
    float stillSpeed = 0.04f, movingSpeed = 0.15f;
};

// Camera-to-eyes offset for phones we know (display-oriented frame, screen in landscape with the phone's top
// on the wearer's left, which is the only way PhoneVR shows the picture). Unknown phones keep the default.
inline Vec3 cameraOffsetFor(const char *model) {
    auto starts = [model](const char *p) {
        for (const char *m = model; *p; p++, m++)
            if (*m != *p) return false;
        return true;
    };
    if (!model) return FusionConfig{}.cameraToEyes;
    // Galaxy A8+ (2018): camera centred at the top edge (portrait), ~1.2 cm in -> wearer's left, mid height.
    if (starts("SM-A730")) return {0.068f, 0.0f, 0.075f};
    // Galaxy S22: main lens near the top-left corner seen from the back -> wearer's left, upper half.
    if (starts("SM-S901")) return {0.057f, -0.020f, 0.075f};
    return FusionConfig{}.cameraToEyes;
}

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
    const FusionConfig &config() const { return cfg_; }
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

        // 1. Neck pivot in ARCore world.
        Vec3 neck = add(in.position, rotate(in.orientation, sub(cfg_.cameraToEyes, cfg_.neckToEyes)));

        // 2. Heading alignment ARCore -> Cardboard (both gravity aligned, so yaw only), low-passed so
        //    Cardboard's slow drift is absorbed without making the position wobble. Skipped during fast turns,
        //    where the two sensors' small timing difference looks like a heading difference.
        float arYaw = yawOf(in.orientation);
        float yawDiff = wrapAngle(yawOf(cardboardAtSample) - arYaw);
        if (!haveYaw_) {
            yawOffset_ = yawDiff;
            haveYaw_ = true;
        } else if (dt > 0 && std::fabs(wrapAngle(arYaw - lastArYaw_)) / dt < cfg_.yawUpdateMaxRate) {
            float a = std::fmin(1.0f, dt / cfg_.yawTimeConstantS);
            yawOffset_ = wrapAngle(yawOffset_ + a * wrapAngle(yawDiff - yawOffset_));
        }
        lastArYaw_ = arYaw;

        // 3. Origin: first sample (or recenter) defines X/Z = 0.
        bool restart = false;
        if (!haveOrigin_ || recenterRequested_) {
            originXZ_ = {neck.x, 0, neck.z};
            startY_ = neck.y;
            restart = true;   // recenter jumps on purpose: no smoothing, fresh velocity
            haveOrigin_ = true;
            recenterRequested_ = false;
        }

        // 4. Into Cardboard world: rotate horizontal offset by the yaw alignment, height above the floor.
        Vec3 rel = sub(neck, originXZ_);
        Vec3 horiz = rotate(yawOnly(yawOffset_), Vec3{rel.x, 0, rel.z});
        float height = in.floorKnown ? neck.y - in.floorY
                                     : neck.y - startY_ + cfg_.defaultEyeHeight - cfg_.neckToEyes.y;
        Vec3 raw{horiz.x, height, horiz.z};

        if (restart || !haveState_) {
            x_ = prevRaw_ = raw;
            v_ = {};
            lag_ = 0;
            big_ = step_ = {};
            stateTs_ = in.timestampNs;
            haveState_ = true;
            tracking_ = true;
            return;
        }

        // 5. Filter, keeping the output continuous: whatever the new sample changes is hidden and faded
        //    out (quickly for ordinary samples, slowly for real jumps). Measured at the time ALVR last asked
        //    for, since its requests run ahead of the camera samples.
        int64_t at = lastQueryNs_ > in.timestampNs ? lastQueryNs_ : in.timestampNs;
        Vec3 before = neckAt(at);
        bool regained = !tracking_;
        Vec3 expected = add(x_, scale(v_, dt));
        bool jumped = regained || length(sub(raw, expected)) > cfg_.jumpThreshold;
        if (jumped) {
            // a jump is not head motion: move the filter there without turning it into speed
            x_ = raw;
            if (regained) v_ = {};
        } else {
            oneEuro(raw, dt);
        }
        prevRaw_ = raw;
        stateTs_ = in.timestampNs;
        tracking_ = true;
        Vec3 change = sub(neckAt(at), before);
        if (jumped) {
            big_ = sub(bigAt(at), change);
            bigTs_ = at;
        } else {
            step_ = sub(stepAt(at), change);
            stepTs_ = at;
        }
    }

    // Eye centre position at targetNs in Cardboard world, given Cardboard's (predicted) rotation for that
    // moment. Call from the tracking thread.
    Vec3 positionAt(int64_t targetNs, Quat cardboardAtTarget) const {
        if (!haveState_) return {0, cfg_.defaultEyeHeight, 0};
        lastQueryNs_ = targetNs;
        Vec3 neck = add(neckAt(targetNs), add(bigAt(targetNs), stepAt(targetNs)));
        return add(neck, rotate(cardboardAtTarget, cfg_.neckToEyes));
    }

    // Neck velocity as used for prediction (zero while still or lost); for ALVR's linear_velocity.
    Vec3 velocity() const { return tracking_ ? predictionVelocity() : Vec3{}; }
    bool tracking() const { return tracking_; }

  private:
    static float alpha(float cutoffHz, float dt) {
        float tau = 1.0f / (2.0f * kPi * cutoffHz);
        return 1.0f / (1.0f + tau / dt);
    }

    void oneEuro(Vec3 raw, float dt) {
        if (dt <= 0) return;
        Vec3 dx = scale(sub(raw, prevRaw_), 1.0f / dt);
        v_ = add(v_, scale(sub(dx, v_), alpha(cfg_.velocityCutoffHz, dt)));
        float cutoff = cfg_.minCutoffHz + cfg_.beta * length(v_);
        x_ = add(x_, scale(sub(raw, x_), alpha(cutoff, dt)));
        lag_ = 1.0f / (2.0f * kPi * cutoff);   // a low-pass trails a steady motion by about this long
    }

    Vec3 predictionVelocity() const {
        float s = length(v_);
        float k = (s - cfg_.stillSpeed) / (cfg_.movingSpeed - cfg_.stillSpeed);
        k = std::fmin(1.0f, std::fmax(0.0f, k));
        return scale(v_, k * k * (3 - 2 * k));
    }

    // Filtered neck pushed ahead to time t (no blend offsets).
    Vec3 neckAt(int64_t t) const {
        float ahead = float(t - stateTs_) * 1e-9f;
        if (ahead < 0) ahead = 0;
        if (tracking_) {
            float total = std::fmin(ahead + lag_, cfg_.maxPredictionS);
            return add(x_, scale(predictionVelocity(), total));
        }
        // lost: glide to a stop instead of flying off, then hold
        float tau = cfg_.lostVelocityDecayS;
        return add(x_, scale(predictionVelocity(), tau * (1.0f - std::exp(-ahead / tau))));
    }

    static Vec3 fade(Vec3 offset, int64_t since, int64_t t, float tau) {
        float s = float(t - since) * 1e-9f;
        if (s <= 0) return offset;
        return scale(offset, std::exp(-s / tau));
    }
    Vec3 bigAt(int64_t t) const { return fade(big_, bigTs_, t, cfg_.blendTimeConstantS); }
    Vec3 stepAt(int64_t t) const { return fade(step_, stepTs_, t, cfg_.stepBlendTimeConstantS); }

    FusionConfig cfg_;
    int64_t lastTs_ = 0, stateTs_ = 0, bigTs_ = 0, stepTs_ = 0;
    bool haveYaw_ = false, haveOrigin_ = false, haveState_ = false, tracking_ = false;
    bool recenterRequested_ = false;
    float yawOffset_ = 0, lastArYaw_ = 0, startY_ = 0, lag_ = 0;
    mutable int64_t lastQueryNs_ = 0;   // positionAt runs on the same (input) thread as addSample
    Vec3 originXZ_, x_, v_, prevRaw_, big_, step_;
};

}   // namespace pvr6
