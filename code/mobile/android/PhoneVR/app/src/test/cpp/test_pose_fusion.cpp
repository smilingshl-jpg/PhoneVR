// Host-side tests for pose_fusion.h. Run: bash run_tests.sh
#include "../../main/cpp/pose_fusion.h"

#include <cstdio>

using namespace pvr6;

static int failures = 0;
#define CHECK(cond)                                                                                \
    do {                                                                                           \
        if (!(cond)) {                                                                             \
            std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);                            \
            failures++;                                                                            \
        }                                                                                          \
    } while (0)
static bool near(float a, float b, float eps = 1e-3f) { return std::fabs(a - b) < eps; }
static const float DEG = kPi / 180.0f;
static const int64_t MS = 1000000;

// ARCore sample for a head whose neck pivot is at `neck` and faces `q` (ARCore world).
static ArInput sampleFor(const FusionConfig &cfg, Vec3 neck, Quat q, int64_t t, bool tracking = true) {
    ArInput in;
    in.timestampNs = t;
    in.orientation = q;
    in.position = sub(neck, rotate(q, sub(cfg.cameraToEyes, cfg.neckToEyes)));   // where the camera lens is
    in.tracking = tracking;
    return in;
}

// Neck position the fusion reports (eyes minus the neck-model arm for that rotation).
static Vec3 neckOut(const HeadPositionFusion &f, int64_t t, Quat q) {
    return sub(f.positionAt(t, q), rotate(q, f.config().neckToEyes));
}

// Small deterministic "noise" in metres, like ARCore's frame-to-frame wobble.
static float wobble(int i, float amp) { return amp * std::sin(i * 2.3f) * std::cos(i * 0.7f); }

static void test_turning_head_swings_eyes_around_a_still_neck() {
    FusionConfig cfg;
    HeadPositionFusion f(cfg);
    Vec3 neck{0.3f, 1.2f, -0.4f};
    int64_t t = 1000 * MS;
    Quat q;
    for (int i = 0; i <= 60; i++, t += 33 * MS) {
        q = normalize(mul(yawOnly(i * 3 * DEG), fromAxisAngle({1, 0, 0}, (i % 10) * DEG)));
        f.addSample(sampleFor(cfg, neck, q, t), q);
    }
    Vec3 n = neckOut(f, t, q);
    CHECK(near(n.x, 0, 0.01f) && near(n.z, 0, 0.01f));                  // neck still where it started
    CHECK(near(n.y, cfg.defaultEyeHeight - cfg.neckToEyes.y, 0.01f));    // floor not known yet
    // eyes follow Cardboard's rotation at the target time straight away, not ARCore's 30 Hz samples
    Quat later = yawOnly(90 * DEG);
    Vec3 e = f.positionAt(t, later);
    Vec3 want = add(n, rotate(later, cfg.neckToEyes));
    CHECK(near(e.x, want.x, 0.005f) && near(e.z, want.z, 0.005f));
}

static void test_leaning_maps_through_the_heading_alignment() {
    FusionConfig cfg;
    HeadPositionFusion f(cfg);
    // Cardboard's "forward" is 90 deg to the left of ARCore's for the same real heading.
    Quat ar = yawOnly(0), cb = yawOnly(90 * DEG);
    int64_t t = 1000 * MS;
    f.addSample(sampleFor(cfg, {0, 1.3f, 0}, ar, t), cb);
    for (int i = 1; i <= 10; i++) f.addSample(sampleFor(cfg, {0.02f * i, 1.3f, 0}, ar, t += 33 * MS), cb);
    for (int i = 0; i < 30; i++) f.addSample(sampleFor(cfg, {0.20f, 1.3f, 0}, ar, t += 33 * MS), cb);   // settle
    Vec3 n = neckOut(f, t, cb);
    // leaning 20 cm to the wearer's right: in Cardboard's world (facing -X) right is -Z
    CHECK(near(n.z, -0.20f, 0.01f) && near(n.x, 0, 0.01f));
}

static void test_floor_height() {
    FusionConfig cfg;
    HeadPositionFusion f(cfg);
    ArInput in = sampleFor(cfg, {0, 0.25f, 0}, Quat{}, 1000 * MS);
    in.floorKnown = true;
    in.floorY = -1.45f;   // floor 1.45 m below the ARCore origin
    f.addSample(in, Quat{});
    CHECK(near(neckOut(f, 1000 * MS, Quat{}).y, 1.70f));
    CHECK(near(f.positionAt(1000 * MS, Quat{}).y, 1.70f + cfg.neckToEyes.y));
}

static void test_prediction_is_clamped() {
    FusionConfig cfg;
    HeadPositionFusion f(cfg);
    int64_t t = 1000 * MS;
    for (int i = 0; i <= 20; i++)
        f.addSample(sampleFor(cfg, {i * 0.033f, 1.5f, 0}, Quat{}, t + i * 33 * MS), Quat{});
    t += 20 * 33 * MS;   // moving +X at 1 m/s, now at 0.66
    float soon = neckOut(f, t + 30 * MS, Quat{}).x;
    float far = neckOut(f, t + 1000 * MS, Quat{}).x;
    CHECK(soon > 0.66f + 0.01f);                       // predicted ahead
    CHECK(far < 0.66f + cfg.maxPredictionS + 0.03f);   // but never far ahead
}

static void test_still_head_does_not_shake() {
    FusionConfig cfg;
    HeadPositionFusion f(cfg);
    int64_t t = 1000 * MS;
    float lo = 1e9f, hi = -1e9f;
    for (int i = 0; i < 300; i++, t += 33 * MS) {
        Vec3 neck{wobble(i, 0.002f), 1.3f + wobble(i + 50, 0.002f), wobble(i + 99, 0.002f)};   // +-2 mm
        f.addSample(sampleFor(cfg, neck, Quat{}, t), Quat{});
        if (i < 60) continue;
        for (int k = 0; k < 3; k++) {   // ALVR asks ~3 times per sample, ~80 ms ahead
            float x = neckOut(f, t + k * 11 * MS + 80 * MS, Quat{}).x;
            lo = std::fmin(lo, x);
            hi = std::fmax(hi, x);
        }
    }
    CHECK(hi - lo < 0.0025f);   // input moves up to 4 mm peak to peak; output must be calmer
}

static void test_moving_output_has_no_30hz_steps() {
    FusionConfig cfg;
    HeadPositionFusion f(cfg);
    int64_t t = 1000 * MS;
    float prev = 0, worst = 0;
    bool have = false;
    for (int i = 0; i < 90; i++) {
        Vec3 neck{0.5f * i * 0.033f + wobble(i, 0.002f), 1.3f, 0};   // walking sideways 0.5 m/s
        f.addSample(sampleFor(cfg, neck, Quat{}, t), Quat{});
        for (int k = 0; k < 33; k += 3) {   // look every 3 ms across the sample boundary
            float x = neckOut(f, t + k * MS + 60 * MS, Quat{}).x;
            if (have && i > 20) worst = std::fmax(worst, std::fabs(x - prev));
            prev = x;
            have = true;
        }
        t += 33 * MS;
    }
    CHECK(worst < 0.004f);   // 0.5 m/s is 1.5 mm per 3 ms; no big steps when a sample lands
}

static void test_tracking_loss_glides_then_holds() {
    FusionConfig cfg;
    HeadPositionFusion f(cfg);
    int64_t t = 1000 * MS;
    for (int i = 0; i <= 20; i++)
        f.addSample(sampleFor(cfg, {i * 0.033f, 1.5f, 0}, Quat{}, t + i * 33 * MS), Quat{});
    t += 20 * 33 * MS;
    f.addSample(sampleFor(cfg, {9, 9, 9}, Quat{}, t + 33 * MS, false), Quat{});   // lost (pose ignored)
    CHECK(!f.tracking());
    Vec3 a = neckOut(f, t + 2000 * MS, Quat{}), b = neckOut(f, t + 4000 * MS, Quat{});
    CHECK(near(a.x, b.x, 1e-4f));                                 // holds still
    CHECK(a.x < 0.66f + cfg.lostVelocityDecayS * 1.0f + 0.03f);   // glided at most v*tau
    CHECK(near(f.velocity().x, 0));
}

static void test_relocalization_jump_is_smoothed() {
    FusionConfig cfg;
    HeadPositionFusion f(cfg);
    int64_t t = 1000 * MS;
    for (int i = 0; i < 5; i++) f.addSample(sampleFor(cfg, {0, 1.5f, 0}, Quat{}, t += 33 * MS), Quat{});
    Vec3 before = neckOut(f, t, Quat{});
    f.addSample(sampleFor(cfg, {0.5f, 1.5f, 0}, Quat{}, t += 33 * MS), Quat{});   // ARCore snaps 50 cm
    Vec3 right_after = neckOut(f, t, Quat{});
    for (int i = 0; i < 30; i++) f.addSample(sampleFor(cfg, {0.5f, 1.5f, 0}, Quat{}, t += 33 * MS), Quat{});
    Vec3 later = neckOut(f, t, Quat{});
    CHECK(near(right_after.x, before.x, 0.02f));   // no visible jump
    CHECK(near(later.x, 0.5f, 0.02f));             // then settles on the new position
    CHECK(length(f.velocity()) < 0.05f);           // and the jump didn't become speed
}

static void test_recenter() {
    FusionConfig cfg;
    HeadPositionFusion f(cfg);
    int64_t t = 1000 * MS;
    f.addSample(sampleFor(cfg, {0, 1.5f, 0}, Quat{}, t), Quat{});
    f.addSample(sampleFor(cfg, {1.0f, 1.5f, 1.0f}, Quat{}, t += 33 * MS), Quat{});
    f.recenter();
    f.addSample(sampleFor(cfg, {1.0f, 1.5f, 1.0f}, Quat{}, t += 33 * MS), Quat{});
    Vec3 n = neckOut(f, t, Quat{});
    CHECK(near(n.x, 0, 0.01f) && near(n.z, 0, 0.01f) && near(n.y, 1.5f - cfg.neckToEyes.y, 0.01f));
}

static void test_phone_presets() {
    CHECK(near(cameraOffsetFor("SM-A730F").x, 0.068f));
    CHECK(near(cameraOffsetFor("SM-S901W").x, 0.057f));
    CHECK(near(cameraOffsetFor("Pixel 7").x, FusionConfig{}.cameraToEyes.x));
    CHECK(near(cameraOffsetFor(nullptr).z, FusionConfig{}.cameraToEyes.z));
}

int main() {
    test_turning_head_swings_eyes_around_a_still_neck();
    test_leaning_maps_through_the_heading_alignment();
    test_floor_height();
    test_prediction_is_clamped();
    test_still_head_does_not_shake();
    test_moving_output_has_no_30hz_steps();
    test_tracking_loss_glides_then_holds();
    test_relocalization_jump_is_smoothed();
    test_recenter();
    test_phone_presets();
    if (failures) {
        std::printf("%d check(s) failed\n", failures);
        return 1;
    }
    std::printf("pose_fusion: all tests passed\n");
    return 0;
}
