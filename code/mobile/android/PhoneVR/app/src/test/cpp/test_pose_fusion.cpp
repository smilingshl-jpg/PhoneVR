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

// ARCore sample for a head whose eye centre is at `eyes` and faces `q` (ARCore world).
static ArInput sampleFor(const FusionConfig &cfg, Vec3 eyes, Quat q, int64_t t, bool tracking = true) {
    ArInput in;
    in.timestampNs = t;
    in.orientation = q;
    in.position = sub(eyes, rotate(q, cfg.cameraToEyes));   // where the camera lens is
    in.tracking = tracking;
    return in;
}

static void test_turning_head_does_not_move_it() {
    FusionConfig cfg;
    HeadPositionFusion f(cfg);
    Vec3 eyes{0.3f, 1.2f, -0.4f};
    int64_t t = 1000 * MS;
    for (int i = 0; i <= 60; i++, t += 33 * MS) {
        Quat q = normalize(mul(yawOnly(i * 3 * DEG), fromAxisAngle({1, 0, 0}, (i % 10) * DEG)));
        f.addSample(sampleFor(cfg, eyes, q, t), q);
    }
    Vec3 p = f.positionAt(t);
    CHECK(near(p.x, 0, 0.01f) && near(p.z, 0, 0.01f));   // still where it started
    CHECK(near(p.y, cfg.defaultEyeHeight, 0.01f));        // floor not known yet: default height
}

static void test_leaning_maps_through_the_heading_alignment() {
    FusionConfig cfg;
    HeadPositionFusion f(cfg);
    // Cardboard's "forward" is 90 deg to the left of ARCore's for the same real heading.
    Quat ar = yawOnly(0), cb = yawOnly(90 * DEG);
    int64_t t = 1000 * MS;
    f.addSample(sampleFor(cfg, {0, 1.3f, 0}, ar, t), cb);
    for (int i = 1; i <= 10; i++) f.addSample(sampleFor(cfg, {0.02f * i, 1.3f, 0}, ar, t += 33 * MS), cb);
    Vec3 p = f.positionAt(t);
    // leaning 20 cm to the wearer's right: in Cardboard's world (facing -X) right is -Z
    CHECK(near(p.z, -0.20f, 0.01f) && near(p.x, 0, 0.01f));
}

static void test_floor_height() {
    FusionConfig cfg;
    HeadPositionFusion f(cfg);
    ArInput in = sampleFor(cfg, {0, 0.25f, 0}, Quat{}, 1000 * MS);
    in.floorKnown = true;
    in.floorY = -1.45f;   // floor 1.45 m below the ARCore origin
    f.addSample(in, Quat{});
    CHECK(near(f.positionAt(1000 * MS).y, 1.70f));
}

static void test_prediction_is_clamped() {
    FusionConfig cfg;
    HeadPositionFusion f(cfg);
    int64_t t = 1000 * MS;
    for (int i = 0; i <= 10; i++) f.addSample(sampleFor(cfg, {i * 0.033f, 1.5f, 0}, Quat{}, t + i * 33 * MS), Quat{});
    t += 10 * 33 * MS;   // moving +X at 1 m/s
    Vec3 soon = f.positionAt(t + 20 * MS);
    Vec3 far = f.positionAt(t + 1000 * MS);
    CHECK(soon.x > 0.33f + 0.01f);                 // predicted ahead
    CHECK(far.x < 0.33f + cfg.maxPredictionS + 0.02f);   // but never more than 100 ms ahead
}

static void test_tracking_loss_glides_then_holds() {
    FusionConfig cfg;
    HeadPositionFusion f(cfg);
    int64_t t = 1000 * MS;
    for (int i = 0; i <= 10; i++) f.addSample(sampleFor(cfg, {i * 0.033f, 1.5f, 0}, Quat{}, t + i * 33 * MS), Quat{});
    t += 10 * 33 * MS;
    f.addSample(sampleFor(cfg, {9, 9, 9}, Quat{}, t + 33 * MS, false), Quat{});   // lost (pose ignored)
    CHECK(!f.tracking());
    Vec3 a = f.positionAt(t + 2000 * MS), b = f.positionAt(t + 4000 * MS);
    CHECK(near(a.x, b.x, 1e-4f));                                   // holds still
    CHECK(a.x < 0.33f + cfg.lostVelocityDecayS * 1.0f + 0.02f);     // glided at most v*tau
    CHECK(near(f.velocity().x, 0));
}

static void test_relocalization_jump_is_smoothed() {
    FusionConfig cfg;
    HeadPositionFusion f(cfg);
    int64_t t = 1000 * MS;
    for (int i = 0; i < 5; i++) f.addSample(sampleFor(cfg, {0, 1.5f, 0}, Quat{}, t += 33 * MS), Quat{});
    Vec3 before = f.positionAt(t);
    f.addSample(sampleFor(cfg, {0.5f, 1.5f, 0}, Quat{}, t += 33 * MS), Quat{});   // ARCore snaps 50 cm
    Vec3 right_after = f.positionAt(t);
    Vec3 later = f.positionAt(t + 1000 * MS);
    CHECK(near(right_after.x, before.x, 0.02f));   // no visible jump
    CHECK(near(later.x, 0.5f, 0.02f));             // then settles on the new position
}

static void test_recenter() {
    FusionConfig cfg;
    HeadPositionFusion f(cfg);
    int64_t t = 1000 * MS;
    f.addSample(sampleFor(cfg, {0, 1.5f, 0}, Quat{}, t), Quat{});
    f.addSample(sampleFor(cfg, {1.0f, 1.5f, 1.0f}, Quat{}, t += 33 * MS), Quat{});
    f.recenter();
    f.addSample(sampleFor(cfg, {1.0f, 1.5f, 1.0f}, Quat{}, t += 33 * MS), Quat{});
    Vec3 p = f.positionAt(t);
    CHECK(near(p.x, 0, 0.01f) && near(p.z, 0, 0.01f) && near(p.y, 1.5f, 0.01f));
}

int main() {
    test_turning_head_does_not_move_it();
    test_leaning_maps_through_the_heading_alignment();
    test_floor_height();
    test_prediction_is_clamped();
    test_tracking_loss_glides_then_holds();
    test_relocalization_jump_is_smoothed();
    test_recenter();
    if (failures) {
        std::printf("%d check(s) failed\n", failures);
        return 1;
    }
    std::printf("pose_fusion: all tests passed\n");
    return 0;
}
