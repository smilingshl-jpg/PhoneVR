// Host-side tests for hand_geometry.h (no Android needed). Run: bash run_tests.sh
#include "../../main/cpp/hand_geometry.h"

#include <cstdio>
#include <cstdlib>
#include <vector>

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

static const Lens LENS{500, 500, 320, 240, 640, 480};
// An open right palm about 9 cm long, around its own middle, in picture axes (x right, y down, z away).
static const float PALM[kPalmPoints][3] = {{0.00f, 0.05f, 0.01f},   {0.03f, 0.02f, 0.00f},  {0.025f, -0.03f, 0.00f},
                                           {0.005f, -0.04f, -0.01f}, {-0.015f, -0.03f, 0.00f}, {-0.03f, -0.02f, 0.01f}};

// What the model would report for a hand whose middle sits at `at` (picture axes), lens-projected.
static std::vector<float> seen(Vec3 at, float modelLeft = 0, float score = 0.9f, float noise = 0) {
    std::vector<float> v{modelLeft, score};
    for (int i = 0; i < kPalmPoints; i++) {
        float x = PALM[i][0] + at.x, y = PALM[i][1] + at.y, z = PALM[i][2] + at.z;
        v.push_back((LENS.fx * x / z + LENS.cx + (i % 2 ? noise : -noise)) / LENS.width);
        v.push_back((LENS.fy * y / z + LENS.cy) / LENS.height);
        for (float w : PALM[i]) v.push_back(w);
    }
    return v;
}

static void test_recovers_a_hand_at_arms_length() {
    HandPoint h;
    Vec3 at{0.10f, 0.05f, 0.45f};
    CHECK(solveHand(seen(at).data(), LENS, 1, Vec3{}, &h));
    CHECK(near(h.head.x, at.x + PALM[0][0]));
    CHECK(near(h.head.y, -(at.y + PALM[0][1])));   // picture y is down, head y is up
    CHECK(near(h.head.z, -(at.z + PALM[0][2])));   // and the hand is in front, along -Z
}

static void test_upside_down_phone_gives_the_same_answer_in_the_head() {
    HandPoint upright, flipped;
    Vec3 at{0.10f, 0.05f, 0.45f};
    solveHand(seen(at).data(), LENS, 1, Vec3{}, &upright);
    // Same hand, phone rotated 180 degrees about the view: everything in the picture swaps sides.
    Vec3 mirrored{-at.x - 2 * PALM[0][0], -at.y - 2 * PALM[0][1], at.z};
    solveHand(seen(mirrored).data(), LENS, 3, Vec3{}, &flipped);
    CHECK(near(flipped.head.x, upright.head.x));
    CHECK(near(flipped.head.y, upright.head.y));
}

static void test_camera_offset_moves_the_hand_into_head_space() {
    HandPoint plain, moved;
    Vec3 at{0.0f, 0.0f, 0.4f}, offset{0.057f, -0.02f, 0.075f};   // S22
    solveHand(seen(at).data(), LENS, 1, Vec3{}, &plain);
    solveHand(seen(at).data(), LENS, 1, offset, &moved);
    CHECK(near(moved.head.x, plain.head.x - offset.x));
    CHECK(near(moved.head.y, plain.head.y - offset.y));
    CHECK(near(moved.head.z, plain.head.z - offset.z));   // eyes sit behind the camera, so the hand is further away
}

static void test_model_left_is_really_the_right_on_a_rear_camera() {
    HandPoint h;
    solveHand(seen({0, 0, 0.4f}, 1).data(), LENS, 1, Vec3{}, &h);
    CHECK(!h.left);
    solveHand(seen({0, 0, 0.4f}, 0).data(), LENS, 1, Vec3{}, &h);
    CHECK(h.left);
}

static void test_a_pixel_of_noise_moves_a_near_hand_only_a_little() {
    HandPoint clean, noisy;
    Vec3 at{0.05f, 0.0f, 0.5f};
    solveHand(seen(at).data(), LENS, 1, Vec3{}, &clean);
    solveHand(seen(at, 0, 0.9f, 1.0f).data(), LENS, 1, Vec3{}, &noisy);
    CHECK(length(sub(clean.head, noisy.head)) < 0.05f);
}

static void test_impossible_distances_are_dropped() {
    HandPoint h;
    CHECK(!solveHand(seen({0, 0, 3.0f}).data(), LENS, 1, Vec3{}, &h));
    CHECK(!solveHand(seen({0, 0, 0.45f}).data(), Lens{}, 1, Vec3{}, &h));   // no lens numbers yet
}

static void test_yuv_grey_and_colour() {
    const int w = 4, h = 2, stride = 6, pixel = 2;   // interleaved U V, with row padding
    uint8_t y[stride * h], uv[stride * 1];
    for (auto &b : y) b = 100;
    for (int i = 0; i < stride; i += pixel) uv[i] = uv[i + 1] = 128;
    uint8_t rgba[w * h * 4];
    yuvToRgba(y, stride, uv, uv + 1, stride, pixel, w, h, rgba);
    CHECK(rgba[0] == 100 && rgba[1] == 100 && rgba[2] == 100 && rgba[3] == 255);
    CHECK(rgba[(w * h - 1) * 4] == 100);

    uv[1] = 255;   // strong V in the first 2x2 block: red goes up, blue doesn't
    yuvToRgba(y, stride, uv, uv + 1, stride, pixel, w, h, rgba);
    CHECK(rgba[0] > 240 && rgba[2] == 100);
    CHECK(rgba[8] == 100);                 // the next block isn't touched
}

static void test_packet_is_small_and_plain() {
    HandPoint hands[2] = {{true, 0.91f, {0.1f, -0.2f, -0.45f}}, {false, 0.8f, {-0.12f, 0.0f, -0.5f}}};
    std::string p = handPacket(7, 42.5f, 15.0f, 23.4f, hands, 2, "bad \"model\"");
    CHECK(p == "{\"seq\":7,\"age_ms\":42.5,\"rate\":15.0,\"ms\":23.4,\"hands\":["
               "{\"side\":\"left\",\"score\":0.91,\"p\":[0.100,-0.200,-0.450]},"
               "{\"side\":\"right\",\"score\":0.80,\"p\":[-0.120,0.000,-0.500]}],\"problem\":\"bad model\"}");
    CHECK(handPacket(1, 0, 0, 0, nullptr, 0, "").find("\"hands\":[]") != std::string::npos);
}

int main() {
    test_recovers_a_hand_at_arms_length();
    test_upside_down_phone_gives_the_same_answer_in_the_head();
    test_camera_offset_moves_the_hand_into_head_space();
    test_model_left_is_really_the_right_on_a_rear_camera();
    test_a_pixel_of_noise_moves_a_near_hand_only_a_little();
    test_impossible_distances_are_dropped();
    test_yuv_grey_and_colour();
    test_packet_is_small_and_plain();
    if (failures) {
        std::printf("%d hand geometry check(s) failed\n", failures);
        return EXIT_FAILURE;
    }
    std::printf("hand geometry: all passed\n");
    return EXIT_SUCCESS;
}
