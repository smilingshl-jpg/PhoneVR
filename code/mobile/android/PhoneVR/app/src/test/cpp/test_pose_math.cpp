// Host-side tests for pose_math.h (no Android needed). Run: bash run_tests.sh
#include "../../main/cpp/pose_math.h"

#include <cstdio>
#include <cstdlib>

using namespace pvr6;

static int failures = 0;
#define CHECK(cond)                                                                                \
    do {                                                                                           \
        if (!(cond)) {                                                                             \
            std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);                            \
            failures++;                                                                            \
        }                                                                                          \
    } while (0)
static bool near(float a, float b, float eps = 1e-4f) { return std::fabs(a - b) < eps; }
static const float DEG = kPi / 180.0f;

static void test_rotate_and_mul() {
    Quat yaw90 = yawOnly(90 * DEG);   // turn left
    Vec3 f = rotate(yaw90, {0, 0, -1});
    CHECK(near(f.x, -1) && near(f.z, 0));   // forward now points left (-X)
    Quat back = mul(yaw90, yaw90);
    Vec3 b = rotate(back, {0, 0, -1});
    CHECK(near(b.z, 1));
    CHECK(near(yawOf(yaw90), 90 * DEG));
}

static void test_angular_velocity() {
    Quat q0 = yawOnly(0), q1 = yawOnly(10 * DEG);
    Vec3 w = angularVelocity(q0, q1, 0.1f);   // 100 deg/s about +Y
    CHECK(near(w.y, 100 * DEG, 1e-3f) && near(w.x, 0) && near(w.z, 0));
    Vec3 zero = angularVelocity(q1, q1, 0.01f);
    CHECK(near(length(zero), 0));
}

static void test_matrix_matches_quaternion() {
    Quat q = normalize(mul(fromAxisAngle({1, 0, 0}, 0.3f), fromAxisAngle({0, 1, 0}, -0.7f)));
    float m[9];
    toMat3ColumnMajor(q, m);
    Vec3 v{0.2f, -0.5f, 0.9f};
    Vec3 byQuat = rotate(q, v);
    Vec3 byMat{m[0] * v.x + m[3] * v.y + m[6] * v.z, m[1] * v.x + m[4] * v.y + m[7] * v.z,
               m[2] * v.x + m[5] * v.y + m[8] * v.z};
    CHECK(near(byQuat.x, byMat.x) && near(byQuat.y, byMat.y) && near(byQuat.z, byMat.z));
}

static void test_reprojection_identity_and_yaw() {
    Fov fov;   // symmetric ~77 deg
    float u, v;
    // no head movement since the frame: every pixel samples itself
    CHECK(reprojectUV(reprojectionRotation(yawOnly(0), yawOnly(0)), fov, fov, 0.3f, 0.7f, &u, &v));
    CHECK(near(u, 0.3f) && near(v, 0.7f));

    // the head has turned LEFT since the frame was rendered: the centre of the current view looks at
    // what was left of centre in the frame, so it must sample u < 0.5 (v unchanged)
    Quat frame = yawOnly(0), now = yawOnly(5 * DEG);
    CHECK(reprojectUV(reprojectionRotation(frame, now), fov, fov, 0.5f, 0.5f, &u, &v));
    CHECK(u < 0.5f && near(v, 0.5f));

    // looked UP since the frame: centre samples above centre (v > 0.5, v points up)
    Quat up = fromAxisAngle({1, 0, 0}, 5 * DEG);
    CHECK(reprojectUV(reprojectionRotation(frame, up), fov, fov, 0.5f, 0.5f, &u, &v));
    CHECK(v > 0.5f && near(u, 0.5f));

    // a 5 deg turn moves the centre by tan(5 deg)/(2 tan(fov)) of the width
    float expected = 0.5f - std::tan(5 * DEG) / (std::tan(fov.right) - std::tan(fov.left));
    reprojectUV(reprojectionRotation(frame, now), fov, fov, 0.5f, 0.5f, &u, &v);
    CHECK(near(u, expected, 1e-3f));
}

static void test_reprojection_behind_is_rejected() {
    Fov fov;
    float u, v;
    CHECK(!reprojectUV(reprojectionRotation(yawOnly(0), yawOnly(180 * DEG)), fov, fov, 0.5f, 0.5f, &u, &v));
}

int main() {
    test_rotate_and_mul();
    test_angular_velocity();
    test_matrix_matches_quaternion();
    test_reprojection_identity_and_yaw();
    test_reprojection_behind_is_rejected();
    if (failures) {
        std::printf("%d check(s) failed\n", failures);
        return 1;
    }
    std::printf("pose_math: all tests passed\n");
    return 0;
}
