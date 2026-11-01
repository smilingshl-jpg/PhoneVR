// Turns what the hand model finds in a camera picture into a wrist position in front of the head (PhoneVR 6DoF
// fork). Header-only and free of Android types so it can be unit tested on a PC (app/src/test/cpp).
//
// The model gives each palm point twice: where it is in the picture, and where it is in metres around the hand's
// own middle. Sliding the real-size hand along the view until it lines up with the picture gives its distance.
#pragma once

#include <cstdint>
#include <cstdio>
#include <string>

#include "pose_math.h"

namespace pvr6 {

constexpr int kPalmPoints = 6;   // wrist + finger bases: the fingers are wrapped around a Joy-Con, so often hidden
constexpr int kHandValues = 2 + kPalmPoints * 5;   // what Java sends per hand, see solveHand
constexpr float kNearestHandM = 0.08f;             // closer or further than this is a bad match, not a hand
constexpr float kFurthestHandM = 1.5f;

struct Lens {
    float fx = 0, fy = 0, cx = 0, cy = 0;   // pixels, in the camera image
    int width = 0, height = 0;
};

struct HandPoint {
    bool left = false;
    float score = 0;
    Vec3 head;   // wrist in head space: +X right, +Y up, -Z forward, metres
};

// Solves (W + t) projecting onto the picture for the translation t, in picture axes (x right, y down, z away).
// With a = x/z: tx - a tz = a Wz - Wx, and the same for b and y. Three unknowns, so normal equations + Cramer.
inline bool solveTranslation(const float rays[][2], const float world[][3], int n, float t[3]) {
    float A[3][3] = {}, B[3] = {};
    auto addRow = [&](float r0, float r1, float r2, float rhs) {
        const float r[3] = {r0, r1, r2};
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) A[i][j] += r[i] * r[j];
            B[i] += r[i] * rhs;
        }
    };
    for (int i = 0; i < n; i++) {
        float a = rays[i][0], b = rays[i][1];
        addRow(1, 0, -a, a * world[i][2] - world[i][0]);
        addRow(0, 1, -b, b * world[i][2] - world[i][1]);
    }
    auto det = [](const float m[3][3]) {
        return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
               m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
               m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    };
    float d = det(A);
    if (std::fabs(d) < 1e-9f) return false;
    for (int k = 0; k < 3; k++) {
        float m[3][3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) m[i][j] = j == k ? B[i] : A[i][j];
        t[k] = det(m) / d;
    }
    return true;
}

// One hand from Java: [modelSaysLeft, score, then per palm point (wrist first): pictureX, pictureY in 0..1,
// worldX, worldY, worldZ]. `displayRotation` is Android's Surface.ROTATION_* (1 = 90, 3 = 270).
// `cameraToEyes`: where the eyes are from the camera, as in FusionConfig.
inline bool solveHand(const float *values, const Lens &lens, int displayRotation, Vec3 cameraToEyes, HandPoint *out) {
    if (lens.fx <= 0 || lens.fy <= 0 || lens.width <= 0 || lens.height <= 0) return false;
    float rays[kPalmPoints][2], world[kPalmPoints][3];
    for (int i = 0; i < kPalmPoints; i++) {
        const float *p = values + 2 + i * 5;
        rays[i][0] = (p[0] * lens.width - lens.cx) / lens.fx;
        rays[i][1] = (p[1] * lens.height - lens.cy) / lens.fy;
        for (int k = 0; k < 3; k++) world[i][k] = p[2 + k];
    }
    float t[3];
    if (!solveTranslation(rays, world, kPalmPoints, t)) return false;
    Vec3 wrist{world[0][0] + t[0], world[0][1] + t[1], world[0][2] + t[2]};
    if (wrist.z < kNearestHandM || wrist.z > kFurthestHandM) return false;

    // Picture axes to display axes. The sensor's picture is upright at rotation 90, upside down at 270.
    Vec3 camera = displayRotation == 3 ? Vec3{-wrist.x, wrist.y, -wrist.z} : Vec3{wrist.x, -wrist.y, -wrist.z};
    out->head = sub(camera, cameraToEyes);
    // The model expects a mirrored selfie picture. A rear camera isn't mirrored, so its left is really the right.
    out->left = values[0] < 0.5f;
    out->score = values[1];
    return true;
}

// Camera pictures are full-range YUV. Writes RGBA, with the planes as ARCore gives them (U and V can share a plane).
inline void yuvToRgba(const uint8_t *y, int yStride, const uint8_t *u, const uint8_t *v, int uvStride, int uvPixelStride,
                      int width, int height, uint8_t *rgba) {
    auto clamp8 = [](int c) { return uint8_t(c < 0 ? 0 : c > 255 ? 255 : c); };
    for (int row = 0; row < height; row++) {
        const uint8_t *yRow = y + row * yStride;
        const uint8_t *uRow = u + (row / 2) * uvStride;
        const uint8_t *vRow = v + (row / 2) * uvStride;
        uint8_t *out = rgba + row * width * 4;
        for (int col = 0; col < width; col++) {
            int Y = yRow[col];
            int U = uRow[(col / 2) * uvPixelStride] - 128;
            int V = vRow[(col / 2) * uvPixelStride] - 128;
            *out++ = clamp8(Y + (359 * V >> 8));
            *out++ = clamp8(Y - ((88 * U + 183 * V) >> 8));
            *out++ = clamp8(Y + (454 * U >> 8));
            *out++ = 255;
        }
    }
}

// What goes to OneTrack over Wi-Fi. No pictures, only where the hands are.
inline std::string handPacket(uint32_t seq, float ageMs, float checksPerS, float msPerCheck, const HandPoint *hands,
                              int count, const std::string &problem) {
    char buf[160];
    std::snprintf(buf, sizeof buf, "{\"seq\":%u,\"age_ms\":%.1f,\"rate\":%.1f,\"ms\":%.1f,\"hands\":[", (unsigned) seq,
                  ageMs, checksPerS, msPerCheck);
    std::string out = buf;
    for (int i = 0; i < count; i++) {
        const HandPoint &h = hands[i];
        std::snprintf(buf, sizeof buf, "%s{\"side\":\"%s\",\"score\":%.2f,\"p\":[%.3f,%.3f,%.3f]}", i ? "," : "",
                      h.left ? "left" : "right", h.score, h.head.x, h.head.y, h.head.z);
        out += buf;
    }
    out += "],\"problem\":\"";
    for (char c : problem)
        if (c != '"' && c != '\\' && c >= ' ') out += c;
    out += "\"}";
    return out;
}

}   // namespace pvr6
