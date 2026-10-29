// Rotational reprojection ("timewarp") for PhoneVR 6DoF.
//
// The Cardboard SDK shows each streamed frame exactly as the PC rendered it, even when the head has turned
// since (late frame = judder). This pass re-aims each eye image to the newest head rotation before the
// Cardboard lens-distortion step: one full-screen triangle per eye into an intermediate texture.
#pragma once

#include <GLES3/gl3.h>

#include "pose_math.h"

class Reprojector {
  public:
    // Call with the GL context current. width/height = stream view size (one eye).
    bool create(int width, int height);
    void destroy();
    // The GL context was lost (its objects are already gone): drop the names without deleting anything,
    // so a later create()/destroy() can't delete unrelated objects that reuse the same numbers.
    void abandon() {
        program_ = vao_ = 0;
        fbo_[0] = fbo_[1] = target_[0] = target_[1] = 0;
    }
    bool ready() const { return program_ != 0; }

    // Renders `srcTexture` (rendered for the frame's eye orientation) re-aimed by `rotation`
    // (pvr6::reprojectionRotation(frame, now)). Returns the texture to show for this eye.
    GLuint apply(int eye, GLuint srcTexture, const pvr6::Quat &rotation, const pvr6::Fov &currentFov,
                 const pvr6::Fov &frameFov);

  private:
    GLuint program_ = 0;
    GLuint vao_ = 0;
    GLuint fbo_[2] = {0, 0};
    GLuint target_[2] = {0, 0};
    GLint uRot_ = -1, uCur_ = -1, uSrcTan_ = -1, uSrc_ = -1;
    int width_ = 0, height_ = 0;
};
