#include "reprojection.h"

#include <cmath>

#include <android/log.h>

// utils.h defines its log() in the header, so it can only be included by alvr_main.cpp.
#define RLOGI(...) __android_log_print(ANDROID_LOG_INFO, "PhoneVR6DoF", __VA_ARGS__)
#define RLOGE(...) __android_log_print(ANDROID_LOG_ERROR, "PhoneVR6DoF", __VA_ARGS__)

namespace {

// Full-screen triangle from gl_VertexID; no vertex buffer needed.
const char *kVertex = R"(#version 300 es
out vec2 vUv;
void main() {
    vec2 p = vec2(float((gl_VertexID << 1) & 2), float(gl_VertexID & 2));
    vUv = p;
    gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);
}
)";

// Same maths as pvr6::reprojectUV (pose_math.h, unit tested on the PC).
const char *kFragment = R"(#version 300 es
precision highp float;
in vec2 vUv;
out vec4 fragColor;
uniform sampler2D uSrc;
uniform mat3 uRot;      // frame^-1 * now
uniform vec4 uCur;      // tan(left), tan(right), tan(down), tan(up) of the current view
uniform vec4 uSrcTan;   // same for the frame
void main() {
    vec3 d = vec3(mix(uCur.x, uCur.y, vUv.x), mix(uCur.z, uCur.w, vUv.y), -1.0);
    vec3 s = uRot * d;
    if (s.z > -1e-4) { fragColor = vec4(0.0, 0.0, 0.0, 1.0); return; }
    vec2 t = s.xy / -s.z;
    vec2 uv = (t - uSrcTan.xz) / (uSrcTan.yw - uSrcTan.xz);
    if (any(lessThan(uv, vec2(0.0))) || any(greaterThan(uv, vec2(1.0)))) {
        fragColor = vec4(0.0, 0.0, 0.0, 1.0);   // the PC never rendered this part
        return;
    }
    fragColor = texture(uSrc, uv);
}
)";

GLuint compile(GLenum type, const char *src) {
    GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, nullptr);
    glCompileShader(s);
    GLint ok = 0;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[512] = {};
        glGetShaderInfoLog(s, sizeof(log), nullptr, log);
        RLOGE("reprojection shader compile failed: %s", log);
        glDeleteShader(s);
        return 0;
    }
    return s;
}

void tans(const pvr6::Fov &f, float out[4]) {
    out[0] = std::tan(f.left);
    out[1] = std::tan(f.right);
    out[2] = std::tan(f.down);
    out[3] = std::tan(f.up);
}

}   // namespace

bool Reprojector::create(int width, int height) {
    destroy();
    GLuint vs = compile(GL_VERTEX_SHADER, kVertex);
    GLuint fs = compile(GL_FRAGMENT_SHADER, kFragment);
    if (!vs || !fs) return false;
    program_ = glCreateProgram();
    glAttachShader(program_, vs);
    glAttachShader(program_, fs);
    glLinkProgram(program_);
    glDeleteShader(vs);
    glDeleteShader(fs);
    GLint linked = 0;
    glGetProgramiv(program_, GL_LINK_STATUS, &linked);
    if (!linked) {
        RLOGE("reprojection program link failed");
        destroy();
        return false;
    }
    uRot_ = glGetUniformLocation(program_, "uRot");
    uCur_ = glGetUniformLocation(program_, "uCur");
    uSrcTan_ = glGetUniformLocation(program_, "uSrcTan");
    uSrc_ = glGetUniformLocation(program_, "uSrc");
    glGenVertexArrays(1, &vao_);

    width_ = width;
    height_ = height;
    glGenTextures(2, target_);
    glGenFramebuffers(2, fbo_);
    for (int eye = 0; eye < 2; eye++) {
        glBindTexture(GL_TEXTURE_2D, target_[eye]);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
        glBindFramebuffer(GL_FRAMEBUFFER, fbo_[eye]);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, target_[eye], 0);
        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
            RLOGE("reprojection framebuffer %d incomplete", eye);
            glBindFramebuffer(GL_FRAMEBUFFER, 0);
            destroy();
            return false;
        }
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    RLOGI("reprojection ready (%dx%d per eye)", width, height);
    return true;
}

void Reprojector::destroy() {
    if (program_) glDeleteProgram(program_);
    if (vao_) glDeleteVertexArrays(1, &vao_);
    if (fbo_[0]) glDeleteFramebuffers(2, fbo_);
    if (target_[0]) glDeleteTextures(2, target_);
    program_ = vao_ = 0;
    fbo_[0] = fbo_[1] = target_[0] = target_[1] = 0;
}

GLuint Reprojector::apply(int eye, GLuint srcTexture, const pvr6::Quat &rotation,
                          const pvr6::Fov &currentFov, const pvr6::Fov &frameFov) {
    if (!program_) return srcTexture;
    float rot[9], cur[4], src[4];
    pvr6::toMat3ColumnMajor(rotation, rot);
    tans(currentFov, cur);
    tans(frameFov, src);

    glBindFramebuffer(GL_FRAMEBUFFER, fbo_[eye]);
    glViewport(0, 0, width_, height_);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glDisable(GL_SCISSOR_TEST);
    glDisable(GL_CULL_FACE);
    glUseProgram(program_);
    glUniformMatrix3fv(uRot_, 1, GL_FALSE, rot);
    glUniform4fv(uCur_, 1, cur);
    glUniform4fv(uSrcTan_, 1, src);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, srcTexture);
    glUniform1i(uSrc_, 0);
    glBindVertexArray(vao_);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glBindVertexArray(0);
    glUseProgram(0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    return target_[eye];
}
