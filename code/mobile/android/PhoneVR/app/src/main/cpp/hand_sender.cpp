#include "hand_sender.h"

#include <android/log.h>
#include <arpa/inet.h>
#include <cstring>
#include <ctime>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#define ALOGI(...) __android_log_print(ANDROID_LOG_INFO, "PhoneVR6DoF", __VA_ARGS__)
#define ALOGE(...) __android_log_print(ANDROID_LOG_ERROR, "PhoneVR6DoF", __VA_ARGS__)

namespace {

constexpr int kPort = 47932;
constexpr char kHello[] = "onetrack-hello";
constexpr char kRecenter[] = "onetrack-recenter";
constexpr int64_t kSecondNs = 1000000000LL;
constexpr int64_t kHelloGoneNs = 5 * kSecondNs;
constexpr int64_t kRecenterGapNs = kSecondNs;   // OneTrack sends it a few times, in case Wi-Fi loses one

int64_t bootTimeNs() {
    timespec t{};
    clock_gettime(CLOCK_BOOTTIME, &t);
    return int64_t(t.tv_sec) * kSecondNs + t.tv_nsec;
}

}   // namespace

void HandSender::start() {
    if (running_.exchange(true)) return;
    socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in me{};
    me.sin_family = AF_INET;
    me.sin_addr.s_addr = htonl(INADDR_ANY);
    me.sin_port = htons(kPort);
    int on = 1;
    setsockopt(socket_, SOL_SOCKET, SO_REUSEADDR, &on, sizeof on);
    timeval wait{0, 300000};
    setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, &wait, sizeof wait);
    if (socket_ < 0 || bind(socket_, reinterpret_cast<sockaddr *>(&me), sizeof me) != 0) {
        ALOGE("hands: couldn't open port %d", kPort);
        if (socket_ >= 0) close(socket_);
        socket_ = -1;
        running_ = false;
        return;
    }
    thread_ = std::thread(&HandSender::loop, this);
}

void HandSender::stop() {
    if (running_.exchange(false) && thread_.joinable()) thread_.join();   // <= one receive wait (300 ms)
    if (socket_ >= 0) close(socket_);
    socket_ = -1;
    std::lock_guard<std::mutex> lock(mutex_);
    lastHelloNs_ = 0;
}

void HandSender::loop() {
    char buf[32];
    while (running_.load()) {
        sockaddr_in from{};
        socklen_t size = sizeof from;
        ssize_t n = recvfrom(socket_, buf, sizeof buf, 0, reinterpret_cast<sockaddr *>(&from), &size);
        int64_t now = bootTimeNs();
        std::lock_guard<std::mutex> lock(mutex_);
        if (n == ssize_t(sizeof kHello - 1) && std::memcmp(buf, kHello, n) == 0) {
            if (now - lastHelloNs_ > kHelloGoneNs) {
                char ip[INET_ADDRSTRLEN] = {};
                inet_ntop(AF_INET, &from.sin_addr, ip, sizeof ip);
                ALOGI("hands: OneTrack found at %s", ip);
            }
            peer_ = from;
            lastHelloNs_ = now;
        } else if (n == ssize_t(sizeof kRecenter - 1) && std::memcmp(buf, kRecenter, n) == 0 &&
                   now - lastRecenterNs_ > kRecenterGapNs) {
            ALOGI("OneTrack asked for a recenter");
            lastRecenterNs_ = now;
            if (onRecenter) onRecenter();
        }
        // Keeps the page honest when no pictures are being checked at all.
        if (now - lastSendNs_ > kSecondNs && now - lastCheckNs_ > kSecondNs) {
            checks_ = 0;
            rate_ = 0;
            send(0, nullptr, 0);
        }
    }
}

void HandSender::send(float ageMs, const pvr6::HandPoint *hands, int count) {
    int64_t now = bootTimeNs();
    if (socket_ < 0 || lastHelloNs_ == 0 || now - lastHelloNs_ > kHelloGoneNs) return;
    std::string packet = pvr6::handPacket(++seq_, ageMs, rate_, msPerCheck_, hands, count, problem_);
    sendto(socket_, packet.data(), packet.size(), MSG_DONTWAIT, reinterpret_cast<sockaddr *>(&peer_), sizeof peer_);
    lastSendNs_ = now;
}

void HandSender::report(float pictureAgeMs, float msPerCheck, const pvr6::HandPoint *hands, int count) {
    int64_t now = bootTimeNs();
    std::lock_guard<std::mutex> lock(mutex_);
    msPerCheck_ = msPerCheck_ == 0 ? msPerCheck : 0.9f * msPerCheck_ + 0.1f * msPerCheck;
    checks_++;
    lastCheckNs_ = now;
    if (rateStartNs_ == 0) rateStartNs_ = now;
    if (now - rateStartNs_ >= kSecondNs) {
        rate_ = checks_ * float(kSecondNs) / float(now - rateStartNs_);
        ALOGI("hands: %.1f checks/s, %.1f ms each, %d in view, picture %.0f ms old", rate_, msPerCheck_, count, pictureAgeMs);
        checks_ = 0;
        rateStartNs_ = now;
    }
    send(pictureAgeMs, hands, count);
}

void HandSender::setProblem(const std::string &problem) {
    std::lock_guard<std::mutex> lock(mutex_);
    problem_ = problem;
}
