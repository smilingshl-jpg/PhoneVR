// Sends the hands found by the camera to OneTrack on the PC (PhoneVR 6DoF fork).
//
// The phone doesn't know the PC's address, so it waits on a UDP port for OneTrack's hello and answers its sender.
// With no hello for a few seconds it goes quiet. Only positions are sent, never pictures.
// It runs whenever the app is open, even with Hand tracking off, because OneTrack's Recenter arrives on the same port.
#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <netinet/in.h>
#include <string>
#include <thread>

#include "hand_geometry.h"

class HandSender {
  public:
    void start();
    void stop();

    // Result of one check, from the hand tracker's thread.
    void report(float pictureAgeMs, float msPerCheck, const pvr6::HandPoint *hands, int count);
    void setProblem(const std::string &problem);

    // OneTrack's Recenter reached the phone. Runs on the receive thread, so keep it short.
    std::function<void()> onRecenter;

  private:
    void loop();
    void send(float ageMs, const pvr6::HandPoint *hands, int count);

    std::thread thread_;
    std::atomic<bool> running_{false};
    int socket_ = -1;

    std::mutex mutex_;   // everything below
    sockaddr_in peer_{};
    int64_t lastHelloNs_ = 0;
    int64_t lastSendNs_ = 0;
    int64_t lastCheckNs_ = 0;
    int64_t rateStartNs_ = 0;
    int64_t lastRecenterNs_ = 0;
    uint32_t seq_ = 0;
    int checks_ = 0;
    float rate_ = 0, msPerCheck_ = 0;
    std::string problem_;
};
