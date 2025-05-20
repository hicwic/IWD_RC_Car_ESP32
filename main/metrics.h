#pragma once
#include <mutex>
#include <string>

struct MetricsData {
    int throttlePercent = 0;
    int dirPercent = 0;
    int mixPercent = 0;
    int velocityReductionRate = 0;
    float targetVelocity = 0.f;
    float baseVelocity = 0.f;
    float leftVelocity = 0.f;
    float rightVelocity = 0.f;
    int dshotValueLeft = 0;
    int dshotValueRight = 0;
    bool reverseMode = false;
    bool readyForReverse = false;
    bool ramping = false;
    float deltaTimeS = 0.f;
    float loopTimeMs = 0.f;
};

class Metrics {
public:
    void update(const MetricsData& newData);
    MetricsData get();
    std::string to_json();

private:
    MetricsData data_{};
    std::mutex mtx_;
};
