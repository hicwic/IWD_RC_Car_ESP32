#include "metrics.h"
#include <sstream>

void Metrics::update(const MetricsData& newData) {
    std::lock_guard<std::mutex> lock(mtx_);
    data_ = newData;
}

MetricsData Metrics::get() {
    std::lock_guard<std::mutex> lock(mtx_);
    return data_;
}

std::string Metrics::to_json() {
    std::lock_guard<std::mutex> lock(mtx_);
    std::ostringstream oss;
    oss << "{";
    oss << "\"throttlePercent\":" << data_.throttlePercent << ",";
    oss << "\"dirPercent\":" << data_.dirPercent << ",";
    oss << "\"mixPercent\":" << data_.mixPercent << ",";
    oss << "\"velocityReductionRate\":" << data_.velocityReductionRate << ",";
    oss << "\"targetVelocity\":" << data_.targetVelocity << ",";
    oss << "\"baseVelocity\":" << data_.baseVelocity << ",";
    oss << "\"leftVelocity\":" << data_.leftVelocity << ",";
    oss << "\"rightVelocity\":" << data_.rightVelocity << ",";
    oss << "\"dshotValueLeft\":" << data_.dshotValueLeft << ",";
    oss << "\"dshotValueRight\":" << data_.dshotValueRight << ",";
    oss << "\"reverseMode\":" << (data_.reverseMode ? "true" : "false") << ",";
    oss << "\"readyForReverse\":" << (data_.readyForReverse ? "true" : "false") << ",";
    oss << "\"ramping\":" << (data_.ramping ? "true" : "false") << ",";
    oss << "\"deltaTimeS\":" << data_.deltaTimeS << ",";
    oss << "\"loopTimeMs\":" << data_.loopTimeMs;
    oss << "}";
    return oss.str();
}