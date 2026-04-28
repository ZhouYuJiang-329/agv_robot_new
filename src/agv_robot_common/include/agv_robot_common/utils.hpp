/**
 * @file utils.hpp
 * @brief 工具函数
 */

#ifndef AGV_ROBOT_COMMON_UTILS_HPP_
#define AGV_ROBOT_COMMON_UTILS_HPP_

#include <cmath>
#include <vector>
#include <algorithm>
#include <chrono>
#include <thread>
#include "types.hpp"

namespace agv_robot {
namespace utils {

// 数学工具
namespace math {

constexpr double PI = 3.14159265358979323846;
constexpr double DEG2RAD = PI / 180.0;
constexpr double RAD2DEG = 180.0 / PI;

inline double deg2rad(double deg) { return deg * DEG2RAD; }
inline double rad2deg(double rad) { return rad * RAD2DEG; }

// 角度归一化到 [-pi, pi]
inline double normalizeAngle(double angle) {
    while (angle > PI) angle -= 2 * PI;
    while (angle < -PI) angle += 2 * PI;
    return angle;
}

// 线性插值
inline double lerp(double a, double b, double t) {
    return a + (b - a) * std::clamp(t, 0.0, 1.0);
}

// 向量线性插值
inline std::vector<double> lerpVector(
    const std::vector<double>& a,
    const std::vector<double>& b,
    double t) {
    std::vector<double> result;
    result.reserve(a.size());
    for (size_t i = 0; i < a.size(); ++i) {
        result.push_back(lerp(a[i], b[i], t));
    }
    return result;
}

// 欧氏距离
inline double euclideanDistance(const Pose& a, const Pose& b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

} // namespace math

// 时间工具
namespace time {

using Clock = std::chrono::steady_clock;
using TimePoint = Clock::time_point;
using Duration = Clock::duration;

inline TimePoint now() { return Clock::now(); }

inline double toSeconds(Duration d) {
    return std::chrono::duration<double>(d).count();
}

inline Duration fromSeconds(double s) {
    return std::chrono::duration_cast<Duration>(
        std::chrono::duration<double>(s));
}

inline void sleep(double seconds) {
    std::this_thread::sleep_for(fromSeconds(seconds));
}

// 频率控制
class Rate {
public:
    explicit Rate(double frequency_hz)
        : period_(fromSeconds(1.0 / frequency_hz))
        , last_time_(now()) {}
    
    void sleep() {
        auto next_time = last_time_ + period_;
        std::this_thread::sleep_until(next_time);
        last_time_ = next_time;
    }
    
private:
    Duration period_;
    TimePoint last_time_;
};

} // namespace time

// 容器工具
namespace container {

// 限制向量元素到范围
template<typename T>
std::vector<T> clampVector(const std::vector<T>& vec, T min_val, T max_val) {
    std::vector<T> result;
    result.reserve(vec.size());
    for (const auto& v : vec) {
        result.push_back(std::clamp(v, min_val, max_val));
    }
    return result;
}

// 检查向量是否在范围内
template<typename T>
bool isInRange(const std::vector<T>& vec, T min_val, T max_val) {
    return std::all_of(vec.begin(), vec.end(),
        [min_val, max_val](T v) { return v >= min_val && v <= max_val; });
}

} // namespace container

// 字符串工具
namespace string {

inline std::string toLower(const std::string& str) {
    std::string result = str;
    std::transform(result.begin(), result.end(), result.begin(), ::tolower);
    return result;
}

inline std::string toUpper(const std::string& str) {
    std::string result = str;
    std::transform(result.begin(), result.end(), result.begin(), ::toupper);
    return result;
}

inline bool startsWith(const std::string& str, const std::string& prefix) {
    return str.size() >= prefix.size() &&
           str.compare(0, prefix.size(), prefix) == 0;
}

inline bool endsWith(const std::string& str, const std::string& suffix) {
    return str.size() >= suffix.size() &&
           str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
}

} // namespace string

// 线程安全队列 (简化版)
template<typename T>
class ThreadSafeQueue {
public:
    void push(const T& item) {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.push(item);
        cond_.notify_one();
    }
    
    bool pop(T& item, std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!cond_.wait_for(lock, timeout, [this] { return !queue_.empty(); })) {
            return false;
        }
        item = queue_.front();
        queue_.pop();
        return true;
    }
    
    bool empty() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }
    
    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }
    
private:
    mutable std::mutex mutex_;
    std::condition_variable cond_;
    std::queue<T> queue_;
};

} // namespace utils
} // namespace agv_robot

#endif // AGV_ROBOT_COMMON_UTILS_HPP_