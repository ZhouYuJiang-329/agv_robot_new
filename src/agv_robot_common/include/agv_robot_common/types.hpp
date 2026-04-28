/**
 * @file types.hpp
 * @brief 公共类型定义
 */

#ifndef AGV_ROBOT_COMMON_TYPES_HPP_
#define AGV_ROBOT_COMMON_TYPES_HPP_

#include <array>
#include <vector>
#include <string>
#include <chrono>
#include <optional>
#include <memory>

namespace agv_robot {

// 基础类型定义
using JointPositions = std::vector<double>;
using JointVelocities = std::vector<double>;
using JointEfforts = std::vector<double>;

// 位姿定义
struct Pose {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double roll{0.0};   // rad
    double pitch{0.0};  // rad
    double yaw{0.0};    // rad
};

struct Quaternion {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double w{1.0};
};

struct PoseQuaternion {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    Quaternion orientation;
};

// 速度定义
struct Twist {
    double linear_x{0.0};
    double linear_y{0.0};
    double linear_z{0.0};
    double angular_x{0.0};
    double angular_y{0.0};
    double angular_z{0.0};
};

// 电池状态
struct BatteryStatus {
    double voltage{0.0};        // V
    double current{0.0};        // A
    double percentage{0.0};     // 0-100
    double remaining_time{0.0}; // minutes
    bool is_charging{false};
};

// 硬件状态
enum class HardwareState {
    UNKNOWN = 0,
    INITIALIZING = 1,
    READY = 2,
    RUNNING = 3,
    ERROR = 4,
    EMERGENCY_STOP = 5
};

struct HardwareStatus {
    std::string device_name;
    HardwareState state{HardwareState::UNKNOWN};
    std::string error_message;
    std::chrono::steady_clock::time_point last_update;
};

// 任务状态
enum class TaskStatus {
    PENDING = 0,
    RUNNING = 1,
    PAUSED = 2,
    COMPLETED = 3,
    FAILED = 4,
    CANCELLED = 5
};

// 检测结果
struct Detection {
    std::string object_id;
    std::string object_class;
    float confidence{0.0f};
    Pose pose;
    float width{0.0f};
    float height{0.0f};
    float depth{0.0f};
};

// 障碍物信息
struct Obstacle {
    std::string id;
    Pose pose;
    double distance{0.0};
    double confidence{0.0};
};

// 轨迹点
struct TrajectoryPoint {
    JointPositions positions;
    JointVelocities velocities;
    JointEfforts efforts;
    double time_from_start{0.0};  // seconds
};

// 轨迹
struct Trajectory {
    std::vector<TrajectoryPoint> points;
    std::chrono::steady_clock::time_point start_time;
};

// 超声传感器配置
struct UltrasoundSensorConfig {
    uint8_t sensor_id{0};
    double mounting_angle{0.0};  // rad, 相对于机器人前方
    double mounting_height{0.0}; // m
    double min_range{0.05};      // m
    double max_range{4.0};       // m
    double fov{0.26};            // rad (15度)
};

// 相机内参
struct CameraIntrinsics {
    double fx{0.0};
    double fy{0.0};
    double cx{0.0};
    double cy{0.0};
    double k1{0.0};
    double k2{0.0};
    double p1{0.0};
    double p2{0.0};
    double k3{0.0};
    int width{0};
    int height{0};
};

// 手眼标定结果
struct HandEyeCalibration {
    Pose camera_to_gripper;
    double reprojection_error{0.0};
    int calibration_samples{0};
};

// 错误码
enum class ErrorCode {
    SUCCESS = 0,
    INVALID_PARAMETER = -1,
    HARDWARE_NOT_READY = -2,
    PLANNING_FAILED = -3,
    EXECUTION_FAILED = -4,
    TIMEOUT = -5,
    COLLISION_DETECTED = -6,
    JOINT_LIMIT_VIOLATION = -7,
    COMMUNICATION_ERROR = -8,
    EMERGENCY_STOP = -9
};

// 结果模板
template<typename T>
struct Result {
    ErrorCode code{ErrorCode::SUCCESS};
    std::string message;
    std::optional<T> value;
    
    bool isSuccess() const { return code == ErrorCode::SUCCESS; }
    bool isFailure() const { return code != ErrorCode::SUCCESS; }
    
    static Result<T> success(const T& val) {
        Result<T> r;
        r.code = ErrorCode::SUCCESS;
        r.value = val;
        return r;
    }
    
    static Result<T> failure(ErrorCode c, const std::string& msg) {
        Result<T> r;
        r.code = c;
        r.message = msg;
        return r;
    }
};

// 特化 void 版本
template<>
struct Result<void> {
    ErrorCode code{ErrorCode::SUCCESS};
    std::string message;
    
    bool isSuccess() const { return code == ErrorCode::SUCCESS; }
    bool isFailure() const { return code != ErrorCode::SUCCESS; }
    
    static Result<void> success() {
        Result<void> r;
        r.code = ErrorCode::SUCCESS;
        return r;
    }
    
    static Result<void> failure(ErrorCode c, const std::string& msg) {
        Result<void> r;
        r.code = c;
        r.message = msg;
        return r;
    }
};

} // namespace agv_robot

#endif // AGV_ROBOT_COMMON_TYPES_HPP_