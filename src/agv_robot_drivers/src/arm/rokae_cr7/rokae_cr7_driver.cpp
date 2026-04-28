/**
 * @file rokae_cr7_driver.cpp
 * @brief 珞石CR7机械臂驱动实现 - 基于官方SDK
 */

#include "agv_robot_drivers/arm/rokae_cr7/rokae_cr7_driver.hpp"

#include <chrono>
#include <thread>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <sstream>

namespace agv_robot {
namespace drivers {

// ==================== Constructor & Destructor ====================

RokaeCR7Driver::RokaeCR7Driver(const AgvConfig& config)
    : robot_ip_(config.robot_ip)
    , robot_name_(config.robot_name)
    , robot_handle_(std::make_unique<rokae::xMateRobot>(robot_ip_))
    , is_connected_(false)
    , is_activated_(false)
    , nrt_executing_(false) {
}

RokaeCR7Driver::~RokaeCR7Driver() {
    if (is_activated_) {
        deactivate();
    }
    if (is_connected_) {
        disconnect();
    }
}

// ==================== Connection ====================

bool RokaeCR7Driver::connect() {
    if (is_connected_) {
        return true;
    }

    try {
        // 清除错误码
        ec_.clear();

        // 连接到机械臂
        robot_handle_->connectToRobot(robot_ip_);

        std::cout << "[RokaeCR7Driver] Connected to robot at " << robot_ip_ << std::endl;
        is_connected_ = true;
        return true;

    } catch (const std::exception& e) {
        std::cerr << "[RokaeCR7Driver] Failed to connect: " << e.what() << std::endl;
        is_connected_ = false;
        return false;
    }
}

void RokaeCR7Driver::disconnect() {
    if (!is_connected_) {
        return;
    }

    try {
        // 如果还在激活状态，先停用
        if (is_activated_) {
            deactivate();
        }

        // 断开连接
        robot_handle_->disconnectFromRobot();
        std::cout << "[RokaeCR7Driver] Disconnected from robot" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "[RokaeCR7Driver] Error during disconnect: " << e.what() << std::endl;
    }

    is_connected_ = false;
}

// ==================== Activation ====================

bool RokaeCR7Driver::activate() {
    if (is_activated_) {
        return true;
    }

    if (!is_connected_) {
        if (!connect()) {
            return false;
        }
    }

    try {
        ec_.clear();

        // 设置运动控制模式为非实时模式
        robot_handle_->setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec_);
        if (ec_) {
            std::cerr << "[RokaeCR7Driver] Failed to set motion control mode" << std::endl;
            return false;
        }

        // 设置操作模式为自动模式
        robot_handle_->setOperateMode(rokae::OperateMode::automatic, ec_);
        if (ec_) {
            std::cerr << "[RokaeCR7Driver] Failed to set operate mode" << std::endl;
            return false;
        }

        // 使能电机（上电）
        robot_handle_->setPowerState(true, ec_);
        if (ec_) {
            std::cerr << "[RokaeCR7Driver] Failed to power on" << std::endl;
            return false;
        }

        // 设置默认速度
        robot_handle_->setDefaultSpeed(300, ec_);
        robot_handle_->adjustSpeedOnline(0.2, ec_);

        std::cout << "[RokaeCR7Driver] Robot activated" << std::endl;
        is_activated_ = true;
        return true;

    } catch (const std::exception& e) {
        std::cerr << "[RokaeCR7Driver] Failed to activate: " << e.what() << std::endl;
        return false;
    }
}

void RokaeCR7Driver::deactivate() {
    if (!is_activated_) {
        return;
    }

    try {
        ec_.clear();

        // 停止运动
        robot_handle_->stop(ec_);

        // 下电
        robot_handle_->setPowerState(false, ec_);

        std::cout << "[RokaeCR7Driver] Robot deactivated" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "[RokaeCR7Driver] Error during deactivate: " << e.what() << std::endl;
    }

    is_activated_ = false;
}

// ==================== Read State ====================

std::array<double, 6> RokaeCR7Driver::getJointPositions() {
    std::array<double, 6> positions{};

    if (!is_connected_) {
        std::cerr << "[RokaeCR7Driver] Not connected" << std::endl;
        return positions;
    }

    try {
        ec_.clear();

        // 获取关节状态
        auto joint_state = robot_handle_->jointStates(ec_);
        if (ec_) {
            std::cerr << "[RokaeCR7Driver] Failed to get joint states" << std::endl;
            return positions;
        }

        // 转换为弧度（SDK返回度）
        for (size_t i = 0; i < 6 && i < joint_state.jointPositions.size(); ++i) {
            positions[i] = degToRad(joint_state.jointPositions[i]);
        }

    } catch (const std::exception& e) {
        std::cerr << "[RokaeCR7Driver] Error getting joint positions: " << e.what() << std::endl;
    }

    return positions;
}

std::array<double, 6> RokaeCR7Driver::getPosturePositions() {
    std::array<double, 6> posture{};

    if (!is_connected_) {
        std::cerr << "[RokaeCR7Driver] Not connected" << std::endl;
        return posture;
    }

    try {
        ec_.clear();

        // 获取末端位姿
        auto pose = robot_handle_->endEffectorPose(ec_);
        if (ec_) {
            std::cerr << "[RokaeCR7Driver] Failed to get end effector pose" << std::endl;
            return posture;
        }

        posture = sdkPoseToArray(pose);

    } catch (const std::exception& e) {
        std::cerr << "[RokaeCR7Driver] Error getting posture: " << e.what() << std::endl;
    }

    return posture;
}

// ==================== Send Commands ====================

bool RokaeCR7Driver::sendJointCommand(const std::array<double, 6>& joints) {
    if (!is_activated_) {
        std::cerr << "[RokaeCR7Driver] Robot not activated" << std::endl;
        return false;
    }

    try {
        ec_.clear();

        // 生成命令ID
        current_cmd_id_ = generateCommandId();

        // 设置运动标记
        nrt_executing_ = true;

        // 将弧度转换为度
        std::vector<double> joint_positions_deg(6);
        for (size_t i = 0; i < 6; ++i) {
            joint_positions_deg[i] = radToDeg(joints[i]);
        }

        // 发送关节运动命令
        robot_handle_->moveJoint(joint_positions_deg, ec_);

        if (ec_) {
            std::cerr << "[RokaeCR7Driver] Failed to send joint command" << std::endl;
            nrt_executing_ = false;
            return false;
        }

        return true;

    } catch (const std::exception& e) {
        std::cerr << "[RokaeCR7Driver] Error sending joint command: " << e.what() << std::endl;
        nrt_executing_ = false;
        return false;
    }
}

bool RokaeCR7Driver::sendCartesianCommand(const std::vector<std::array<double, 6>>& points,
                                          const std::string& move_type, double speed) {
    if (!is_activated_) {
        std::cerr << "[RokaeCR7Driver] Robot not activated" << std::endl;
        return false;
    }

    if (points.empty()) {
        std::cerr << "[RokaeCR7Driver] Empty points" << std::endl;
        return false;
    }

    try {
        ec_.clear();

        // 生成命令ID
        current_cmd_id_ = generateCommandId();

        // 设置运动标记
        nrt_executing_ = true;

        // 设置速度
        robot_handle_->setDefaultSpeed(static_cast<int>(speed), ec_);

        if (move_type == "linear") {
            // 直线运动
            for (const auto& point : points) {
                auto pose = arrayToSdkPose(point);
                robot_handle_->moveLinear(pose, ec_);
                if (ec_) {
                    std::cerr << "[RokaeCR7Driver] Linear motion failed" << std::endl;
                    nrt_executing_ = false;
                    return false;
                }
            }
        } else {
            // 关节运动（默认）
            for (const auto& point : points) {
                auto pose = arrayToSdkPose(point);
                robot_handle_->moveJoint(pose, ec_);
                if (ec_) {
                    std::cerr << "[RokaeCR7Driver] Joint motion failed" << std::endl;
                    nrt_executing_ = false;
                    return false;
                }
            }
        }

        return true;

    } catch (const std::exception& e) {
        std::cerr << "[RokaeCR7Driver] Error sending cartesian command: " << e.what() << std::endl;
        nrt_executing_ = false;
        return false;
    }
}

// ==================== Wait & Check ====================

bool RokaeCR7Driver::waitForMotionComplete(int timeout_ms) {
    if (!nrt_executing_) {
        return true;
    }

    auto start = std::chrono::steady_clock::now();

    while (isMotionExecuting()) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count();

        if (elapsed >= timeout_ms) {
            std::cerr << "[RokaeCR7Driver] Wait for motion complete timeout" << std::endl;
            nrt_executing_ = false;
            return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return true;
}

// ==================== Helper Methods ====================

double RokaeCR7Driver::radToDeg(double rad) {
    return rad * 180.0 / M_PI;
}

double RokaeCR7Driver::degToRad(double deg) {
    return deg * M_PI / 180.0;
}

std::array<double, 6> RokaeCR7Driver::sdkPoseToArray(const rokae::Pose& pose) {
    std::array<double, 6> arr{};

    // 位置 (m -> mm)
    arr[0] = pose.position.x * 1000.0;
    arr[1] = pose.position.y * 1000.0;
    arr[2] = pose.position.z * 1000.0;

    // 姿态（四元数转欧拉角 rx, ry, rz）
    // 这里使用简化的转换，实际项目中可能需要更精确的转换
    double x = pose.orientation.x;
    double y = pose.orientation.y;
    double z = pose.orientation.z;
    double w = pose.orientation.w;

    // Roll (rx)
    double sinr_cosp = 2.0 * (w * x + y * z);
    double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    arr[3] = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (ry)
    double sinp = 2.0 * (w * y - z * x);
    if (std::abs(sinp) >= 1.0) {
        arr[4] = std::copysign(M_PI / 2.0, sinp);
    } else {
        arr[4] = std::asin(sinp);
    }

    // Yaw (rz)
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    arr[5] = std::atan2(siny_cosp, cosy_cosp);

    return arr;
}

rokae::Pose RokaeCR7Driver::arrayToSdkPose(const std::array<double, 6>& arr) {
    rokae::Pose pose;

    // 位置 (mm -> m)
    pose.position.x = arr[0] / 1000.0;
    pose.position.y = arr[1] / 1000.0;
    pose.position.z = arr[2] / 1000.0;

    // 姿态（欧拉角转四元数）
    double rx = arr[3];
    double ry = arr[4];
    double rz = arr[5];

    double cy = std::cos(rz * 0.5);
    double sy = std::sin(rz * 0.5);
    double cp = std::cos(ry * 0.5);
    double sp = std::sin(ry * 0.5);
    double cr = std::cos(rx * 0.5);
    double sr = std::sin(rx * 0.5);

    pose.orientation.w = cr * cp * cy + sr * sp * sy;
    pose.orientation.x = sr * cp * cy - cr * sp * sy;
    pose.orientation.y = cr * sp * cy + sr * cp * sy;
    pose.orientation.z = cr * cp * sy - sr * sp * cy;

    return pose;
}

std::string RokaeCR7Driver::generateCommandId() {
    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count();

    std::stringstream ss;
    ss << "cmd_" << ms;
    return ss.str();
}

} // namespace drivers
} // namespace agv_robot
