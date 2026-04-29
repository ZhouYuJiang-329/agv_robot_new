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
        robot_handle_->disconnectFromRobot(ec_);
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

        // 获取关节位置
        auto joint_positions = robot_handle_->jointPos(ec_);
        if (ec_) {
            std::cerr << "[RokaeCR7Driver] Failed to get joint positions" << std::endl;
            return positions;
        }

        // SDK 返回的是弧度，直接使用
        for (size_t i = 0; i < 6 && i < joint_positions.size(); ++i) {
            positions[i] = joint_positions[i];
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

        // 获取末端位姿 [x, y, z, rx, ry, rz] (单位: m, rad)
        auto pose = robot_handle_->posture(rokae::CoordinateType::endInRef, ec_);
        if (ec_) {
            std::cerr << "[RokaeCR7Driver] Failed to get end effector pose" << std::endl;
            return posture;
        }

        // 转换为 mm
        posture[0] = pose[0] * 1000.0;  // X
        posture[1] = pose[1] * 1000.0;  // Y
        posture[2] = pose[2] * 1000.0;  // Z
        posture[3] = pose[3];           // Rx
        posture[4] = pose[4];           // Ry
        posture[5] = pose[5];           // Rz

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

        // 关节角度已经是弧度，直接使用
        std::vector<double> joint_positions(joints.begin(), joints.end());

        // 创建关节位置目标
        rokae::JointPosition target;
        target.joints = joint_positions;

        // 创建绝对关节运动命令
        rokae::MoveAbsJCommand cmd(target);

        // 发送命令
        std::string cmd_id;
        robot_handle_->moveAppend(cmd, cmd_id, ec_);
        if (ec_) {
            std::cerr << "[RokaeCR7Driver] Failed to append joint command" << std::endl;
            nrt_executing_ = false;
            return false;
        }

        // 开始运动
        robot_handle_->moveStart(ec_);
        if (ec_) {
            std::cerr << "[RokaeCR7Driver] Failed to start motion" << std::endl;
            nrt_executing_ = false;
            return false;
        }

        current_cmd_id_ = cmd_id;
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

        // 添加所有路径点
        for (const auto& point : points) {
            // 创建笛卡尔位置目标 (mm -> m)
            rokae::CartesianPosition target;
            target.trans[0] = point[0] / 1000.0;  // X
            target.trans[1] = point[1] / 1000.0;  // Y
            target.trans[2] = point[2] / 1000.0;  // Z
            target.rpy[0] = point[3];             // Rx
            target.rpy[1] = point[4];             // Ry
            target.rpy[2] = point[5];             // Rz

            if (move_type == "linear") {
                // 直线运动
                rokae::MoveLCommand cmd(target);
                cmd.zone = 0.5;  // mm
                robot_handle_->moveAppend(cmd, current_cmd_id_, ec_);
            } else {
                // 关节运动（默认）
                rokae::MoveJCommand cmd(target);
                cmd.zone = 0.5;  // mm
                robot_handle_->moveAppend(cmd, current_cmd_id_, ec_);
            }

            if (ec_) {
                std::cerr << "[RokaeCR7Driver] Failed to append motion command" << std::endl;
                nrt_executing_ = false;
                return false;
            }
        }

        // 开始运动
        robot_handle_->moveStart(ec_);
        if (ec_) {
            std::cerr << "[RokaeCR7Driver] Failed to start motion" << std::endl;
            nrt_executing_ = false;
            return false;
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
