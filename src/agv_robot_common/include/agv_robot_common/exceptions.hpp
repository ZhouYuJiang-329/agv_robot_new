/**
 * @file exceptions.hpp
 * @brief 异常类定义
 */

#ifndef AGV_ROBOT_COMMON_EXCEPTIONS_HPP_
#define AGV_ROBOT_COMMON_EXCEPTIONS_HPP_

#include <stdexcept>
#include <string>
#include "types.hpp"

namespace agv_robot {
namespace exceptions {

// 基础异常类
class RobotException : public std::runtime_error {
public:
    explicit RobotException(const std::string& message)
        : std::runtime_error(message) {}
    
    RobotException(ErrorCode code, const std::string& message)
        : std::runtime_error(message)
        , error_code_(code) {}
    
    ErrorCode getErrorCode() const { return error_code_; }
    
private:
    ErrorCode error_code_{ErrorCode::SUCCESS};
};

// 硬件异常
class HardwareException : public RobotException {
public:
    explicit HardwareException(const std::string& message)
        : RobotException(ErrorCode::HARDWARE_NOT_READY, message) {}
};

// 规划异常
class PlanningException : public RobotException {
public:
    explicit PlanningException(const std::string& message)
        : RobotException(ErrorCode::PLANNING_FAILED, message) {}
};

// 执行异常
class ExecutionException : public RobotException {
public:
    explicit ExecutionException(const std::string& message)
        : RobotException(ErrorCode::EXECUTION_FAILED, message) {}
};

// 通信异常
class CommunicationException : public RobotException {
public:
    explicit CommunicationException(const std::string& message)
        : RobotException(ErrorCode::COMMUNICATION_ERROR, message) {}
};

// 超时异常
class TimeoutException : public RobotException {
public:
    explicit TimeoutException(const std::string& message)
        : RobotException(ErrorCode::TIMEOUT, message) {}
};

// 碰撞异常
class CollisionException : public RobotException {
public:
    explicit CollisionException(const std::string& message)
        : RobotException(ErrorCode::COLLISION_DETECTED, message) {}
};

// 急停异常
class EmergencyStopException : public RobotException {
public:
    explicit EmergencyStopException(const std::string& message)
        : RobotException(ErrorCode::EMERGENCY_STOP, message) {}
};

} // namespace exceptions
} // namespace agv_robot

#endif // AGV_ROBOT_COMMON_EXCEPTIONS_HPP_