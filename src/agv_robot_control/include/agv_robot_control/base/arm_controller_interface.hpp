/**
 * @file arm_controller_interface.hpp
 * @brief 机械臂控制器接口
 */

#ifndef AGV_ROBOT_CONTROL_BASE_ARM_CONTROLLER_INTERFACE_HPP_
#define AGV_ROBOT_CONTROL_BASE_ARM_CONTROLLER_INTERFACE_HPP_

#include <vector>
#include <optional>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "agv_robot_control/base/controller_interface.hpp"

namespace agv_robot {
namespace control {

/**
 * @brief 轨迹执行状态
 */
enum class TrajectoryExecutionState {
    IDLE,           // 空闲
    EXECUTING,      // 执行中
    PAUSED,         // 已暂停
    COMPLETED,      // 已完成
    CANCELLED,      // 已取消
    ERROR           // 错误
};

/**
 * @brief 轨迹执行结果
 */
struct TrajectoryExecutionResult {
    bool success{false};
    TrajectoryExecutionState state{TrajectoryExecutionState::IDLE};
    double completion_percentage{0.0};  // 完成百分比
    double position_error{0.0};         // 位置误差
    double velocity_error{0.0};         // 速度误差
    std::string error_message;
    size_t current_waypoint{0};         // 当前路点索引
};

/**
 * @brief 伺服命令（用于实时控制）
 */
struct ServoCommand {
    std::optional<std::vector<double>> joint_positions;     // 关节位置
    std::optional<std::vector<double>> joint_velocities;    // 关节速度
    std::optional<geometry_msgs::msg::Pose> cartesian_pose; // 笛卡尔位姿
    std::optional<geometry_msgs::msg::Twist> cartesian_twist; // 笛卡尔速度
    double timestamp{0.0};  // 命令时间戳
};

/**
 * @brief 机械臂控制器接口
 * 
 * 定义机械臂控制的标准接口，包括：
 * - 轨迹执行
 * - 伺服控制（实时）
 * - 状态监控
 * 
 * 实现类应专注于控制算法，不处理ROS通信
 */
class IArmController : public IController {
public:
    ~IArmController() override = default;

    // ==================== 轨迹执行 ====================
    
    /**
     * @brief 执行关节空间轨迹
     * @param trajectory 关节轨迹
     * @return 是否成功开始执行
     * 
     * 此函数应异步执行，立即返回
     * 使用 getTrajectoryExecutionResult() 查询执行状态
     */
    virtual bool executeJointTrajectory(
        const trajectory_msgs::msg::JointTrajectory& trajectory) = 0;
    
    /**
     * @brief 执行笛卡尔空间轨迹
     * @param trajectory 机器人轨迹（包含关节轨迹）
     * @return 是否成功开始执行
     */
    virtual bool executeCartesianTrajectory(
        const moveit_msgs::msg::RobotTrajectory& trajectory) = 0;
    
    /**
     * @brief 执行点到点运动
     * @param target_joint_positions 目标关节位置
     * @param velocity_scaling 速度缩放因子 (0.0-1.0)
     * @param acceleration_scaling 加速度缩放因子 (0.0-1.0)
     * @return 是否成功开始执行
     */
    virtual bool moveToJointPositions(
        const std::vector<double>& target_joint_positions,
        double velocity_scaling = 0.5,
        double acceleration_scaling = 0.5) = 0;
    
    /**
     * @brief 执行直线运动
     * @param target_pose 目标位姿
     * @param velocity_scaling 速度缩放因子
     * @param acceleration_scaling 加速度缩放因子
     * @return 是否成功开始执行
     */
    virtual bool moveLinear(
        const geometry_msgs::msg::Pose& target_pose,
        double velocity_scaling = 0.5,
        double acceleration_scaling = 0.5) = 0;
    
    // ==================== 轨迹控制 ====================
    
    /**
     * @brief 取消当前轨迹执行
     * @return 是否成功
     */
    virtual bool cancelTrajectory() = 0;
    
    /**
     * @brief 暂停轨迹执行
     * @return 是否成功
     */
    virtual bool pauseTrajectory() = 0;
    
    /**
     * @brief 恢复轨迹执行
     * @return 是否成功
     */
    virtual bool resumeTrajectory() = 0;
    
    /**
     * @brief 获取轨迹执行状态
     * @return 执行结果和状态
     */
    virtual TrajectoryExecutionResult getTrajectoryExecutionResult() const = 0;
    
    /**
     * @brief 是否正在执行轨迹
     * @return 是否执行中
     */
    virtual bool isExecutingTrajectory() const = 0;
    
    // ==================== 伺服控制（实时控制）====================
    
    /**
     * @brief 发送伺服命令
     * @param command 伺服命令
     * @return 是否成功
     * 
     * 用于实时控制，如视觉伺服、力控等
     * 需要以固定频率调用
     */
    virtual bool sendServoCommand(const ServoCommand& command) = 0;
    
    /**
     * @brief 设置伺服模式
     * @param enable 是否启用伺服模式
     * @return 是否成功
     */
    virtual bool setServoMode(bool enable) = 0;
    
    /**
     * @brief 是否在伺服模式
     * @return 是否伺服模式
     */
    virtual bool isInServoMode() const = 0;
    
    // ==================== 状态查询 ====================
    
    /**
     * @brief 获取当前关节状态
     * @return 关节状态
     */
    virtual sensor_msgs::msg::JointState getJointState() const = 0;
    
    /**
     * @brief 获取当前末端位姿
     * @return 末端位姿
     */
    virtual geometry_msgs::msg::Pose getEndEffectorPose() const = 0;
    
    /**
     * @brief 获取关节数量
     * @return 关节数
     */
    virtual size_t getJointCount() const = 0;
    
    /**
     * @brief 获取关节名称
     * @return 关节名称列表
     */
    virtual std::vector<std::string> getJointNames() const = 0;
    
    // ==================== 配置 ====================
    
    /**
     * @brief 设置速度缩放
     * @param scaling 缩放因子 (0.0-1.0)
     */
    virtual void setVelocityScaling(double scaling) = 0;
    
    /**
     * @brief 设置加速度缩放
     * @param scaling 缩放因子 (0.0-1.0)
     */
    virtual void setAccelerationScaling(double scaling) = 0;
    
    /**
     * @brief 设置目标容差
     * @param position_tolerance 位置容差（米）
     * @param orientation_tolerance 姿态容差（弧度）
     */
    virtual void setGoalTolerance(
        double position_tolerance,
        double orientation_tolerance) = 0;
};

/**
 * @brief 机械臂控制器接口智能指针
 */
using IArmControllerPtr = std::shared_ptr<IArmController>;

} // namespace control
} // namespace agv_robot

#endif // AGV_ROBOT_CONTROL_BASE_ARM_CONTROLLER_INTERFACE_HPP_
