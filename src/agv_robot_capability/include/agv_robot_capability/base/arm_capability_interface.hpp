/**
 * @file arm_capability_interface.hpp
 * @brief 机械臂能力接口
 */

#ifndef AGV_ROBOT_CAPABILITY_BASE_ARM_CAPABILITY_INTERFACE_HPP_
#define AGV_ROBOT_CAPABILITY_BASE_ARM_CAPABILITY_INTERFACE_HPP_

#include <vector>
#include <optional>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include "agv_robot_capability/base/capability_interface.hpp"

namespace agv_robot {
namespace capability {

// 前向声明
namespace planning {
class IArmPlanner;
}
namespace control {
class IArmController;
}

/**
 * @brief 运动选项
 */
struct ArmMotionOptions {
    double velocity{0.5};           // 速度缩放 (0.0-1.0)
    double acceleration{0.5};       // 加速度缩放 (0.0-1.0)
    bool async{false};              // 是否异步执行
    double timeout{60.0};           // 超时时间（秒）
    bool check_collision{true};     // 是否检查碰撞
    double goal_tolerance{0.001};   // 目标容差 (m)
};

/**
 * @brief 机械臂能力接口
 *
 * 提供高层次的机械臂操作接口，封装规划和控制细节
 * 这是应用程序直接调用的接口
 */
class IArmCapability : public ICapability {
public:
    ~IArmCapability() override = default;

    // ==================== 初始化 ====================

    /**
     * @brief 初始化（带规划器和控制器）
     * @param planner 运动规划器
     * @param controller 控制器
     * @return 是否成功
     */
    virtual bool initialize(
        const std::shared_ptr<planning::IArmPlanner>& planner,
        const std::shared_ptr<control::IArmController>& controller) = 0;

    // ==================== 关节空间运动 ====================

    /**
     * @brief 关节空间运动到目标位置
     * @param joint_positions 目标关节位置 (rad)
     * @param options 运动选项
     * @return 操作结果
     */
    virtual OperationResult moveJ(
        const std::vector<double>& joint_positions,
        const ArmMotionOptions& options = ArmMotionOptions{}) = 0;

    /**
     * @brief 关节空间相对运动
     * @param joint_deltas 关节增量 (rad)
     * @param options 运动选项
     * @return 操作结果
     */
    virtual OperationResult moveJRel(
        const std::vector<double>& joint_deltas,
        const ArmMotionOptions& options = ArmMotionOptions{}) = 0;

    /**
     * @brief 运动到命名状态
     * @param state_name 状态名称 (如 "home", "ready")
     * @param options 运动选项
     * @return 操作结果
     */
    virtual OperationResult moveToNamedState(
        const std::string& state_name,
        const ArmMotionOptions& options = ArmMotionOptions{}) = 0;

    // ==================== 笛卡尔空间运动 ====================

    /**
     * @brief 直线运动到目标位姿
     * @param target_pose 目标位姿
     * @param options 运动选项
     * @return 操作结果
     */
    virtual OperationResult moveL(
        const geometry_msgs::msg::Pose& target_pose,
        const ArmMotionOptions& options = ArmMotionOptions{}) = 0;

    /**
     * @brief 直线相对运动
     * @param delta_pose 位姿增量
     * @param options 运动选项
     * @return 操作结果
     */
    virtual OperationResult moveLRel(
        const geometry_msgs::msg::Pose& delta_pose,
        const ArmMotionOptions& options = ArmMotionOptions{}) = 0;

    /**
     * @brief 圆弧运动
     * @param via_point 中间点
     * @param target_pose 目标点
     * @param options 运动选项
     * @return 操作结果
     */
    virtual OperationResult moveC(
        const geometry_msgs::msg::Pose& via_point,
        const geometry_msgs::msg::Pose& target_pose,
        const ArmMotionOptions& options = ArmMotionOptions{}) = 0;

    /**
     * @brief 通过多个路点运动
     * @param waypoints 路点列表
     * @param options 运动选项
     * @return 操作结果
     */
    virtual OperationResult moveThroughWaypoints(
        const std::vector<geometry_msgs::msg::Pose>& waypoints,
        const ArmMotionOptions& options = ArmMotionOptions{}) = 0;

    // ==================== 抓取操作 ====================

    /**
     * @brief 抓取物体
     * @param object_pose 物体位姿
     * @param approach_direction 接近方向
     * @param approach_distance 接近距离 (m)
     * @param options 运动选项
     * @return 操作结果
     */
    virtual OperationResult pick(
        const geometry_msgs::msg::Pose& object_pose,
        const geometry_msgs::msg::Vector3& approach_direction,
        double approach_distance = 0.1,
        const ArmMotionOptions& options = ArmMotionOptions{}) = 0;

    /**
     * @brief 放置物体
     * @param place_pose 放置位姿
     * @param retreat_direction 撤退方向
     * @param retreat_distance 撤退距离 (m)
     * @param options 运动选项
     * @return 操作结果
     */
    virtual OperationResult place(
        const geometry_msgs::msg::Pose& place_pose,
        const geometry_msgs::msg::Vector3& retreat_direction,
        double retreat_distance = 0.1,
        const ArmMotionOptions& options = ArmMotionOptions{}) = 0;

    // ==================== 夹爪控制 ====================

    /**
     * @brief 打开夹爪
     * @param width 开口宽度 (m, 0表示完全打开)
     * @param force 夹持力 (N, 0表示默认)
     * @return 操作结果
     */
    virtual OperationResult openGripper(
        double width = 0.0,
        double force = 0.0) = 0;

    /**
     * @brief 关闭夹爪
     * @param force 夹持力 (N)
     * @return 操作结果
     */
    virtual OperationResult closeGripper(double force = 0.0) = 0;

    /**
     * @brief 设置夹爪位置
     * @param position 位置 (m)
     * @param force 力 (N)
     * @return 操作结果
     */
    virtual OperationResult setGripper(
        double position,
        double force = 0.0) = 0;

    // ==================== 状态查询 ====================

    /**
     * @brief 获取当前关节状态
     * @return 关节状态
     */
    virtual sensor_msgs::msg::JointState getJointState() = 0;

    /**
     * @brief 获取当前末端位姿
     * @return 末端位姿
     */
    virtual geometry_msgs::msg::Pose getEndEffectorPose() = 0;

    /**
     * @brief 获取命名状态的关节位置
     * @param state_name 状态名称
     * @return 关节位置
     */
    virtual std::optional<std::vector<double>> getNamedState(
        const std::string& state_name) = 0;

    /**
     * @brief 检查位姿是否可达
     * @param pose 目标位姿
     * @return 是否可达
     */
    virtual bool isPoseReachable(const geometry_msgs::msg::Pose& pose) = 0;

    /**
     * @brief 获取可达的最近位姿
     * @param desired_pose 期望位姿
     * @return 可达位姿
     */
    virtual std::optional<geometry_msgs::msg::Pose> getNearestReachablePose(
        const geometry_msgs::msg::Pose& desired_pose) = 0;

    // ==================== 停止与恢复 ====================

    /**
     * @brief 停止运动
     * @param deceleration 减速度 (0表示立即停止)
     * @return 操作结果
     */
    virtual OperationResult stop(double deceleration = 0.0) = 0;

    /**
     * @brief 暂停运动
     * @return 操作结果
     */
    virtual OperationResult pause() = 0;

    /**
     * @brief 恢复运动
     * @return 操作结果
     */
    virtual OperationResult resume() = 0;

    // ==================== 伺服模式 ====================

    /**
     * @brief 进入伺服模式（实时控制）
     * @return 操作结果
     */
    virtual OperationResult enterServoMode() = 0;

    /**
     * @brief 退出伺服模式
     * @return 操作结果
     */
    virtual OperationResult exitServoMode() = 0;

    /**
     * @brief 在伺服模式下发送位姿命令
     * @param target_pose 目标位姿
     * @return 操作结果
     */
    virtual OperationResult servoToPose(
        const geometry_msgs::msg::Pose& target_pose) = 0;

    /**
     * @brief 是否在伺服模式
     * @return 是否伺服模式
     */
    virtual bool isInServoMode() const = 0;
};

/**
 * @brief 机械臂能力接口智能指针
 */
using IArmCapabilityPtr = std::shared_ptr<IArmCapability>;

} // namespace capability
} // namespace agv_robot

#endif // AGV_ROBOT_CAPABILITY_BASE_ARM_CAPABILITY_INTERFACE_HPP_
