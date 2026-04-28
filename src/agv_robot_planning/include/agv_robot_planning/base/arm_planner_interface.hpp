/**
 * @file arm_planner_interface.hpp
 * @brief 机械臂规划器接口
 */

#ifndef AGV_ROBOT_PLANNING_BASE_ARM_PLANNER_INTERFACE_HPP_
#define AGV_ROBOT_PLANNING_BASE_ARM_PLANNER_INTERFACE_HPP_

#include <vector>
#include <memory>
#include <optional>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "agv_robot_planning/base/planner_interface.hpp"

namespace agv_robot {
namespace planning {

// 前向声明
struct ArmPlanningResult;

/**
 * @brief 规划选项
 */
struct ArmPlanningOptions {
    double planning_time{5.0};           // 规划时间限制 (秒)
    int max_planning_attempts{5};        // 最大尝试次数
    double goal_tolerance{0.001};        // 目标位置容差 (m)
    double orientation_tolerance{0.01};  // 目标姿态容差 (rad)
    double velocity_scaling{0.5};        // 速度缩放 (0.0-1.0)
    double acceleration_scaling{0.5};   // 加速度缩放 (0.0-1.0)
    bool avoid_collisions{true};         // 是否避障
    std::string planner_id{"RRTConnect"}; // 规划器ID
    std::string planning_pipeline{"ompl"}; // 规划管道
};

/**
 * @brief 机械臂规划结果
 */
struct ArmPlanningResult : public PlanningResult {
    std::optional<moveit_msgs::msg::RobotTrajectory> trajectory;  // 规划轨迹
    std::vector<std::string> planned_joint_names;                 // 规划的关节名称
    double trajectory_duration{0.0};                              // 轨迹时长
    
    /**
     * @brief 获取关节轨迹
     * @return 关节轨迹
     */
    std::optional<trajectory_msgs::msg::JointTrajectory> getJointTrajectory() const {
        if (trajectory && !trajectory->joint_trajectory.points.empty()) {
            return trajectory->joint_trajectory;
        }
        return std::nullopt;
    }
};

/**
 * @brief 约束定义
 */
struct PlanningConstraints {
    std::optional<geometry_msgs::msg::Pose> position_constraint;      // 位置约束
    std::optional<geometry_msgs::msg::Quaternion> orientation_constraint; // 姿态约束
    std::optional<std::vector<double>> joint_constraints;             // 关节约束
    std::optional<double> max_path_length;                            // 最大路径长度
    std::optional<double> min_clearance;                              // 最小间隙
};

/**
 * @brief 机械臂规划器接口
 * 
 * 定义机械臂运动规划的标准接口，支持：
 * - 关节空间规划
 * - 笛卡尔空间规划
 * - 带约束规划
 * - 多目标规划
 */
class IArmPlanner : public IPlanner {
public:
    ~IArmPlanner() override = default;

    // ==================== 关节空间规划 ====================
    
    /**
     * @brief 规划关节空间路径
     * @param start_state 起始关节状态
     * @param target_positions 目标关节位置
     * @param options 规划选项
     * @return 规划结果
     */
    virtual ArmPlanningResult planJointPath(
        const sensor_msgs::msg::JointState& start_state,
        const std::vector<double>& target_positions,
        const ArmPlanningOptions& options = ArmPlanningOptions{}) = 0;
    
    /**
     * @brief 规划到命名状态
     * @param start_state 起始关节状态
     * @param named_state 状态名称 (如 "home", "ready")
     * @param options 规划选项
     * @return 规划结果
     */
    virtual ArmPlanningResult planToNamedState(
        const sensor_msgs::msg::JointState& start_state,
        const std::string& named_state,
        const ArmPlanningOptions& options = ArmPlanningOptions{}) = 0;
    
    /**
     * @brief 规划多段关节路径
     * @param start_state 起始状态
     * @param waypoints 路点（关节位置列表）
     * @param options 规划选项
     * @return 规划结果
     */
    virtual ArmPlanningResult planMultiWaypoints(
        const sensor_msgs::msg::JointState& start_state,
        const std::vector<std::vector<double>>& waypoints,
        const ArmPlanningOptions& options = ArmPlanningOptions{}) = 0;
    
    // ==================== 笛卡尔空间规划 ====================
    
    /**
     * @brief 规划到目标位姿
     * @param start_state 起始关节状态
     * @param target_pose 目标位姿
     * @param options 规划选项
     * @return 规划结果
     */
    virtual ArmPlanningResult planToPose(
        const sensor_msgs::msg::JointState& start_state,
        const geometry_msgs::msg::Pose& target_pose,
        const ArmPlanningOptions& options = ArmPlanningOptions{}) = 0;
    
    /**
     * @brief 规划直线运动
     * @param start_state 起始关节状态
     * @param target_pose 目标位姿
     * @param options 规划选项
     * @return 规划结果
     * 
     * 使用笛卡尔路径规划，确保末端沿直线运动
     */
    virtual ArmPlanningResult planCartesianPath(
        const sensor_msgs::msg::JointState& start_state,
        const geometry_msgs::msg::Pose& target_pose,
        const ArmPlanningOptions& options = ArmPlanningOptions{}) = 0;
    
    /**
     * @brief 规划通过多个笛卡尔路点
     * @param start_state 起始关节状态
     * @param waypoints 笛卡尔路点
     * @param options 规划选项
     * @return 规划结果
     */
    virtual ArmPlanningResult planCartesianWaypoints(
        const sensor_msgs::msg::JointState& start_state,
        const std::vector<geometry_msgs::msg::Pose>& waypoints,
        const ArmPlanningOptions& options = ArmPlanningOptions{}) = 0;
    
    // ==================== 约束规划 ====================
    
    /**
     * @brief 带约束的规划
     * @param start_state 起始状态
     * @param target_pose 目标位姿
     * @param constraints 约束条件
     * @param options 规划选项
     * @return 规划结果
     */
    virtual ArmPlanningResult planWithConstraints(
        const sensor_msgs::msg::JointState& start_state,
        const geometry_msgs::msg::Pose& target_pose,
        const PlanningConstraints& constraints,
        const ArmPlanningOptions& options = ArmPlanningOptions{}) = 0;
    
    // ==================== 验证与优化 ====================
    
    /**
     * @brief 验证轨迹
     * @param trajectory 轨迹
     * @return 是否有效
     */
    virtual bool validateTrajectory(
        const moveit_msgs::msg::RobotTrajectory& trajectory) = 0;
    
    /**
     * @brief 优化轨迹（时间参数化）
     * @param trajectory 输入轨迹
     * @param velocity_scaling 速度缩放
     * @param acceleration_scaling 加速度缩放
     * @return 优化后的轨迹
     */
    virtual std::optional<moveit_msgs::msg::RobotTrajectory> optimizeTrajectory(
        const moveit_msgs::msg::RobotTrajectory& trajectory,
        double velocity_scaling = 0.5,
        double acceleration_scaling = 0.5) = 0;
    
    // ==================== 场景管理 ====================
    
    /**
     * @brief 更新碰撞场景
     * @param world_objects 世界物体
     * @param attached_objects 附着物体
     * @return 是否成功
     */
    virtual bool updatePlanningScene(
        const std::vector<moveit_msgs::msg::CollisionObject>& world_objects,
        const std::vector<moveit_msgs::msg::AttachedCollisionObject>& attached_objects = {}) = 0;
    
    /**
     * @brief 清除碰撞场景
     * @return 是否成功
     */
    virtual bool clearPlanningScene() = 0;
    
    // ==================== 查询 ====================
    
    /**
     * @brief 检查状态有效性
     * @param joint_state 关节状态
     * @return 是否有效
     */
    virtual bool isStateValid(const sensor_msgs::msg::JointState& joint_state) = 0;
    
    /**
     * @brief 获取可达位姿
     * @param joint_state 关节状态
     * @return 末端位姿
     */
    virtual std::optional<geometry_msgs::msg::Pose> getForwardKinematics(
        const sensor_msgs::msg::JointState& joint_state) = 0;
    
    /**
     * @brief 获取逆运动学解
     * @param pose 目标位姿
     * @param seed_state 种子状态
     * @return 关节解列表
     */
    virtual std::vector<std::vector<double>> getInverseKinematics(
        const geometry_msgs::msg::Pose& pose,
        const sensor_msgs::msg::JointState& seed_state) = 0;
    
    /**
     * @brief 获取关节名称
     * @return 关节名称列表
     */
    virtual std::vector<std::string> getJointNames() const = 0;
    
    /**
     * @brief 获取规划组名称
     * @return 规划组名称
     */
    virtual std::string getPlanningGroup() const = 0;
};

/**
 * @brief 机械臂规划器接口智能指针
 */
using IArmPlannerPtr = std::shared_ptr<IArmPlanner>;

} // namespace planning
} // namespace agv_robot

#endif // AGV_ROBOT_PLANNING_BASE_ARM_PLANNER_INTERFACE_HPP_
