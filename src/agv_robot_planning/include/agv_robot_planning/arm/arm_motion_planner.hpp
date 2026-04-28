/**
 * @file arm_motion_planner.hpp
 * @brief 机械臂运动规划器
 */

#ifndef AGV_ROBOT_PLANNING_ARM_MOTION_PLANNER_HPP_
#define AGV_ROBOT_PLANNING_ARM_MOTION_PLANNER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include "agv_robot_common/types.hpp"

namespace agv_robot {
namespace planning {

/**
 * @brief 规划选项
 */
struct PlanningOptions {
    double planning_time{5.0};           // 规划时间限制 (秒)
    int max_planning_attempts{5};        // 最大尝试次数
    double goal_tolerance{0.001};        // 目标容差 (m)
    double orientation_tolerance{0.01};  // 姿态容差 (rad)
    bool avoid_collisions{true};         // 是否避障
    std::string planner_id{"RRTConnect"}; // 规划器ID
};

/**
 * @brief 机械臂运动规划器
 * 负责生成无碰撞的运动轨迹
 */
class ArmMotionPlanner : public rclcpp::Node {
public:
    explicit ArmMotionPlanner(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~ArmMotionPlanner() override;

    // ==================== 初始化 ====================
    
    /**
     * @brief 初始化规划器
     * @param planning_group 规划组名称
     * @return 是否成功
     */
    bool initialize(const std::string& planning_group = "arm");
    
    // ==================== 关节空间规划 ====================
    
    /**
     * @brief 规划关节空间路径
     * @param target_positions 目标关节位置
     * @param start_state 起始状态 (nullptr表示当前状态)
     * @param options 规划选项
     * @return 轨迹
     */
    std::optional<moveit_msgs::msg::RobotTrajectory> planJointPath(
        const std::vector<double>& target_positions,
        const moveit::core::RobotStatePtr& start_state = nullptr,
        const PlanningOptions& options = PlanningOptions{});
    
    /**
     * @brief 规划到命名状态
     * @param named_state 状态名称 (如 "home", "ready")
     * @param options 规划选项
     * @return 轨迹
     */
    std::optional<moveit_msgs::msg::RobotTrajectory> planToNamedState(
        const std::string& named_state,
        const PlanningOptions& options = PlanningOptions{});
    
    // ==================== 笛卡尔空间规划 ====================
    
    /**
     * @brief 规划直线运动
     * @param target_pose 目标位姿
     * @param start_state 起始状态
     * @param velocity 速度 (m/s)
     * @param acceleration 加速度 (m/s^2)
     * @return 轨迹
     */
    std::optional<moveit_msgs::msg::RobotTrajectory> planCartesianPath(
        const geometry_msgs::msg::Pose& target_pose,
        const moveit::core::RobotStatePtr& start_state = nullptr,
        double velocity = 0.1,
        double acceleration = 0.5);
    
    /**
     * @brief 规划圆弧运动
     * @param waypoints 路径点 (至少3个)
     * @param velocity 速度
     * @param acceleration 加速度
     * @return 轨迹
     */
    std::optional<moveit_msgs::msg::RobotTrajectory> planCircularPath(
        const std::vector<geometry_msgs::msg::Pose>& waypoints,
        double velocity = 0.1,
        double acceleration = 0.5);
    
    /**
     * @brief 规划通过多个路径点
     * @param waypoints 路径点
     * @param velocity 速度
     * @param acceleration 加速度
     * @return 轨迹
     */
    std::optional<moveit_msgs::msg::RobotTrajectory> planThroughWaypoints(
        const std::vector<geometry_msgs::msg::Pose>& waypoints,
        double velocity = 0.1,
        double acceleration = 0.5);
    
    // ==================== 避碰规划 ====================
    
    /**
     * @brief 添加碰撞物体
     * @param object_id 物体ID
     * @param shape 形状
     * @param pose 位姿
     */
    void addCollisionObject(
        const std::string& object_id,
        const shape_msgs::msg::SolidPrimitive& shape,
        const geometry_msgs::msg::Pose& pose);
    
    /**
     * @brief 移除碰撞物体
     * @param object_id 物体ID
     */
    void removeCollisionObject(const std::string& object_id);
    
    /**
     * @brief 清除所有碰撞物体
     */
    void clearCollisionObjects();
    
    /**
     * @brief 更新规划场景
     */
    void updatePlanningScene();
    
    // ==================== 轨迹处理 ====================
    
    /**
     * @brief 时间参数化
     * @param trajectory 轨迹
     * @param velocity_scaling 速度缩放
     * @param acceleration_scaling 加速度缩放
     * @return 参数化后的轨迹
     */
    moveit_msgs::msg::RobotTrajectory timeParameterize(
        const moveit_msgs::msg::RobotTrajectory& trajectory,
        double velocity_scaling = 1.0,
        double acceleration_scaling = 1.0);
    
    /**
     * @brief 轨迹平滑
     * @param trajectory 轨迹
     * @return 平滑后的轨迹
     */
    moveit_msgs::msg::RobotTrajectory smoothTrajectory(
        const moveit_msgs::msg::RobotTrajectory& trajectory);
    
    /**
     * @brief 检查轨迹可行性
     * @param trajectory 轨迹
     * @return 是否可行
     */
    bool checkTrajectoryFeasibility(const moveit_msgs::msg::RobotTrajectory& trajectory);
    
    // ==================== 状态获取 ====================
    
    /**
     * @brief 获取当前机器人状态
     * @return 机器人状态
     */
    moveit::core::RobotStatePtr getCurrentState() const;
    
    /**
     * @brief 获取规划组名称
     * @return 规划组名称
     */
    std::string getPlanningGroup() const { return planning_group_; }
    
    /**
     * @brief 检查目标是否可达
     * @param pose 目标位姿
     * @return 是否可达
     */
    bool isPoseReachable(const geometry_msgs::msg::Pose& pose);
    
    /**
     * @brief 获取可达的逆解
     * @param pose 目标位姿
     * @return 关节位置列表
     */
    std::vector<std::vector<double>> getReachableIkSolutions(
        const geometry_msgs::msg::Pose& pose);

private:
    // MoveIt 接口
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
    
    // 轨迹处理
    trajectory_processing::TimeOptimalTrajectoryGeneration time_parametrization_;
    
    // 参数
    std::string planning_group_;
    std::string robot_description_;
    
    // 状态
    bool is_initialized_{false};
    
    // 日志
    rclcpp::Logger logger_{rclcpp::get_logger("ArmMotionPlanner")};
};

} // namespace planning
} // namespace agv_robot

#endif // AGV_ROBOT_PLANNING_ARM_MOTION_PLANNER_HPP_