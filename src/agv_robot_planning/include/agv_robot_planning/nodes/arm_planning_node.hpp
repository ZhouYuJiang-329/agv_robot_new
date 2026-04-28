/**
 * @file arm_planning_node.hpp
 * @brief 机械臂规划ROS2节点
 *
 * 职责：
 * 1. ROS2通信（Action服务）
 * 2. 将ROS消息转换为规划请求
 * 3. 管理规划场景
 * 4. 不实现规划算法，委托给IArmPlanner
 */

#ifndef AGV_ROBOT_PLANNING_NODES_ARM_PLANNING_NODE_HPP_
#define AGV_ROBOT_PLANNING_NODES_ARM_PLANNING_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "agv_robot_planning/base/arm_planner_interface.hpp"
#include "agv_robot_msgs/action/move_arm.hpp"

namespace agv_robot {
namespace planning {

/**
 * @brief 机械臂规划ROS2节点
 */
class ArmPlanningNode : public rclcpp::Node {
public:
    explicit ArmPlanningNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~ArmPlanningNode() override;

    /**
     * @brief 设置规划器实现
     * @param planner 规划器接口
     */
    void setPlanner(const IArmPlannerPtr& planner);

    /**
     * @brief 初始化
     * @return 是否成功
     */
    bool initialize();

private:
    // 规划器接口
    IArmPlannerPtr planner_;

    // ROS2通信
    rclcpp::Subscription<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_sub_;
    rclcpp::Publisher<moveit_msgs::msg::RobotTrajectory>::SharedPtr trajectory_pub_;

    // Action服务器
    rclcpp_action::Server<agv_robot_msgs::action::MoveArm>::SharedPtr move_arm_action_server_;

    // 参数
    std::string planning_group_{"arm"};
    double default_planning_time_{5.0};

    // 回调函数
    void planningSceneCallback(const moveit_msgs::msg::PlanningScene::SharedPtr msg);

    // Action回调
    rclcpp_action::GoalResponse handleMoveArmGoal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const agv_robot_msgs::action::MoveArm::Goal> goal);
    rclcpp_action::CancelResponse handleMoveArmCancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<agv_robot_msgs::action::MoveArm>> goal_handle);
    void executeMoveArm(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<agv_robot_msgs::action::MoveArm>> goal_handle);
};

} // namespace planning
} // namespace agv_robot

#endif // AGV_ROBOT_PLANNING_NODES_ARM_PLANNING_NODE_HPP_
