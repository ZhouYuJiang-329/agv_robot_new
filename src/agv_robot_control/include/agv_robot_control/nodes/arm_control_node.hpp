/**
 * @file arm_control_node.hpp
 * @brief 机械臂控制ROS2节点
 * 
 * 职责：
 * 1. ROS2通信（订阅/发布/Action）
 * 2. 将ROS消息转换为控制器命令
 * 3. 定时调用控制器update()
 * 4. 不实现控制逻辑，委托给IArmController
 */

#ifndef AGV_ROBOT_CONTROL_NODES_ARM_CONTROL_NODE_HPP_
#define AGV_ROBOT_CONTROL_NODES_ARM_CONTROL_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <agv_robot_msgs/action/move_arm.hpp>

#include "agv_robot_control/base/arm_controller_interface.hpp"

namespace agv_robot {
namespace control {

/**
 * @brief 机械臂控制ROS2节点
 */
class ArmControlNode : public rclcpp::Node {
public:
    explicit ArmControlNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~ArmControlNode() override;

    /**
     * @brief 设置控制器实现
     * @param controller 控制器接口
     * 
     * 使用依赖注入，便于测试和切换实现
     */
    void setController(const IArmControllerPtr& controller);

    /**
     * @brief 初始化
     * @return 是否成功
     */
    bool initialize();

private:
    // 控制器接口
    IArmControllerPtr controller_;
    
    // ROS2通信
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr state_pub_timer_;
    
    // Action服务器
    rclcpp_action::Server<control_msgs::action::FollowJointTrajectory>::SharedPtr 
        follow_traj_action_server_;
    rclcpp_action::Server<agv_robot_msgs::action::MoveArm>::SharedPtr 
        move_arm_action_server_;
    
    // 参数
    double control_rate_{100.0};  // Hz
    double state_pub_rate_{50.0}; // Hz
    
    // 回调函数
    void trajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
    void controlTimerCallback();
    void statePublishTimerCallback();
    
    // Action回调
    rclcpp_action::GoalResponse handleFollowTrajectoryGoal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal);
    rclcpp_action::CancelResponse handleFollowTrajectoryCancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle);
    void executeFollowTrajectory(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle);
};

} // namespace control
} // namespace agv_robot

#endif // AGV_ROBOT_CONTROL_NODES_ARM_CONTROL_NODE_HPP_
