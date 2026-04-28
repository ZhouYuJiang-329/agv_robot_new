/**
 * @file agv_control_node.hpp
 * @brief AGV控制ROS2节点
 */

#ifndef AGV_ROBOT_CONTROL_NODES_AGV_CONTROL_NODE_HPP_
#define AGV_ROBOT_CONTROL_NODES_AGV_CONTROL_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include "agv_robot_control/base/agv_controller_interface.hpp"

namespace agv_robot {
namespace control {

/**
 * @brief AGV控制ROS2节点
 */
class AGVControlNode : public rclcpp::Node {
public:
    explicit AGVControlNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~AGVControlNode() override;

    /**
     * @brief 设置控制器实现
     * @param controller 控制器接口
     */
    void setController(const IAGVControllerPtr& controller);

    /**
     * @brief 初始化
     * @return 是否成功
     */
    bool initialize();

private:
    // 控制器接口
    IAGVControllerPtr controller_;
    
    // ROS2通信
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr odom_pub_timer_;
    
    // 参数
    double control_rate_{50.0};  // Hz
    double odom_pub_rate_{50.0}; // Hz
    std::string odom_frame_{"odom"};
    std::string base_frame_{"base_link"};
    
    // 回调函数
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void controlTimerCallback();
    void odomPublishTimerCallback();
};

} // namespace control
} // namespace agv_robot

#endif // AGV_ROBOT_CONTROL_NODES_AGV_CONTROL_NODE_HPP_
