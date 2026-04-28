/**
 * @file agv_planning_node.hpp
 * @brief AGV规划ROS2节点
 */

#ifndef AGV_ROBOT_PLANNING_NODES_AGV_PLANNING_NODE_HPP_
#define AGV_ROBOT_PLANNING_NODES_AGV_PLANNING_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "agv_robot_planning/base/agv_planner_interface.hpp"
#include "agv_robot_msgs/srv/navigate_to_pose.hpp"

namespace agv_robot {
namespace planning {

/**
 * @brief AGV规划ROS2节点
 */
class AGVPlanningNode : public rclcpp::Node {
public:
    explicit AGVPlanningNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~AGVPlanningNode() override;

    /**
     * @brief 设置规划器实现
     * @param planner 规划器接口
     */
    void setPlanner(const IAGVPlannerPtr& planner);

    /**
     * @brief 初始化
     * @return 是否成功
     */
    bool initialize();

private:
    // 规划器接口
    IAGVPlannerPtr planner_;

    // ROS2通信
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global_map_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_map_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;

    // Service服务器
    rclcpp::Service<agv_robot_msgs::srv::NavigateToPose>::SharedPtr navigate_service_;

    // 参数
    std::string global_frame_{"map"};
    std::string robot_base_frame_{"base_link"};
    double planning_rate_{10.0}; // Hz

    // 回调函数
    void globalMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void localMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void navigateServiceCallback(
        const std::shared_ptr<agv_robot_msgs::srv::NavigateToPose::Request> request,
        std::shared_ptr<agv_robot_msgs::srv::NavigateToPose::Response> response);
};

} // namespace planning
} // namespace agv_robot

#endif // AGV_ROBOT_PLANNING_NODES_AGV_PLANNING_NODE_HPP_
