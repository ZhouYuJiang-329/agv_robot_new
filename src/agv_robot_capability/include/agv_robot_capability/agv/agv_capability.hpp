/**
 * @file agv_capability.hpp
 * @brief AGV能力抽象层
 */

#ifndef AGV_ROBOT_CAPABILITY_AGV_CAPABILITY_HPP_
#define AGV_ROBOT_CAPABILITY_AGV_CAPABILITY_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include "agv_robot_common/types.hpp"

namespace agv_robot {
namespace capability {

// 前向声明
namespace planning {
class AGVPlanner;
}
namespace control {
class AGVController;
}

/**
 * @brief AGV能力类
 * 提供高层次的AGV导航接口
 */
class AGVCapability : public rclcpp::Node {
public:
    explicit AGVCapability(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~AGVCapability() override;

    // ==================== 初始化 ====================
    
    /**
     * @brief 初始化能力层
     * @param planner 路径规划器
     * @param controller 控制器
     * @return 是否成功
     */
    bool initialize(
        const std::shared_ptr<planning::AGVPlanner>& planner,
        const std::shared_ptr<control::AGVController>& controller);
    
    // ==================== 导航接口 ====================
    
    /**
     * @brief 导航到目标位姿
     * @param target_pose 目标位姿
     * @param timeout 超时时间 (秒)
     * @return 是否成功
     */
    bool navigateTo(const geometry_msgs::msg::PoseStamped& target_pose,
                    double timeout = 60.0);
    
    /**
     * @brief 导航到指定位置
     * @param x X坐标
     * @param y Y坐标
     * @param yaw 朝向 (rad)
     * @param frame_id 坐标系
     * @param timeout 超时时间
     * @return 是否成功
     */
    bool navigateTo(double x, double y, double yaw,
                    const std::string& frame_id = "map",
                    double timeout = 60.0);
    
    /**
     * @brief 导航到命名位置
     * @param location_name 位置名称 (如 "table1", "charging_station")
     * @param timeout 超时时间
     * @return 是否成功
     */
    bool navigateTo(const std::string& location_name, double timeout = 60.0);
    
    /**
     * @brief 通过多个路径点导航
     * @param waypoints 路径点列表
     * @param loop_count 循环次数 (-1表示无限循环)
     * @return 是否成功
     */
    bool navigateThroughPoses(
        const std::vector<geometry_msgs::msg::PoseStamped>& waypoints,
        int loop_count = 1);
    
    // ==================== 相对运动 ====================
    
    /**
     * @brief 相对移动
     * @param delta_x X方向增量 (m)
     * @param delta_y Y方向增量 (m)
     * @param delta_yaw 角度增量 (rad)
     * @return 是否成功
     */
    bool moveRel(double delta_x, double delta_y, double delta_yaw);
    
    /**
     * @brief 旋转到指定角度
     * @param target_yaw 目标角度 (rad)
     * @return 是否成功
     */
    bool rotateTo(double target_yaw);
    
    /**
     * @brief 相对旋转
     * @param delta_yaw 角度增量 (rad)
     * @return 是否成功
     */
    bool rotateRel(double delta_yaw);
    
    // ==================== 速度控制 ====================
    
    /**
     * @brief 设置速度
     * @param linear_x 线速度 (m/s)
     * @param angular_z 角速度 (rad/s)
     * @return 是否成功
     */
    bool setVelocity(double linear_x, double angular_z);
    
    /**
     * @brief 停止运动
     * @return 是否成功
     */
    bool stop();
    
    // ==================== 跟随模式 ====================
    
    /**
     * @brief 跟随目标
     * @param target_frame 目标TF帧
     * @param distance 跟随距离 (m)
     * @return 是否成功
     */
    bool follow(const std::string& target_frame, double distance = 1.0);
    
    /**
     * @brief 停止跟随
     * @return 是否成功
     */
    bool stopFollow();
    
    // ==================== 状态获取 ====================
    
    /**
     * @brief 获取当前位姿
     * @return 当前位姿
     */
    geometry_msgs::msg::PoseStamped getCurrentPose() const;
    
    /**
     * @brief 获取里程计
     * @return 里程计
     */
    nav_msgs::msg::Odometry getOdometry() const;
    
    /**
     * @brief 获取电池状态
     * @return 电池状态
     */
    BatteryStatus getBatteryStatus() const;
    
    /**
     * @brief 检查是否正在导航
     * @return 是否导航中
     */
    bool isNavigating() const;
    
    /**
     * @brief 检查是否到达目标
     * @param tolerance 位置容差 (m)
     * @param yaw_tolerance 角度容差 (rad)
     * @return 是否到达
     */
    bool isAtGoal(double tolerance = 0.1, double yaw_tolerance = 0.1) const;
    
    /**
     * @brief 等待导航完成
     * @param timeout 超时时间 (秒)
     * @return 是否成功完成
     */
    bool waitForNavigation(double timeout = 60.0);
    
    // ==================== 路径管理 ====================
    
    /**
     * @brief 添加命名位置
     * @param name 位置名称
     * @param pose 位姿
     */
    void addLocation(const std::string& name,
                     const geometry_msgs::msg::PoseStamped& pose);
    
    /**
     * @brief 获取命名位置
     * @param name 位置名称
     * @return 位姿
     */
    std::optional<geometry_msgs::msg::PoseStamped> getLocation(
        const std::string& name) const;
    
    /**
     * @brief 取消当前导航
     * @return 是否成功
     */
    bool cancelNavigation();

private:
    // 规划器
    std::shared_ptr<planning::AGVPlanner> planner_;
    
    // 控制器
    std::shared_ptr<control::AGVController> controller_;
    
    // Action clients
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_to_pose_client_;
    rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr nav_through_poses_client_;
    
    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // 命名位置地图
    std::unordered_map<std::string, geometry_msgs::msg::PoseStamped> locations_;
    
    // 状态
    bool is_initialized_{false};
    bool is_navigating_{false};
    
    // 日志
    rclcpp::Logger logger_{rclcpp::get_logger("AGVCapability")};
    
    // 回调
    void navigationResultCallback(
        const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result);
};

} // namespace capability
} // namespace agv_robot

#endif // AGV_ROBOT_CAPABILITY_AGV_CAPABILITY_HPP_