/**
 * @file capability_node.hpp
 * @brief 能力层ROS2节点
 *
 * 职责：
 * 1. ROS2通信（服务/Action/Topic）
 * 2. 将ROS请求转换为能力调用
 * 3. 管理多个能力组件
 * 4. 不实现业务逻辑，委托给ICapability接口
 */

#ifndef AGV_ROBOT_CAPABILITY_NODES_CAPABILITY_NODE_HPP_
#define AGV_ROBOT_CAPABILITY_NODES_CAPABILITY_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "agv_robot_capability/base/arm_capability_interface.hpp"
#include "agv_robot_capability/base/agv_capability_interface.hpp"
#include "agv_robot_capability/base/vision_capability_interface.hpp"
#include "agv_robot_capability/base/ultrasound_capability_interface.hpp"

namespace agv_robot {
namespace capability {

/**
 * @brief 能力层ROS2节点
 *
 * 整合所有能力接口，提供统一的ROS2服务入口
 */
class CapabilityNode : public rclcpp::Node {
public:
    explicit CapabilityNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~CapabilityNode() override;

    // ==================== 设置能力实现 ====================

    void setArmCapability(const IArmCapabilityPtr& capability);
    void setAGVCapability(const IAGVCapabilityPtr& capability);
    void setVisionCapability(const IVisionCapabilityPtr& capability);
    void setUltrasoundCapability(const IUltrasoundCapabilityPtr& capability);

    /**
     * @brief 初始化
     * @return 是否成功
     */
    bool initialize();

    // ==================== 获取能力接口 ====================

    IArmCapabilityPtr getArmCapability() const { return arm_capability_; }
    IAGVCapabilityPtr getAGVCapability() const { return agv_capability_; }
    IVisionCapabilityPtr getVisionCapability() const { return vision_capability_; }
    IUltrasoundCapabilityPtr getUltrasoundCapability() const { return ultrasound_capability_; }

private:
    // 能力接口
    IArmCapabilityPtr arm_capability_;
    IAGVCapabilityPtr agv_capability_;
    IVisionCapabilityPtr vision_capability_;
    IUltrasoundCapabilityPtr ultrasound_capability_;

    // ROS2通信 - 机械臂服务
    rclcpp::Service<agv_robot_msgs::srv::MoveArm>::SharedPtr move_arm_service_;

    // ROS2通信 - AGV服务
    rclcpp::Service<agv_robot_msgs::srv::NavigateToPose>::SharedPtr navigate_service_;

    // ROS2通信 - 视觉服务
    rclcpp::Service<agv_robot_msgs::srv::DetectObject>::SharedPtr detect_object_service_;

    // Action服务器
    rclcpp_action::Server<agv_robot_msgs::action::PickPlace>::SharedPtr pick_place_action_server_;
    rclcpp_action::Server<agv_robot_msgs::action::Patrol>::SharedPtr patrol_action_server_;

    // 回调函数
    void moveArmServiceCallback(
        const std::shared_ptr<agv_robot_msgs::srv::MoveArm::Request> request,
        std::shared_ptr<agv_robot_msgs::srv::MoveArm::Response> response);

    void navigateServiceCallback(
        const std::shared_ptr<agv_robot_msgs::srv::NavigateToPose::Request> request,
        std::shared_ptr<agv_robot_msgs::srv::NavigateToPose::Response> response);

    void detectObjectServiceCallback(
        const std::shared_ptr<agv_robot_msgs::srv::DetectObject::Request> request,
        std::shared_ptr<agv_robot_msgs::srv::DetectObject::Response> response);

    // Action回调
    rclcpp_action::GoalResponse handlePickPlaceGoal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const agv_robot_msgs::action::PickPlace::Goal> goal);
    rclcpp_action::CancelResponse handlePickPlaceCancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<agv_robot_msgs::action::PickPlace>> goal_handle);
    void executePickPlace(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<agv_robot_msgs::action::PickPlace>> goal_handle);
};

} // namespace capability
} // namespace agv_robot

#endif // AGV_ROBOT_CAPABILITY_NODES_CAPABILITY_NODE_HPP_
