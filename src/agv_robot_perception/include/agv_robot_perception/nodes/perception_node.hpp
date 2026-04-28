/**
 * @file perception_node.hpp
 * @brief 感知层ROS2节点
 *
 * 职责：
 * 1. ROS2通信（服务/Topic）
 * 2. 将ROS请求转换为感知能力调用
 * 3. 管理视觉和超声能力组件
 * 4. 不实现业务逻辑，委托给IPerceptionCapability接口
 */

#ifndef AGV_ROBOT_PERCEPTION_NODES_PERCEPTION_NODE_HPP_
#define AGV_ROBOT_PERCEPTION_NODES_PERCEPTION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include "agv_robot_perception/base/perception_capability_interface.hpp"
#include "agv_robot_perception/vision/vision_capability.hpp"
#include "agv_robot_perception/ultrasound/ultrasound_capability.hpp"

namespace agv_robot {
namespace perception {

/**
 * @brief 感知层ROS2节点
 *
 * 整合视觉和超声能力，提供统一的ROS2服务入口
 */
class PerceptionNode : public rclcpp::Node {
public:
    explicit PerceptionNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~PerceptionNode() override;

    // ==================== 设置能力实现 ====================

    void setVisionCapability(const std::shared_ptr<VisionCapability>& capability);
    void setUltrasoundCapability(const std::shared_ptr<UltrasoundCapability>& capability);

    /**
     * @brief 初始化
     * @return 是否成功
     */
    bool initialize();

    // ==================== 获取能力接口 ====================

    std::shared_ptr<VisionCapability> getVisionCapability() const { return vision_capability_; }
    std::shared_ptr<UltrasoundCapability> getUltrasoundCapability() const { return ultrasound_capability_; }

private:
    // 能力接口
    std::shared_ptr<VisionCapability> vision_capability_;
    std::shared_ptr<UltrasoundCapability> ultrasound_capability_;

    // ROS2通信 - 检测服务
    rclcpp::Service<agv_robot_msgs::srv::DetectObject>::SharedPtr detect_object_service_;

    // ROS2通信 - 位姿估计服务
    rclcpp::Service<agv_robot_msgs::srv::EstimatePose>::SharedPtr estimate_pose_service_;

    // ROS2通信 - 障碍物查询服务
    rclcpp::Service<agv_robot_msgs::srv::GetObstacles>::SharedPtr get_obstacles_service_;

    // ROS2通信 - 安全查询服务
    rclcpp::Service<agv_robot_msgs::srv::CheckSafety>::SharedPtr check_safety_service_;

    // 发布者
    rclcpp::Publisher<agv_robot_msgs::msg::DetectionArray>::SharedPtr detections_pub_;
    rclcpp::Publisher<agv_robot_msgs::msg::ObstacleArray>::SharedPtr obstacles_pub_;
    rclcpp::Publisher<agv_robot_msgs::msg::PerceptionStatus>::SharedPtr status_pub_;

    // 定时器
    rclcpp::TimerBase::SharedPtr publish_timer_;

    // 回调函数
    void detectObjectServiceCallback(
        const std::shared_ptr<agv_robot_msgs::srv::DetectObject::Request> request,
        std::shared_ptr<agv_robot_msgs::srv::DetectObject::Response> response);

    void estimatePoseServiceCallback(
        const std::shared_ptr<agv_robot_msgs::srv::EstimatePose::Request> request,
        std::shared_ptr<agv_robot_msgs::srv::EstimatePose::Response> response);

    void getObstaclesServiceCallback(
        const std::shared_ptr<agv_robot_msgs::srv::GetObstacles::Request> request,
        std::shared_ptr<agv_robot_msgs::srv::GetObstacles::Response> response);

    void checkSafetyServiceCallback(
        const std::shared_ptr<agv_robot_msgs::srv::CheckSafety::Request> request,
        std::shared_ptr<agv_robot_msgs::srv::CheckSafety::Response> response);

    void publishTimerCallback();
};

} // namespace perception
} // namespace agv_robot

#endif // AGV_ROBOT_PERCEPTION_NODES_PERCEPTION_NODE_HPP_
