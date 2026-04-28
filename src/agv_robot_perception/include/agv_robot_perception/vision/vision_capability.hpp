/**
 * @file vision_capability.hpp
 * @brief 视觉能力实现
 *
 * 职责：
 * 1. 组合 Camera + Detector + PoseEstimator
 * 2. 对外提供统一的视觉感知接口
 * 3. 管理坐标变换（相机→基座→机械臂）
 */

#ifndef AGV_ROBOT_PERCEPTION_VISION_VISION_CAPABILITY_HPP_
#define AGV_ROBOT_PERCEPTION_VISION_VISION_CAPABILITY_HPP_

#include "agv_robot_perception/base/perception_capability_interface.hpp"
#include "agv_robot_perception/vision/camera_interface.hpp"
#include "agv_robot_perception/vision/detector_interface.hpp"
#include "agv_robot_perception/vision/pose_estimator_interface.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace agv_robot {
namespace perception {

/**
 * @brief 视觉能力实现
 *
 * 实现 IPerceptionCapability 接口
 * 内部组合 Camera、Detector、PoseEstimator
 */
class VisionCapability : public IPerceptionCapability {
public:
    VisionCapability();
    ~VisionCapability() override;

    // ==================== 初始化 ====================

    bool initialize(const std::string& config = "") override;

    void deinitialize() override;

    /**
     * @brief 设置相机
     * @param camera 相机接口
     */
    void setCamera(const ICameraPtr& camera);

    /**
     * @brief 设置检测器
     * @param detector 检测器接口
     */
    void setDetector(const IDetectorPtr& detector);

    /**
     * @brief 设置位姿估计器
     * @param estimator 位姿估计器接口
     */
    void setPoseEstimator(const IPoseEstimatorPtr& estimator);

    /**
     * @brief 设置TF监听器
     * @param buffer TF缓冲区
     */
    void setTFBuffer(const std::shared_ptr<tf2_ros::Buffer>& buffer);

    // ==================== 目标检测 ====================

    std::vector<Detection> detectObjects(
        const DetectionOptions& options = DetectionOptions{}) override;

    std::vector<Detection> detectObjects(
        const std::string& object_class,
        const DetectionOptions& options = DetectionOptions{}) override;

    std::optional<Detection> detectObjectById(
        const std::string& object_id) override;

    std::optional<Detection> detectNearestObject(
        const std::string& object_class = "") override;

    // ==================== 位姿估计 ====================

    std::optional<geometry_msgs::msg::Pose> estimatePose(
        const Detection& detection) override;

    std::vector<geometry_msgs::msg::Pose> detectAndEstimatePoses(
        const std::string& object_class = "") override;

    /**
     * @brief 获取物体在指定坐标系下的位姿
     * @param detection 检测结果
     * @param target_frame 目标坐标系
     * @return 位姿
     */
    std::optional<geometry_msgs::msg::Pose> getObjectPoseInFrame(
        const Detection& detection,
        const std::string& target_frame);

    // ==================== 障碍物检测（视觉部分）====================

    std::vector<Obstacle> getObstacles(
        const ObstacleQueryOptions& options = ObstacleQueryOptions{}) override;

    std::optional<Obstacle> getNearestObstacle() override;

    bool hasObstacleInDirection(
        double direction,
        double distance_threshold = 0.5,
        double angle_tolerance = 0.3) override;

    double getClearanceInDirection(
        double direction,
        double max_check_distance = 5.0) override;

    // ==================== 场景理解 ====================

    std::string getSceneDescription() override;

    bool isSceneSafe(
        double direction = 0.0,
        double required_clearance = 0.5) override;

    // ==================== 传感器状态 ====================

    bool isReady() const override;

    std::string getLastError() const override;

    // ==================== 相机控制 ====================

    /**
     * @brief 开始图像采集
     * @return 是否成功
     */
    bool startCapture();

    /**
     * @brief 停止图像采集
     * @return 是否成功
     */
    bool stopCapture();

    /**
     * @brief 保存当前图像
     * @param file_path 文件路径
     * @return 是否成功
     */
    bool saveImage(const std::string& file_path);

private:
    // 组件
    ICameraPtr camera_;
    IDetectorPtr detector_;
    IPoseEstimatorPtr pose_estimator_;

    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // 状态
    bool initialized_{false};
    std::string last_error_;

    // 缓存
    cv::Mat last_color_image_;
    cv::Mat last_depth_image_;
    std::vector<Detection> last_detections_;

    // 辅助函数
    bool updateImages();
    geometry_msgs::msg::Pose transformPose(
        const geometry_msgs::msg::Pose& pose,
        const std::string& source_frame,
        const std::string& target_frame);
};

} // namespace perception
} // namespace agv_robot

#endif // AGV_ROBOT_PERCEPTION_VISION_VISION_CAPABILITY_HPP_
