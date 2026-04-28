/**
 * @file vision_capability_interface.hpp
 * @brief 视觉能力接口
 */

#ifndef AGV_ROBOT_CAPABILITY_BASE_VISION_CAPABILITY_INTERFACE_HPP_
#define AGV_ROBOT_CAPABILITY_BASE_VISION_CAPABILITY_INTERFACE_HPP_

#include <vector>
#include <optional>
#include <string>
#include <geometry_msgs/msg/pose.hpp>
#include "agv_robot_common/types.hpp"
#include "agv_robot_capability/base/capability_interface.hpp"

namespace agv_robot {
namespace capability {

// 前向声明
namespace perception {
class VisionProcessor;
}

/**
 * @brief 检测选项
 */
struct DetectionOptions {
    float confidence_threshold{0.5f};       // 置信度阈值
    std::vector<std::string> target_classes; // 目标类别（空表示所有类别）
    double max_distance{2.0};               // 最大检测距离 (m)
    bool require_3d_pose{true};             // 是否需要3D位姿
};

/**
 * @brief 视觉能力接口
 *
 * 提供高层次的视觉感知接口
 * 这是应用程序直接调用的接口
 */
class IVisionCapability : public ICapability {
public:
    ~IVisionCapability() override = default;

    // ==================== 初始化 ====================

    /**
     * @brief 初始化（带视觉处理器）
     * @param processor 视觉处理器
     * @return 是否成功
     */
    virtual bool initialize(
        const std::shared_ptr<perception::VisionProcessor>& processor) = 0;

    // ==================== 目标检测 ====================

    /**
     * @brief 检测特定类别的物体
     * @param object_class 物体类别 (如 "cup", "bottle")
     * @param options 检测选项
     * @return 检测结果列表
     */
    virtual std::vector<Detection> detectObjects(
        const std::string& object_class = "",
        const DetectionOptions& options = DetectionOptions{}) = 0;

    /**
     * @brief 检测多个类别的物体
     * @param object_classes 物体类别列表
     * @param options 检测选项
     * @return 检测结果列表
     */
    virtual std::vector<Detection> detectObjects(
        const std::vector<std::string>& object_classes,
        const DetectionOptions& options = DetectionOptions{}) = 0;

    /**
     * @brief 检测特定ID的物体
     * @param object_id 物体ID
     * @return 检测结果
     */
    virtual std::optional<Detection> detectObjectById(
        const std::string& object_id) = 0;

    /**
     * @brief 检测最近的物体
     * @param object_class 物体类别
     * @param options 检测选项
     * @return 检测结果
     */
    virtual std::optional<Detection> detectNearestObject(
        const std::string& object_class = "",
        const DetectionOptions& options = DetectionOptions{}) = 0;

    /**
     * @brief 检测最大的物体
     * @param object_class 物体类别
     * @param options 检测选项
     * @return 检测结果
     */
    virtual std::optional<Detection> detectLargestObject(
        const std::string& object_class = "",
        const DetectionOptions& options = DetectionOptions{}) = 0;

    // ==================== 位姿估计 ====================

    /**
     * @brief 估计物体位姿
     * @param detection 检测结果
     * @return 3D位姿
     */
    virtual geometry_msgs::msg::Pose estimateObjectPose(
        const Detection& detection) = 0;

    /**
     * @brief 获取物体在机械臂坐标系下的位姿
     * @param object_id 物体ID
     * @return 位姿
     */
    virtual std::optional<geometry_msgs::msg::Pose> getObjectPoseInArmFrame(
        const std::string& object_id) = 0;

    /**
     * @brief 获取物体在基座坐标系下的位姿
     * @param object_id 物体ID
     * @return 位姿
     */
    virtual std::optional<geometry_msgs::msg::Pose> getObjectPoseInBaseFrame(
        const std::string& object_id) = 0;

    /**
     * @brief 获取物体在指定坐标系下的位姿
     * @param object_id 物体ID
     * @param target_frame 目标坐标系
     * @return 位姿
     */
    virtual std::optional<geometry_msgs::msg::Pose> getObjectPoseInFrame(
        const std::string& object_id,
        const std::string& target_frame) = 0;

    // ==================== 手眼标定 ====================

    /**
     * @brief 执行手眼标定
     * @param calibration_pattern 标定板类型 ("chessboard", "circles")
     * @param num_samples 采样数量
     * @param output_path 输出文件路径
     * @return 操作结果
     */
    virtual OperationResult performHandEyeCalibration(
        const std::string& calibration_pattern = "chessboard",
        int num_samples = 10,
        const std::string& output_path = "") = 0;

    /**
     * @brief 加载手眼标定结果
     * @param calibration_file 标定文件路径
     * @return 是否成功
     */
    virtual bool loadHandEyeCalibration(
        const std::string& calibration_file) = 0;

    /**
     * @brief 获取相机到机械臂末端的变换
     * @return 变换矩阵
     */
    virtual std::optional<geometry_msgs::msg::Pose> getCameraToEndEffectorTransform() = 0;

    // ==================== 相机控制 ====================

    /**
     * @brief 开始图像采集
     * @return 操作结果
     */
    virtual OperationResult startCapture() = 0;

    /**
     * @brief 停止图像采集
     * @return 操作结果
     */
    virtual OperationResult stopCapture() = 0;

    /**
     * @brief 是否正在采集图像
     * @return 是否采集中
     */
    virtual bool isCapturing() const = 0;

    /**
     * @brief 保存当前图像
     * @param file_path 文件路径
     * @return 操作结果
     */
    virtual OperationResult saveImage(const std::string& file_path) = 0;

    // ==================== 模型管理 ====================

    /**
     * @brief 加载检测模型
     * @param model_path 模型路径
     * @param config_path 配置文件路径
     * @return 操作结果
     */
    virtual OperationResult loadModel(
        const std::string& model_path,
        const std::string& config_path = "") = 0;

    /**
     * @brief 获取支持的物体类别
     * @return 类别列表
     */
    virtual std::vector<std::string> getSupportedClasses() = 0;

    // ==================== 跟踪 ====================

    /**
     * @brief 开始跟踪物体
     * @param object_id 物体ID
     * @return 操作结果
     */
    virtual OperationResult startTracking(const std::string& object_id) = 0;

    /**
     * @brief 停止跟踪
     * @return 操作结果
     */
    virtual OperationResult stopTracking() = 0;

    /**
     * @brief 获取被跟踪物体的位姿
     * @return 位姿
     */
    virtual std::optional<geometry_msgs::msg::Pose> getTrackedObjectPose() = 0;

    /**
     * @brief 是否正在跟踪
     * @return 是否跟踪中
     */
    virtual bool isTracking() const = 0;
};

/**
 * @brief 视觉能力接口智能指针
 */
using IVisionCapabilityPtr = std::shared_ptr<IVisionCapability>;

} // namespace capability
} // namespace agv_robot

#endif // AGV_ROBOT_CAPABILITY_BASE_VISION_CAPABILITY_INTERFACE_HPP_
