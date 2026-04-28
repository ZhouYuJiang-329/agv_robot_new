/**
 * @file pose_estimator_interface.hpp
 * @brief 位姿估计器接口
 *
 * 职责：
 * 1. 提供位姿估计的抽象接口
 * 2. 支持PnP、深度学习、点云匹配等多种实现
 */

#ifndef AGV_ROBOT_PERCEPTION_VISION_POSE_ESTIMATOR_INTERFACE_HPP_
#define AGV_ROBOT_PERCEPTION_VISION_POSE_ESTIMATOR_INTERFACE_HPP_

#include <opencv2/core/mat.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <optional>
#include "agv_robot_common/types.hpp"

namespace agv_robot {
namespace perception {

/**
 * @brief 位姿估计选项
 */
struct PoseEstimationOptions {
    bool use_depth{true};               // 是否使用深度
    int refinement_iterations{10};      // 优化迭代次数
    double confidence_threshold{0.8};   // 置信度阈值
};

/**
 * @brief 位姿估计器接口
 */
class IPoseEstimator {
public:
    virtual ~IPoseEstimator() = default;

    /**
     * @brief 初始化估计器
     * @param camera_info 相机参数
     * @param config 配置参数
     * @return 是否成功
     */
    virtual bool initialize(
        const sensor_msgs::msg::CameraInfo& camera_info,
        const std::string& config = "") = 0;

    /**
     * @brief 反初始化
     */
    virtual void deinitialize() = 0;

    /**
     * @brief 估计检测到的物体的位姿
     * @param detection 检测结果
     * @param color_image 彩色图像
     * @param depth_image 深度图像（可选）
     * @param options 估计选项
     * @return 位姿
     */
    virtual std::optional<geometry_msgs::msg::Pose> estimate(
        const Detection& detection,
        const cv::Mat& color_image,
        const cv::Mat& depth_image = cv::Mat(),
        const PoseEstimationOptions& options = PoseEstimationOptions{}) = 0;

    /**
     * @brief 估计位姿（从2D框和深度）
     * @param bbox 2D边界框
     * @param depth_image 深度图像
     * @param camera_info 相机参数
     * @return 位姿
     */
    virtual std::optional<geometry_msgs::msg::Pose> estimateFromDepth(
        const BoundingBox& bbox,
        const cv::Mat& depth_image,
        const sensor_msgs::msg::CameraInfo& camera_info) = 0;

    /**
     * @brief 估计位姿（从点云）
     * @param detection 检测结果
     * @param point_cloud 点云
     * @return 位姿
     */
    virtual std::optional<geometry_msgs::msg::Pose> estimateFromPointCloud(
        const Detection& detection,
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& point_cloud) = 0;

    /**
     * @brief 设置物体模型
     * @param object_class 物体类别
     * @param model_path 模型路径
     * @return 是否成功
     */
    virtual bool setObjectModel(
        const std::string& object_class,
        const std::string& model_path) = 0;

    /**
     * @brief 获取估计置信度
     * @return 置信度 (0.0-1.0)
     */
    virtual double getConfidence() const = 0;

    /**
     * @brief 获取估计误差（米）
     * @return 误差
     */
    virtual double getEstimationError() const = 0;
};

/**
 * @brief 位姿估计器接口智能指针
 */
using IPoseEstimatorPtr = std::shared_ptr<IPoseEstimator>;

} // namespace perception
} // namespace agv_robot

#endif // AGV_ROBOT_PERCEPTION_VISION_POSE_ESTIMATOR_INTERFACE_HPP_
