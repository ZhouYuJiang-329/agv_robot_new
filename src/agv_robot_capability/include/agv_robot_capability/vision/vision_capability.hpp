/**
 * @file vision_capability.hpp
 * @brief 视觉能力抽象层
 */

#ifndef AGV_ROBOT_CAPABILITY_VISION_CAPABILITY_HPP_
#define AGV_ROBOT_CAPABILITY_VISION_CAPABILITY_HPP_

#include <rclcpp/rclcpp.hpp>
#include <opencv2/core/mat.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "agv_robot_common/types.hpp"

namespace agv_robot {
namespace capability {

// 前向声明
namespace perception {
class VisionProcessor;
}

/**
 * @brief 视觉能力类
 * 提供高层次的视觉感知接口
 */
class VisionCapability : public rclcpp::Node {
public:
    explicit VisionCapability(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~VisionCapability() override;

    // ==================== 初始化 ====================
    
    /**
     * @brief 初始化能力层
     * @param processor 视觉处理器
     * @return 是否成功
     */
    bool initialize(const std::shared_ptr<perception::VisionProcessor>& processor);
    
    // ==================== 目标检测 ====================
    
    /**
     * @brief 检测特定类别的物体
     * @param object_class 物体类别 (如 "cup", "bottle")
     * @param confidence_threshold 置信度阈值
     * @return 检测结果列表
     */
    std::vector<Detection> detectObjects(
        const std::string& object_class = "",
        float confidence_threshold = 0.5f);
    
    /**
     * @brief 检测特定ID的物体
     * @param object_id 物体ID
     * @return 检测结果
     */
    std::optional<Detection> detectObjectById(const std::string& object_id);
    
    /**
     * @brief 检测最近的物体
     * @param object_class 物体类别
     * @return 检测结果
     */
    std::optional<Detection> detectNearestObject(
        const std::string& object_class = "");
    
    // ==================== 位姿估计 ====================
    
    /**
     * @brief 估计物体位姿
     * @param detection 检测结果
     * @return 3D位姿
     */
    geometry_msgs::msg::Pose estimateObjectPose(const Detection& detection);
    
    /**
     * @brief 获取物体在机械臂坐标系下的位姿
     * @param object_id 物体ID
     * @return 位姿
     */
    std::optional<geometry_msgs::msg::Pose> getObjectPoseInArmFrame(
        const std::string& object_id);
    
    /**
     * @brief 获取物体在基座坐标系下的位姿
     * @param object_id 物体ID
     * @return 位姿
     */
    std::optional<geometry_msgs::msg::Pose> getObjectPoseInBaseFrame(
        const std::string& object_id);
    
    // ==================== 手眼标定 ====================
    
    /**
     * @brief 执行手眼标定
     * @param calibration_pattern 标定板类型 ("chessboard", "circles")
     * @param num_samples 采样数量
     * @param output_path 输出文件路径
     * @return 是否成功
     */
    bool calibrateHandEye(const std::string& calibration_pattern = "chessboard",
                          int num_samples = 10,
                          const std::string& output_path = "");
    
    /**
     * @brief 加载手眼标定结果
     * @param calibration_file 标定文件路径
     * @return 是否成功
     */
    bool loadHandEyeCalibration(const std::string& calibration_file);
    
    // ==================== 图像获取 ====================
    
    /**
     * @brief 获取彩色图像
     * @return 彩色图像
     */
    cv::Mat getColorImage();
    
    /**
     * @brief 获取深度图像
     * @return 深度图像
     */
    cv::Mat getDepthImage();
    
    /**
     * @brief 获取对齐后的深度图像
     * @return 对齐后的深度图像
     */
    cv::Mat getAlignedDepthImage();
    
    // ==================== 点云处理 ====================
    
    /**
     * @brief 获取场景点云
     * @return 点云
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getScenePointCloud();
    
    /**
     * @brief 获取物体点云
     * @param detection 检测结果
     * @return 物体点云
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getObjectPointCloud(
        const Detection& detection);
    
    // ==================== 跟踪 ====================
    
    /**
     * @brief 开始跟踪物体
     * @param object_id 物体ID
     * @return 是否成功
     */
    bool startTracking(const std::string& object_id);
    
    /**
     * @brief 停止跟踪
     * @param object_id 物体ID
     * @return 是否成功
     */
    bool stopTracking(const std::string& object_id);
    
    /**
     * @brief 获取跟踪物体的当前位姿
     * @param object_id 物体ID
     * @return 位姿
     */
    std::optional<geometry_msgs::msg::Pose> getTrackedObjectPose(
        const std::string& object_id);
    
    // ==================== 高级功能 ====================
    
    /**
     * @brief 视觉伺服
     * @param target_feature 目标特征
     * @param control_gain 控制增益
     * @return 速度命令
     */
    geometry_msgs::msg::Twist visualServo(
        const geometry_msgs::msg::Pose& target_feature,
        double control_gain = 0.5);
    
    /**
     * @brief 检测平面
     * @param distance_threshold 距离阈值
     * @return 平面参数
     */
    std::optional<pcl::ModelCoefficients::Ptr> detectPlane(
        double distance_threshold = 0.01);
    
    /**
     * @brief 检测边缘
     * @return 边缘点云
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr detectEdges();

private:
    // 视觉处理器
    std::shared_ptr<perception::VisionProcessor> processor_;
    
    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // 手眼标定
    HandEyeCalibration hand_eye_calibration_;
    
    // 状态
    bool is_initialized_{false};
    
    // 日志
    rclcpp::Logger logger_{rclcpp::get_logger("VisionCapability")};
};

} // namespace capability
} // namespace agv_robot

#endif // AGV_ROBOT_CAPABILITY_VISION_CAPABILITY_HPP_