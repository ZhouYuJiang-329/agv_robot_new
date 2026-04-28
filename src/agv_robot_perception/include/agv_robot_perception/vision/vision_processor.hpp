/**
 * @file vision_processor.hpp
 * @brief 视觉处理器
 */

#ifndef AGV_ROBOT_PERCEPTION_VISION_PROCESSOR_HPP_
#define AGV_ROBOT_PERCEPTION_VISION_PROCESSOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <opencv2/core/mat.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include "agv_robot_common/types.hpp"
#include "agv_robot_hal/camera/camera_interface.hpp"

namespace agv_robot {
namespace perception {

/**
 * @brief 视觉处理器
 * 负责图像处理和目标检测
 */
class VisionProcessor : public rclcpp::Node {
public:
    explicit VisionProcessor(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~VisionProcessor() override;

    // ==================== 初始化 ====================
    
    /**
     * @brief 初始化处理器
     * @param camera 相机接口
     * @return 是否成功
     */
    bool initialize(const std::shared_ptr<hal::CameraInterface>& camera);
    
    /**
     * @brief 加载模型
     * @param model_path 模型路径
     * @param config_path 配置文件路径
     * @return 是否成功
     */
    bool loadModel(const std::string& model_path,
                   const std::string& config_path = "");
    
    // ==================== 目标检测 ====================
    
    /**
     * @brief 检测图像中的物体
     * @param image 输入图像
     * @param confidence_threshold 置信度阈值
     * @return 检测结果
     */
    std::vector<Detection> detect(const cv::Mat& image,
                                   float confidence_threshold = 0.5f);
    
    /**
     * @brief 检测特定类别
     * @param image 输入图像
     * @param object_class 物体类别
     * @param confidence_threshold 置信度阈值
     * @return 检测结果
     */
    std::vector<Detection> detectByClass(const cv::Mat& image,
                                          const std::string& object_class,
                                          float confidence_threshold = 0.5f);
    
    /**
     * @brief 从相机检测
     * @param confidence_threshold 置信度阈值
     * @return 检测结果
     */
    std::vector<Detection> detectFromCamera(float confidence_threshold = 0.5f);
    
    // ==================== 位姿估计 ====================
    
    /**
     * @brief 估计物体3D位姿
     * @param detection 检测结果
     * @param depth_image 深度图像
     * @return 3D位姿
     */
    Pose estimatePose(const Detection& detection,
                      const cv::Mat& depth_image);
    
    /**
     * @brief 估计物体3D位姿 (使用点云)
     * @param detection 检测结果
     * @param point_cloud 点云
     * @return 3D位姿
     */
    Pose estimatePoseFromPointCloud(
        const Detection& detection,
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& point_cloud);
    
    /**
     * @brief 获取物体点云
     * @param detection 检测结果
     * @return 物体点云
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getObjectPointCloud(
        const Detection& detection);
    
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
     * @return 场景点云
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getScenePointCloud();
    
    /**
     * @brief 平面分割
     * @param point_cloud 输入点云
     * @param distance_threshold 距离阈值
     * @return 平面系数和点云
     */
    std::pair<pcl::ModelCoefficients::Ptr,
              pcl::PointCloud<pcl::PointXYZRGB>::Ptr>
    segmentPlane(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& point_cloud,
                 double distance_threshold = 0.01);
    
    /**
     * @brief 聚类分割
     * @param point_cloud 输入点云
     * @param cluster_tolerance 聚类容差
     * @param min_cluster_size 最小聚类大小
     * @param max_cluster_size 最大聚类大小
     * @return 聚类结果
     */
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>
    clusterSegmentation(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& point_cloud,
        double cluster_tolerance = 0.02,
        int min_cluster_size = 100,
        int max_cluster_size = 25000);
    
    // ==================== 手眼标定 ====================
    
    /**
     * @brief 收集标定数据
     * @param robot_poses 机器人位姿列表
     * @param image_corners 图像角点列表
     * @return 是否成功
     */
    bool collectCalibrationData(
        const std::vector<Pose>& robot_poses,
        const std::vector<std::vector<cv::Point2f>>& image_corners);
    
    /**
     * @brief 执行手眼标定
     * @param calibration_pattern 标定板类型
     * @param pattern_size 标定板尺寸
     * @param square_size 方格大小
     * @return 标定结果
     */
    HandEyeCalibration calibrateHandEye(
        const std::string& calibration_pattern = "chessboard",
        const cv::Size& pattern_size = cv::Size(9, 6),
        double square_size = 0.025);
    
    /**
     * @brief 保存标定结果
     * @param filename 文件名
     * @return 是否成功
     */
    bool saveCalibration(const std::string& filename);
    
    /**
     * @brief 加载标定结果
     * @param filename 文件名
     * @return 是否成功
     */
    bool loadCalibration(const std::string& filename);

private:
    // 相机接口
    std::shared_ptr<hal::CameraInterface> camera_;
    
    // 神经网络推理 (TensorRT/ONNX)
    class InferenceEngine;
    std::unique_ptr<InferenceEngine> inference_engine_;
    
    // 相机参数
    CameraIntrinsics color_intrinsics_;
    CameraIntrinsics depth_intrinsics_;
    
    // 手眼标定
    HandEyeCalibration hand_eye_calibration_;
    bool calibration_loaded_{false};
    
    // 标定数据
    std::vector<Pose> calibration_robot_poses_;
    std::vector<std::vector<cv::Point2f>> calibration_image_corners_;
    
    // 状态
    bool is_initialized_{false};
    bool model_loaded_{false};
    
    // 参数
    std::string model_path_;
    std::string config_path_;
    float default_confidence_threshold_{0.5f};
    
    // 日志
    rclcpp::Logger logger_{rclcpp::get_logger("VisionProcessor")};
    
    // 发布
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detection_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    
    // 处理循环
    rclcpp::TimerBase::SharedPtr process_timer_;
    void processLoop();
};

} // namespace perception
} // namespace agv_robot

#endif // AGV_ROBOT_PERCEPTION_VISION_PROCESSOR_HPP_
