/**
 * @file camera_interface.hpp
 * @brief 相机硬件抽象接口
 */

#ifndef AGV_ROBOT_HAL_CAMERA_INTERFACE_HPP_
#define AGV_ROBOT_HAL_CAMERA_INTERFACE_HPP_

#include <opencv2/core/mat.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "agv_robot_hal/base/hardware_interface.hpp"
#include "agv_robot_common/types.hpp"

namespace agv_robot {
namespace hal {

/**
 * @brief 相机接口
 * 定义相机的通用操作接口
 */
class CameraInterface : public SensorInterface {
public:
    ~CameraInterface() override = default;
    
    // ==================== 图像获取 ====================
    
    /**
     * @brief 获取彩色图像
     * @return OpenCV Mat 格式的彩色图像
     */
    virtual cv::Mat getColorImage() = 0;
    
    /**
     * @brief 获取彩色图像 (ROS消息格式)
     * @return ROS图像消息
     */
    virtual sensor_msgs::msg::Image getColorImageMsg() = 0;
    
    /**
     * @brief 获取深度图像
     * @return OpenCV Mat 格式的深度图像 (CV_16UC1, 单位: mm)
     */
    virtual cv::Mat getDepthImage() = 0;
    
    /**
     * @brief 获取深度图像 (ROS消息格式)
     * @return ROS图像消息
     */
    virtual sensor_msgs::msg::Image getDepthImageMsg() = 0;
    
    /**
     * @brief 获取对齐后的深度图像 (与彩色图像对齐)
     * @return 对齐后的深度图像
     */
    virtual cv::Mat getAlignedDepthImage() = 0;
    
    // ==================== 点云获取 ====================
    
    /**
     * @brief 获取点云
     * @return 点云消息
     */
    virtual sensor_msgs::msg::PointCloud2 getPointCloud() = 0;
    
    /**
     * @brief 获取彩色点云 (带RGB信息)
     * @return 彩色点云消息
     */
    virtual sensor_msgs::msg::PointCloud2 getColoredPointCloud() = 0;
    
    // ==================== 相机参数 ====================
    
    /**
     * @brief 获取相机内参
     * @return 相机内参
     */
    virtual CameraIntrinsics getColorCameraIntrinsics() = 0;
    
    /**
     * @brief 获取深度相机内参
     * @return 深度相机内参
     */
    virtual CameraIntrinsics getDepthCameraIntrinsics() = 0;
    
    /**
     * @brief 获取深度到彩色的外参
     * @return 变换矩阵
     */
    virtual std::array<double, 16> getDepthToColorExtrinsics() = 0;
    
    // ==================== 相机控制 ====================
    
    /**
     * @brief 设置曝光时间
     * @param exposure_ms 曝光时间 (ms)
     * @return 是否成功
     */
    virtual bool setExposure(double exposure_ms) = 0;
    
    /**
     * @brief 设置增益
     * @param gain 增益值
     * @return 是否成功
     */
    virtual bool setGain(double gain) = 0;
    
    /**
     * @brief 设置白平衡
     * @param r 红色增益
     * @param g 绿色增益
     * @param b 蓝色增益
     * @return 是否成功
     */
    virtual bool setWhiteBalance(double r, double g, double b) = 0;
    
    /**
     * @brief 启用/禁用自动曝光
     * @param enable 是否启用
     * @return 是否成功
     */
    virtual bool setAutoExposure(bool enable) = 0;
    
    // ==================== 参数获取 ====================
    
    /**
     * @brief 获取图像宽度
     * @return 宽度 (像素)
     */
    virtual int getWidth() const = 0;
    
    /**
     * @brief 获取图像高度
     * @return 高度 (像素)
     */
    virtual int getHeight() const = 0;
    
    /**
     * @brief 获取帧率
     * @return 帧率 (fps)
     */
    virtual double getFrameRate() const = 0;
    
    /**
     * @brief 是否支持深度
     * @return 是否支持
     */
    virtual bool hasDepth() const = 0;
};

} // namespace hal
} // namespace agv_robot

#endif // AGV_ROBOT_HAL_CAMERA_INTERFACE_HPP_