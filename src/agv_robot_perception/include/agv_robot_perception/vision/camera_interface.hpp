/**
 * @file camera_interface.hpp
 * @brief 相机接口
 *
 * 职责：
 * 1. 提供图像采集的抽象接口
 * 2. 支持真实相机、仿真相机、文件相机等多种实现
 */

#ifndef AGV_ROBOT_PERCEPTION_VISION_CAMERA_INTERFACE_HPP_
#define AGV_ROBOT_PERCEPTION_VISION_CAMERA_INTERFACE_HPP_

#include <opencv2/core/mat.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <string>
#include <optional>

namespace agv_robot {
namespace perception {

/**
 * @brief 相机接口
 */
class ICamera {
public:
    virtual ~ICamera() = default;

    /**
     * @brief 初始化相机
     * @param config 配置参数
     * @return 是否成功
     */
    virtual bool initialize(const std::string& config = "") = 0;

    /**
     * @brief 反初始化
     */
    virtual void deinitialize() = 0;

    /**
     * @brief 获取彩色图像
     * @return 图像
     */
    virtual cv::Mat getColorImage() = 0;

    /**
     * @brief 获取深度图像
     * @return 深度图像
     */
    virtual cv::Mat getDepthImage() = 0;

    /**
     * @brief 获取对齐后的深度图像
     * @return 对齐深度图像
     */
    virtual cv::Mat getAlignedDepthImage() = 0;

    /**
     * @brief 获取彩色和深度图像
     * @param color 输出彩色图像
     * @param depth 输出深度图像
     * @return 是否成功
     */
    virtual bool getImages(cv::Mat& color, cv::Mat& depth) = 0;

    /**
     * @brief 获取相机参数
     * @return 相机参数
     */
    virtual sensor_msgs::msg::CameraInfo getCameraInfo() = 0;

    /**
     * @brief 是否已连接
     * @return 是否连接
     */
    virtual bool isConnected() const = 0;

    /**
     * @brief 开始采集
     * @return 是否成功
     */
    virtual bool startCapture() = 0;

    /**
     * @brief 停止采集
     * @return 是否成功
     */
    virtual bool stopCapture() = 0;

    /**
     * @brief 是否正在采集
     * @return 是否采集中
     */
    virtual bool isCapturing() const = 0;
};

/**
 * @brief 相机接口智能指针
 */
using ICameraPtr = std::shared_ptr<ICamera>;

} // namespace perception
} // namespace agv_robot

#endif // AGV_ROBOT_PERCEPTION_VISION_CAMERA_INTERFACE_HPP_
