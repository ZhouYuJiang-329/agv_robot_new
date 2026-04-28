/**
 * @file detector_interface.hpp
 * @brief 目标检测器接口
 *
 * 职责：
 * 1. 提供目标检测的抽象接口
 * 2. 支持YOLO、TensorRT、传统CV、仿真数据等多种实现
 * 3. 策略模式，可运行时切换
 */

#ifndef AGV_ROBOT_PERCEPTION_VISION_DETECTOR_INTERFACE_HPP_
#define AGV_ROBOT_PERCEPTION_VISION_DETECTOR_INTERFACE_HPP_

#include <opencv2/core/mat.hpp>
#include <vector>
#include <string>
#include "agv_robot_common/types.hpp"

namespace agv_robot {
namespace perception {

/**
 * @brief 检测选项
 */
struct DetectorOptions {
    float confidence_threshold{0.5f};
    float nms_threshold{0.45f};
    std::vector<std::string> target_classes;
    int input_width{640};
    int input_height{480};
};

/**
 * @brief 检测器接口
 */
class IDetector {
public:
    virtual ~IDetector() = default;

    /**
     * @brief 初始化检测器
     * @param model_path 模型路径
     * @param config_path 配置文件路径
     * @return 是否成功
     */
    virtual bool initialize(
        const std::string& model_path,
        const std::string& config_path = "") = 0;

    /**
     * @brief 反初始化
     */
    virtual void deinitialize() = 0;

    /**
     * @brief 检测图像中的目标
     * @param image 输入图像
     * @param options 检测选项
     * @return 检测结果列表
     */
    virtual std::vector<Detection> detect(
        const cv::Mat& image,
        const DetectorOptions& options = DetectorOptions{}) = 0;

    /**
     * @brief 检测特定类别
     * @param image 输入图像
     * @param target_class 目标类别
     * @param options 检测选项
     * @return 检测结果列表
     */
    virtual std::vector<Detection> detectByClass(
        const cv::Mat& image,
        const std::string& target_class,
        const DetectorOptions& options = DetectorOptions{}) = 0;

    /**
     * @brief 获取支持的类别
     * @return 类别列表
     */
    virtual std::vector<std::string> getSupportedClasses() const = 0;

    /**
     * @brief 是否已加载模型
     * @return 是否加载
     */
    virtual bool isModelLoaded() const = 0;

    /**
     * @brief 获取模型名称
     * @return 模型名称
     */
    virtual std::string getModelName() const = 0;

    /**
     * @brief 获取推理时间（毫秒）
     * @return 推理时间
     */
    virtual double getInferenceTime() const = 0;
};

/**
 * @brief 检测器接口智能指针
 */
using IDetectorPtr = std::shared_ptr<IDetector>;

} // namespace perception
} // namespace agv_robot

#endif // AGV_ROBOT_PERCEPTION_VISION_DETECTOR_INTERFACE_HPP_
