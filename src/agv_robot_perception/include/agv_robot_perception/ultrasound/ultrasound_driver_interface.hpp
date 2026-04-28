/**
 * @file ultrasound_driver_interface.hpp
 * @brief 超声驱动接口
 *
 * 职责：
 * 1. 提供超声传感器数据的抽象接口
 * 2. 支持真实硬件、仿真、日志回放等多种实现
 */

#ifndef AGV_ROBOT_PERCEPTION_ULTRASOUND_ULTRASOUND_DRIVER_INTERFACE_HPP_
#define AGV_ROBOT_PERCEPTION_ULTRASOUND_ULTRASOUND_DRIVER_INTERFACE_HPP_

#include <vector>
#include <cstddef>
#include <cstdint>
#include <string>
#include <memory>

namespace agv_robot {
namespace perception {

/**
 * @brief 超声传感器配置
 */
struct UltrasoundSensorConfig {
    size_t id{0};                       // 传感器ID
    std::string name;                   // 传感器名称
    double mounting_angle{0.0};         // 安装角度 (rad)
    double mounting_height{0.0};        // 安装高度 (m)
    double min_range{0.05};             // 最小测量范围 (m)
    double max_range{4.0};              // 最大测量范围 (m)
    double fov{0.26};                   // 视场角 (rad)
    bool enabled{true};                 // 是否启用
};

/**
 * @brief 超声读数
 */
struct UltrasoundReading {
    size_t sensor_id{0};                // 传感器ID
    double distance{0.0};               // 测量距离 (m)
    double timestamp{0.0};              // 时间戳
    bool valid{false};                  // 是否有效
    uint8_t quality{0};                 // 数据质量 (0-255)
};

/**
 * @brief 超声驱动接口
 */
class IUltrasoundDriver {
public:
    virtual ~IUltrasoundDriver() = default;

    /**
     * @brief 初始化驱动
     * @param configs 传感器配置列表
     * @return 是否成功
     */
    virtual bool initialize(
        const std::vector<UltrasoundSensorConfig>& configs) = 0;

    /**
     * @brief 反初始化
     */
    virtual void deinitialize() = 0;

    /**
     * @brief 获取所有传感器的读数
     * @return 读数列表
     */
    virtual std::vector<UltrasoundReading> getAllReadings() = 0;

    /**
     * @brief 获取指定传感器的读数
     * @param sensor_id 传感器ID
     * @return 读数
     */
    virtual UltrasoundReading getReading(size_t sensor_id) = 0;

    /**
     * @brief 获取传感器数量
     * @return 传感器数量
     */
    virtual size_t getSensorCount() const = 0;

    /**
     * @brief 获取传感器配置
     * @param sensor_id 传感器ID
     * @return 配置
     */
    virtual UltrasoundSensorConfig getSensorConfig(size_t sensor_id) const = 0;

    /**
     * @brief 启用/禁用传感器
     * @param sensor_id 传感器ID
     * @param enable 是否启用
     * @return 是否成功
     */
    virtual bool setSensorEnabled(size_t sensor_id, bool enable) = 0;

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
 * @brief 超声驱动接口智能指针
 */
using IUltrasoundDriverPtr = std::shared_ptr<IUltrasoundDriver>;

} // namespace perception
} // namespace agv_robot

#endif // AGV_ROBOT_PERCEPTION_ULTRASOUND_ULTRASOUND_DRIVER_INTERFACE_HPP_
