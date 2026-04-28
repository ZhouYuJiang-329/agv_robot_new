/**
 * @file ultrasound_interface.hpp
 * @brief 超声传感器硬件抽象接口
 */

#ifndef AGV_ROBOT_HAL_ULTRASOUND_INTERFACE_HPP_
#define AGV_ROBOT_HAL_ULTRASOUND_INTERFACE_HPP_

#include <vector>
#include "agv_robot_hal/base/hardware_interface.hpp"
#include "agv_robot_common/types.hpp"

namespace agv_robot {
namespace hal {

/**
 * @brief 单个超声传感器数据
 */
struct UltrasoundReading {
    uint8_t sensor_id{0};
    double distance{0.0};      // m
    double confidence{0.0};    // 0-1
    double timestamp{0.0};     // seconds
};

/**
 * @brief 超声传感器接口
 * 定义超声传感器的通用操作接口
 */
class UltrasoundInterface : public SensorInterface {
public:
    ~UltrasoundInterface() override = default;
    
    // ==================== 数据获取 ====================
    
    /**
     * @brief 获取单个传感器距离
     * @param sensor_id 传感器ID
     * @return 距离 (m), 无效时返回 -1
     */
    virtual double getDistance(uint8_t sensor_id) = 0;
    
    /**
     * @brief 获取单个传感器完整读数
     * @param sensor_id 传感器ID
     * @return 传感器读数
     */
    virtual UltrasoundReading getReading(uint8_t sensor_id) = 0;
    
    /**
     * @brief 获取所有传感器读数
     * @return 所有传感器读数
     */
    virtual std::vector<UltrasoundReading> getAllReadings() = 0;
    
    /**
     * @brief 获取所有传感器距离
     * @return 距离数组
     */
    virtual std::vector<double> getAllDistances() = 0;
    
    // ==================== 批量操作 ====================
    
    /**
     * @brief 触发一次测量
     * @return 是否成功
     */
    virtual bool triggerMeasurement() = 0;
    
    /**
     * @brief 设置测量模式
     * @param continuous 是否连续测量
     * @return 是否成功
     */
    virtual bool setContinuousMode(bool continuous) = 0;
    
    // ==================== 参数获取 ====================
    
    /**
     * @brief 获取传感器数量
     * @return 传感器数量
     */
    virtual size_t getSensorCount() const = 0;
    
    /**
     * @brief 获取传感器配置
     * @param sensor_id 传感器ID
     * @return 传感器配置
     */
    virtual UltrasoundSensorConfig getSensorConfig(uint8_t sensor_id) const = 0;
    
    /**
     * @brief 获取最大测量范围
     * @return 最大范围 (m)
     */
    virtual double getMaxRange() const = 0;
    
    /**
     * @brief 获取最小测量范围
     * @return 最小范围 (m)
     */
    virtual double getMinRange() const = 0;
    
    /**
     * @brief 获取测量频率
     * @return 频率 (Hz)
     */
    virtual double getMeasurementRate() const = 0;
};

} // namespace hal
} // namespace agv_robot

#endif // AGV_ROBOT_HAL_ULTRASOUND_INTERFACE_HPP_