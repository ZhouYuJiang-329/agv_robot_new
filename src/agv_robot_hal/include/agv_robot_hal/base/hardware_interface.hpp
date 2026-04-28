/**
 * @file hardware_interface.hpp
 * @brief 硬件抽象层基础接口
 */

#ifndef AGV_ROBOT_HAL_HARDWARE_INTERFACE_HPP_
#define AGV_ROBOT_HAL_HARDWARE_INTERFACE_HPP_

#include <string>
#include <memory>
#include "agv_robot_common/types.hpp"
#include "agv_robot_common/exceptions.hpp"

namespace agv_robot {
namespace hal {

/**
 * @brief 硬件接口基类
 * 所有硬件设备的抽象基类
 */
class HardwareInterface {
public:
    virtual ~HardwareInterface() = default;
    
    /**
     * @brief 初始化硬件
     * @return 是否成功
     */
    virtual bool initialize() = 0;
    
    /**
     * @brief 启动硬件
     * @return 是否成功
     */
    virtual bool start() = 0;
    
    /**
     * @brief 停止硬件
     * @return 是否成功
     */
    virtual bool stop() = 0;
    
    /**
     * @brief 紧急停止
     * @return 是否成功
     */
    virtual bool emergencyStop() = 0;
    
    /**
     * @brief 关闭硬件
     * @return 是否成功
     */
    virtual bool shutdown() = 0;
    
    /**
     * @brief 获取硬件状态
     * @return 硬件状态
     */
    virtual HardwareStatus getStatus() const = 0;
    
    /**
     * @brief 检查硬件是否就绪
     * @return 是否就绪
     */
    virtual bool isReady() const {
        return getStatus().state == HardwareState::READY;
    }
    
    /**
     * @brief 获取设备名称
     * @return 设备名称
     */
    virtual std::string getName() const = 0;
};

/**
 * @brief 传感器接口基类
 */
class SensorInterface : public HardwareInterface {
public:
    ~SensorInterface() override = default;
    
    /**
     * @brief 检查是否有新数据
     * @return 是否有新数据
     */
    virtual bool hasNewData() const = 0;
    
    /**
     * @brief 获取数据时间戳
     * @return 时间戳
     */
    virtual std::chrono::steady_clock::time_point getTimestamp() const = 0;
};

/**
 * @brief 执行器接口基类
 */
class ActuatorInterface : public HardwareInterface {
public:
    ~ActuatorInterface() override = default;
    
    /**
     * @brief 检查是否正在执行命令
     * @return 是否忙碌
     */
    virtual bool isBusy() const = 0;
    
    /**
     * @brief 等待执行完成
     * @param timeout 超时时间(秒)
     * @return 是否成功完成
     */
    virtual bool waitForCompletion(double timeout) = 0;
};

} // namespace hal
} // namespace agv_robot

#endif // AGV_ROBOT_HAL_HARDWARE_INTERFACE_HPP_