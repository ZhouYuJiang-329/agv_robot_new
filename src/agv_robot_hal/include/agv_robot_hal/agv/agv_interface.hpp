/**
 * @file agv_interface.hpp
 * @brief AGV硬件抽象接口
 */

#ifndef AGV_ROBOT_HAL_AGV_INTERFACE_HPP_
#define AGV_ROBOT_HAL_AGV_INTERFACE_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "agv_robot_hal/base/hardware_interface.hpp"
#include "agv_robot_common/types.hpp"

namespace agv_robot {
namespace hal {

/**
 * @brief AGV接口
 * 定义AGV的通用操作接口
 */
class AGVInterface : public ActuatorInterface {
public:
    ~AGVInterface() override = default;
    
    // ==================== 速度控制 ====================
    
    /**
     * @brief 设置速度命令
     * @param linear_x 前进速度 (m/s)
     * @param angular_z 旋转速度 (rad/s)
     * @return 是否成功
     */
    virtual bool setVelocity(double linear_x, double angular_z) = 0;
    
    /**
     * @brief 设置速度命令 (Twist消息)
     * @param twist 速度命令
     * @return 是否成功
     */
    virtual bool setVelocity(const geometry_msgs::msg::Twist& twist) = 0;
    
    /**
     * @brief 停止运动
     * @return 是否成功
     */
    virtual bool stop() override = 0;
    
    // ==================== 状态获取 ====================
    
    /**
     * @brief 获取里程计
     * @return 里程计消息
     */
    virtual nav_msgs::msg::Odometry getOdometry() = 0;
    
    /**
     * @brief 获取电池状态
     * @return 电池状态
     */
    virtual BatteryStatus getBatteryStatus() = 0;
    
    /**
     * @brief 获取当前速度
     * @param linear_x 输出前进速度
     * @param angular_z 输出旋转速度
     * @return 是否成功
     */
    virtual bool getCurrentVelocity(double& linear_x, double& angular_z) = 0;
    
    // ==================== 参数获取 ====================
    
    /**
     * @brief 获取最大线速度
     * @return 最大线速度 (m/s)
     */
    virtual double getMaxLinearVelocity() const = 0;
    
    /**
     * @brief 获取最大角速度
     * @return 最大角速度 (rad/s)
     */
    virtual double getMaxAngularVelocity() const = 0;
    
    /**
     * @brief 获取轮距
     * @return 轮距 (m)
     */
    virtual double getWheelBase() const = 0;
    
    /**
     * @brief 获取轮子半径
     * @return 轮子半径 (m)
     */
    virtual double getWheelRadius() const = 0;
};

} // namespace hal
} // namespace agv_robot

#endif // AGV_ROBOT_HAL_AGV_INTERFACE_HPP_