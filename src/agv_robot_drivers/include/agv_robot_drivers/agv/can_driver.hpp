/**
 * @file can_driver.hpp
 * @brief CAN总线AGV驱动
 */

#ifndef AGV_ROBOT_DRIVERS_AGV_CAN_DRIVER_HPP_
#define AGV_ROBOT_DRIVERS_AGV_CAN_DRIVER_HPP_

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/socket.h>
#include <thread>
#include <atomic>
#include "agv_robot_hal/agv/agv_interface.hpp"

namespace agv_robot {
namespace drivers {

/**
 * @brief CAN帧ID定义
 */
namespace CANIds {
constexpr uint32_t CMD_VELOCITY = 0x100;      // 速度命令
constexpr uint32_t CMD_ENABLE = 0x101;        // 使能
constexpr uint32_t CMD_DISABLE = 0x102;       // 失能
constexpr uint32_t CMD_RESET = 0x103;         // 复位
constexpr uint32_t CMD_ESTOP = 0x1FF;         // 急停

constexpr uint32_t FB_ODOMETRY = 0x200;       // 里程计反馈
constexpr uint32_t FB_BATTERY = 0x201;        // 电池状态
constexpr uint32_t FB_STATUS = 0x202;         // 状态反馈
constexpr uint32_t FB_ERROR = 0x203;          // 错误反馈
}

/**
 * @brief CAN总线AGV驱动
 */
class AGVCANDriver : public hal::AGVInterface {
public:
    AGVCANDriver();
    ~AGVCANDriver() override;

    // HardwareInterface implementation
    bool initialize() override;
    bool start() override;
    bool stop() override;
    bool emergencyStop() override;
    bool shutdown() override;
    HardwareStatus getStatus() const override;
    std::string getName() const override { return "AGVCANDriver"; }
    
    // ActuatorInterface implementation
    bool isBusy() const override { return false; }
    bool waitForCompletion(double timeout) override { return true; }
    
    // AGVInterface implementation
    bool setVelocity(double linear_x, double angular_z) override;
    bool setVelocity(const geometry_msgs::msg::Twist& twist) override;
    bool stop() override;
    nav_msgs::msg::Odometry getOdometry() override;
    BatteryStatus getBatteryStatus() override;
    bool getCurrentVelocity(double& linear_x, double& angular_z) override;
    double getMaxLinearVelocity() const override { return max_linear_velocity_; }
    double getMaxAngularVelocity() const override { return max_angular_velocity_; }
    double getWheelBase() const override { return wheel_base_; }
    double getWheelRadius() const override { return wheel_radius_; }

private:
    // CAN socket
    int can_socket_{-1};
    std::string can_interface_{"can0"};
    
    // 配置
    double max_linear_velocity_{1.0};   // m/s
    double max_angular_velocity_{1.0};  // rad/s
    double wheel_base_{0.5};            // m
    double wheel_radius_{0.1};          // m
    double encoder_resolution_{4096.0}; // counts/rev
    
    // 状态
    mutable std::mutex state_mutex_;
    nav_msgs::msg::Odometry odometry_;
    BatteryStatus battery_status_;
    double current_linear_velocity_{0.0};
    double current_angular_velocity_{0.0};
    HardwareStatus status_;
    
    // 接收线程
    std::thread receive_thread_;
    std::atomic<bool> receive_running_{false};
    
    // 辅助函数
    bool openSocket();
    void closeSocket();
    bool sendFrame(const struct can_frame& frame);
    void receiveLoop();
    void processFrame(const struct can_frame& frame);
    
    // 编码解码
    void encodeVelocityCommand(double linear, double angular, struct can_frame& frame);
    void decodeOdometry(const struct can_frame& frame);
    void decodeBatteryStatus(const struct can_frame& frame);
};

} // namespace drivers
} // namespace agv_robot

#endif // AGV_ROBOT_DRIVERS_AGV_CAN_DRIVER_HPP_
