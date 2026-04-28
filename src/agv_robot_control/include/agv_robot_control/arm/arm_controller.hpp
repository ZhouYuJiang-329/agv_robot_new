/**
 * @file arm_controller.hpp
 * @brief 机械臂控制器
 */

#ifndef AGV_ROBOT_CONTROL_ARM_CONTROLLER_HPP_
#define AGV_ROBOT_CONTROL_ARM_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include "agv_robot_hal/arm/arm_interface.hpp"

namespace agv_robot {
namespace control {

/**
 * @brief 控制模式
 */
enum class ControlMode {
    IDLE,           // 空闲
    POSITION,       // 位置控制
    VELOCITY,       // 速度控制
    TRAJECTORY,     // 轨迹跟踪
    SERVO,          // 伺服模式
    FORCE           // 力控制
};

/**
 * @brief 控制器状态
 */
struct ControllerState {
    ControlMode mode{ControlMode::IDLE};
    sensor_msgs::msg::JointState current_state;
    sensor_msgs::msg::JointState target_state;
    double tracking_error{0.0};
    bool is_moving{false};
    bool in_error{false};
    std::string error_message;
};

/**
 * @brief 机械臂控制器
 * 负责执行轨迹和实时控制
 */
class ArmController : public rclcpp::Node {
public:
    explicit ArmController(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~ArmController() override;

    // ==================== 初始化 ====================
    
    /**
     * @brief 初始化控制器
     * @param arm_interface 机械臂硬件接口
     * @return 是否成功
     */
    bool initialize(const std::shared_ptr<hal::ArmInterface>& arm_interface);
    
    // ==================== 轨迹执行 ====================
    
    /**
     * @brief 执行轨迹
     * @param trajectory 轨迹
     * @param async 是否异步执行
     * @return 是否成功
     */
    bool executeTrajectory(const moveit_msgs::msg::RobotTrajectory& trajectory,
                           bool async = false);
    
    /**
     * @brief 执行关节轨迹
     * @param trajectory 关节轨迹
     * @param async 是否异步
     * @return 是否成功
     */
    bool executeJointTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory,
                                 bool async = false);
    
    /**
     * @brief 取消当前执行
     * @return 是否成功
     */
    bool cancelExecution();
    
    /**
     * @brief 暂停执行
     * @return 是否成功
     */
    bool pause();
    
    /**
     * @brief 恢复执行
     * @return 是否成功
     */
    bool resume();
    
    // ==================== 伺服控制 ====================
    
    /**
     * @brief 启用伺服模式
     * @return 是否成功
     */
    bool enableServoMode();
    
    /**
     * @brief 禁用伺服模式
     * @return 是否成功
     */
    bool disableServoMode();
    
    /**
     * @brief 伺服命令 (笛卡尔速度)
     * @param twist 速度命令
     * @return 是否成功
     */
    bool servoCartesian(const geometry_msgs::msg::TwistStamped& twist);
    
    /**
     * @brief 伺服命令 (关节速度)
     * @param joint_velocities 关节速度
     * @return 是否成功
     */
    bool servoJoint(const std::vector<double>& joint_velocities);
    
    // ==================== 力控制 ====================
    
    /**
     * @brief 启用力控制模式
     * @return 是否成功
     */
    bool enableForceMode();
    
    /**
     * @brief 设置目标力/力矩
     * @param wrench 力/力矩
     * @return 是否成功
     */
    bool setTargetWrench(const geometry_msgs::msg::Wrench& wrench);
    
    // ==================== 状态获取 ====================
    
    /**
     * @brief 获取当前状态
     * @return 控制器状态
     */
    ControllerState getState() const { return state_; }
    
    /**
     * @brief 获取当前关节状态
     * @return 关节状态
     */
    sensor_msgs::msg::JointState getCurrentState() const;
    
    /**
     * @brief 检查是否正在执行
     * @return 是否执行中
     */
    bool isExecuting() const { return state_.is_moving; }
    
    /**
     * @brief 等待执行完成
     * @param timeout 超时时间 (秒)
     * @return 是否成功完成
     */
    bool waitForExecution(double timeout = 30.0);
    
    /**
     * @brief 获取轨迹进度
     * @return 进度 (0.0-1.0)
     */
    double getProgress() const;
    
    // ==================== 安全功能 ====================
    
    /**
     * @brief 紧急停止
     * @return 是否成功
     */
    bool emergencyStop();
    
    /**
     * @brief 清除错误
     * @return 是否成功
     */
    bool clearError();
    
    /**
     * @brief 检查碰撞
     * @return 是否碰撞
     */
    bool checkCollision();

private:
    // 硬件接口
    std::shared_ptr<hal::ArmInterface> arm_interface_;
    
    // Action client
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr trajectory_client_;
    
    // 当前轨迹
    trajectory_msgs::msg::JointTrajectory current_trajectory_;
    size_t current_trajectory_index_{0};
    
    // 状态
    ControllerState state_;
    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> pause_requested_{false};
    
    // 控制循环
    rclcpp::TimerBase::SharedPtr control_timer_;
    void controlLoop();
    
    // PID 控制器
    struct PIDController {
        double kp{0.0};
        double ki{0.0};
        double kd{0.0};
        double integral{0.0};
        double prev_error{0.0};
        
        double compute(double error, double dt);
        void reset();
    };
    std::vector<PIDController> pid_controllers_;
    
    // 参数
    double control_rate_{100.0};  // Hz
    double position_tolerance_{0.01};  // rad
    double velocity_tolerance_{0.1};   // rad/s
    
    // 日志
    rclcpp::Logger logger_{rclcpp::get_logger("ArmController")};
    
    // 执行线程
    std::future<bool> execution_future_;
    bool executeTrajectoryAsync();
};

} // namespace control
} // namespace agv_robot

#endif // AGV_ROBOT_CONTROL_ARM_CONTROLLER_HPP_