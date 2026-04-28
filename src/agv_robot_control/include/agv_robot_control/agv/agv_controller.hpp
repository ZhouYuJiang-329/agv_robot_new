/**
 * @file agv_controller.hpp
 * @brief AGV控制器
 */

#ifndef AGV_ROBOT_CONTROL_AGV_CONTROLLER_HPP_
#define AGV_ROBOT_CONTROL_AGV_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include "agv_robot_hal/agv/agv_interface.hpp"

namespace agv_robot {
namespace control {

/**
 * @brief 控制模式
 */
enum class AGVControlMode {
    IDLE,           // 空闲
    VELOCITY,       // 速度控制
    PATH_FOLLOWING, // 路径跟踪
    POINT_TRACKING  // 点跟踪
};

/**
 * @brief 控制器状态
 */
struct AGVControllerState {
    AGVControlMode mode{AGVControlMode::IDLE};
    geometry_msgs::msg::Twist current_cmd;
    nav_msgs::msg::Odometry current_odom;
    size_t path_index{0};
    bool is_moving{false};
    bool in_error{false};
    std::string error_message;
};

/**
 * @brief AGV控制器
 * 负责执行速度命令和路径跟踪
 */
class AGVController : public rclcpp::Node {
public:
    explicit AGVController(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~AGVController() override;

    // ==================== 初始化 ====================
    
    /**
     * @brief 初始化控制器
     * @param agv_interface AGV硬件接口
     * @return 是否成功
     */
    bool initialize(const std::shared_ptr<hal::AGVInterface>& agv_interface);
    
    // ==================== 速度控制 ====================
    
    /**
     * @brief 设置速度命令
     * @param linear_x 线速度 (m/s)
     * @param angular_z 角速度 (rad/s)
     * @return 是否成功
     */
    bool setVelocity(double linear_x, double angular_z);
    
    /**
     * @brief 设置速度命令
     * @param twist Twist消息
     * @return 是否成功
     */
    bool setVelocity(const geometry_msgs::msg::Twist& twist);
    
    /**
     * @brief 停止
     * @return 是否成功
     */
    bool stop();
    
    // ==================== 路径跟踪 ====================
    
    /**
     * @brief 跟踪路径
     * @param path 路径
     * @param linear_velocity 线速度
     * @param angular_velocity 角速度
     * @return 是否成功
     */
    bool trackPath(const nav_msgs::msg::Path& path,
                   double linear_velocity = 0.5,
                   double angular_velocity = 1.0);
    
    /**
     * @brief 跟踪点
     * @param target_pose 目标位姿
     * @param tolerance 容差
     * @return 是否成功
     */
    bool trackPoint(const geometry_msgs::msg::PoseStamped& target_pose,
                    double tolerance = 0.1);
    
    /**
     * @brief 取消跟踪
     * @return 是否成功
     */
    bool cancelTracking();
    
    // ==================== 状态获取 ====================
    
    /**
     * @brief 获取当前状态
     * @return 控制器状态
     */
    AGVControllerState getState() const { return state_; }
    
    /**
     * @brief 获取当前里程计
     * @return 里程计
     */
    nav_msgs::msg::Odometry getOdometry() const;
    
    /**
     * @brief 检查是否到达目标
     * @param tolerance 位置容差
     * @param yaw_tolerance 角度容差
     * @return 是否到达
     */
    bool isAtGoal(double tolerance = 0.1, double yaw_tolerance = 0.1) const;
    
    /**
     * @brief 等待到达目标
     * @param timeout 超时时间
     * @return 是否成功到达
     */
    bool waitForGoal(double timeout = 60.0);
    
    // ==================== 安全功能 ====================
    
    /**
     * @brief 紧急停止
     * @return 是否成功
     */
    bool emergencyStop();
    
    /**
     * @brief 设置安全速度限制
     * @param max_linear 最大线速度
     * @param max_angular 最大角速度
     */
    void setSafetyLimits(double max_linear, double max_angular);
    
    /**
     * @brief 启用避障
     * @param enable 是否启用
     */
    void enableObstacleAvoidance(bool enable);
    
    /**
     * @brief 设置障碍物距离
     * @param distance 障碍物距离
     */
    void setObstacleDistance(double distance);

private:
    // 硬件接口
    std::shared_ptr<hal::AGVInterface> agv_interface_;
    
    // 当前路径
    nav_msgs::msg::Path current_path_;
    
    // 状态
    AGVControllerState state_;
    std::atomic<bool> stop_requested_{false};
    
    // 目标
    geometry_msgs::msg::PoseStamped target_pose_;
    
    // 控制循环
    rclcpp::TimerBase::SharedPtr control_timer_;
    void controlLoop();
    
    // 纯追踪控制器
    struct PurePursuitConfig {
        double lookahead_distance{0.5};
        double max_linear_velocity{1.0};
        double max_angular_velocity{1.0};
        double wheel_base{0.5};
    } pure_pursuit_config_;
    
    geometry_msgs::msg::Twist computePurePursuit(
        const nav_msgs::msg::Odometry& odom,
        const nav_msgs::msg::Path& path,
        size_t& path_index);
    
    // PID 控制器
    struct PIDController {
        double kp{1.0};
        double ki{0.0};
        double kd{0.1};
        double integral{0.0};
        double prev_error{0.0};
        
        double compute(double error, double dt);
        void reset();
    };
    PIDController linear_pid_;
    PIDController angular_pid_;
    
    // 参数
    double control_rate_{50.0};  // Hz
    double position_tolerance_{0.1};   // m
    double orientation_tolerance_{0.1}; // rad
    
    // 安全
    double obstacle_distance_{999.0};
    bool obstacle_avoidance_enabled_{true};
    
    // 日志
    rclcpp::Logger logger_{rclcpp::get_logger("AGVController")};
};

} // namespace control
} // namespace agv_robot

#endif // AGV_ROBOT_CONTROL_AGV_CONTROLLER_HPP_