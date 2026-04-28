/**
 * @file agv_controller_interface.hpp
 * @brief AGV控制器接口
 */

#ifndef AGV_ROBOT_CONTROL_BASE_AGV_CONTROLLER_INTERFACE_HPP_
#define AGV_ROBOT_CONTROL_BASE_AGV_CONTROLLER_INTERFACE_HPP_

#include <vector>
#include <optional>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "agv_robot_control/base/controller_interface.hpp"

namespace agv_robot {
namespace control {

/**
 * @brief 路径跟踪状态
 */
enum class PathTrackingState {
    IDLE,           // 空闲
    TRACKING,       // 跟踪中
    PAUSED,         // 已暂停
    REACHED_GOAL,   // 到达目标
    CANCELLED,      // 已取消
    ERROR           // 错误
};

/**
 * @brief 路径跟踪结果
 */
struct PathTrackingResult {
    bool success{false};
    PathTrackingState state{PathTrackingState::IDLE};
    double completion_percentage{0.0};      // 完成百分比
    double distance_to_goal{0.0};           // 到目标距离
    double cross_track_error{0.0};          // 横向误差
    double heading_error{0.0};              // 航向误差
    std::string error_message;
    size_t current_waypoint{0};             // 当前路点索引
};

/**
 * @brief 速度限制
 */
struct VelocityLimits {
    double max_linear_velocity{1.0};        // 最大线速度 (m/s)
    double max_angular_velocity{1.0};       // 最大角速度 (rad/s)
    double max_linear_acceleration{1.0};    // 最大线加速度 (m/s²)
    double max_angular_acceleration{1.0};   // 最大角加速度 (rad/s²)
    double min_linear_velocity{0.0};        // 最小线速度 (m/s)
};

/**
 * @brief AGV控制器接口
 * 
 * 定义AGV控制的标准接口，包括：
 * - 速度控制
 * - 路径跟踪
 * - 点跟踪
 * - 状态监控
 */
class IAGVController : public IController {
public:
    ~IAGVController() override = default;

    // ==================== 速度控制 ====================
    
    /**
     * @brief 设置速度命令
     * @param linear_x 线速度 (m/s)
     * @param angular_z 角速度 (rad/s)
     * @return 是否成功
     * 
     * 直接速度控制，不经过路径规划
     * 适用于遥控或简单控制场景
     */
    virtual bool setVelocity(double linear_x, double angular_z) = 0;
    
    /**
     * @brief 设置速度命令
     * @param twist Twist消息
     * @return 是否成功
     */
    virtual bool setVelocity(const geometry_msgs::msg::Twist& twist) = 0;
    
    /**
     * @brief 停止运动
     * @param deceleration 减速度 (m/s²)，0表示立即停止
     * @return 是否成功
     */
    virtual bool stop(double deceleration = 0.0) = 0;
    
    // ==================== 路径跟踪 ====================
    
    /**
     * @brief 跟踪路径
     * @param path 路径
     * @param linear_velocity 期望线速度 (m/s)
     * @param goal_tolerance 目标容差 (m)
     * @return 是否成功开始跟踪
     * 
     * 此函数应异步执行，立即返回
     * 使用 getPathTrackingResult() 查询跟踪状态
     */
    virtual bool trackPath(
        const nav_msgs::msg::Path& path,
        double linear_velocity = 0.5,
        double goal_tolerance = 0.1) = 0;
    
    /**
     * @brief 取消路径跟踪
     * @return 是否成功
     */
    virtual bool cancelPathTracking() = 0;
    
    /**
     * @brief 暂停路径跟踪
     * @return 是否成功
     */
    virtual bool pausePathTracking() = 0;
    
    /**
     * @brief 恢复路径跟踪
     * @return 是否成功
     */
    virtual bool resumePathTracking() = 0;
    
    /**
     * @brief 获取路径跟踪状态
     * @return 跟踪结果和状态
     */
    virtual PathTrackingResult getPathTrackingResult() const = 0;
    
    /**
     * @brief 是否正在跟踪路径
     * @return 是否跟踪中
     */
    virtual bool isTrackingPath() const = 0;
    
    // ==================== 点跟踪 ====================
    
    /**
     * @brief 跟踪目标点
     * @param target_pose 目标位姿
     * @param linear_velocity 期望线速度
     * @param goal_tolerance 目标容差
     * @param orientation_tolerance 姿态容差
     * @return 是否成功开始跟踪
     * 
     * 内部会生成路径并调用 trackPath
     */
    virtual bool trackPoint(
        const geometry_msgs::msg::Pose& target_pose,
        double linear_velocity = 0.5,
        double goal_tolerance = 0.1,
        double orientation_tolerance = 0.1) = 0;
    
    /**
     * @brief 跟踪路点序列
     * @param waypoints 路点列表
     * @param linear_velocity 期望线速度
     * @param waypoint_tolerance 路点容差
     * @return 是否成功开始跟踪
     */
    virtual bool trackWaypoints(
        const std::vector<geometry_msgs::msg::Pose>& waypoints,
        double linear_velocity = 0.5,
        double waypoint_tolerance = 0.1) = 0;
    
    // ==================== 状态查询 ====================
    
    /**
     * @brief 获取当前里程计
     * @return 里程计信息
     */
    virtual nav_msgs::msg::Odometry getOdometry() const = 0;
    
    /**
     * @brief 获取当前速度
     * @param linear_x 输出线速度
     * @param angular_z 输出角速度
     */
    virtual void getCurrentVelocity(double& linear_x, double& angular_z) const = 0;
    
    /**
     * @brief 获取当前位姿
     * @return 当前位姿
     */
    virtual geometry_msgs::msg::Pose getCurrentPose() const = 0;
    
    // ==================== 配置 ====================
    
    /**
     * @brief 设置速度限制
     * @param limits 速度限制
     */
    virtual void setVelocityLimits(const VelocityLimits& limits) = 0;
    
    /**
     * @brief 获取速度限制
     * @return 速度限制
     */
    virtual VelocityLimits getVelocityLimits() const = 0;
    
    /**
     * @brief 设置控制参数
     * @param kp 比例增益
     * @param ki 积分增益
     * @param kd 微分增益
     */
    virtual void setPIDGains(double kp, double ki, double kd) = 0;
    
    /**
     * @brief 启用/禁用避障
     * @param enable 是否启用
     */
    virtual void setObstacleAvoidance(bool enable) = 0;
    
    /**
     * @brief 是否启用避障
     * @return 是否启用
     */
    virtual bool isObstacleAvoidanceEnabled() const = 0;
};

/**
 * @brief AGV控制器接口智能指针
 */
using IAGVControllerPtr = std::shared_ptr<IAGVController>;

} // namespace control
} // namespace agv_robot

#endif // AGV_ROBOT_CONTROL_BASE_AGV_CONTROLLER_INTERFACE_HPP_
