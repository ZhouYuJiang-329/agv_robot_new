/**
 * @file agv_capability_interface.hpp
 * @brief AGV能力接口
 */

#ifndef AGV_ROBOT_CAPABILITY_BASE_AGV_CAPABILITY_INTERFACE_HPP_
#define AGV_ROBOT_CAPABILITY_BASE_AGV_CAPABILITY_INTERFACE_HPP_

#include <vector>
#include <optional>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "agv_robot_capability/base/capability_interface.hpp"

namespace agv_robot {
namespace capability {

// 前向声明
namespace planning {
class IAGVPlanner;
}
namespace control {
class IAGVController;
}

/**
 * @brief 导航选项
 */
struct NavigationOptions {
    double timeout{60.0};               // 超时时间（秒）
    double goal_tolerance{0.1};         // 目标容差 (m)
    double yaw_tolerance{0.1};          // 角度容差 (rad)
    bool avoid_obstacles{true};         // 是否避障
    double max_velocity{1.0};           // 最大速度 (m/s)
    bool use_waypoints{false};          // 是否使用路径点
    int replan_attempts{3};             // 重规划尝试次数
};

/**
 * @brief 导航结果
 */
struct NavigationResult : public OperationResult {
    double distance_traveled{0.0};      // 行驶距离
    double time_taken{0.0};             // 耗时
    int replan_count{0};                // 重规划次数
    nav_msgs::msg::Path executed_path;  // 实际执行路径
};

/**
 * @brief AGV能力接口
 *
 * 提供高层次的AGV导航接口
 * 这是应用程序直接调用的接口
 */
class IAGVCapability : public ICapability {
public:
    ~IAGVCapability() override = default;

    // ==================== 初始化 ====================

    /**
     * @brief 初始化（带规划器和控制器）
     * @param planner 路径规划器
     * @param controller 控制器
     * @return 是否成功
     */
    virtual bool initialize(
        const std::shared_ptr<planning::IAGVPlanner>& planner,
        const std::shared_ptr<control::IAGVController>& controller) = 0;

    // ==================== 导航接口 ====================

    /**
     * @brief 导航到目标位姿
     * @param target_pose 目标位姿
     * @param options 导航选项
     * @return 导航结果
     */
    virtual NavigationResult navigateTo(
        const geometry_msgs::msg::PoseStamped& target_pose,
        const NavigationOptions& options = NavigationOptions{}) = 0;

    /**
     * @brief 导航到指定位置
     * @param x X坐标
     * @param y Y坐标
     * @param yaw 朝向 (rad)
     * @param frame_id 坐标系
     * @param options 导航选项
     * @return 导航结果
     */
    virtual NavigationResult navigateTo(
        double x, double y, double yaw,
        const std::string& frame_id = "map",
        const NavigationOptions& options = NavigationOptions{}) = 0;

    /**
     * @brief 导航到命名位置
     * @param location_name 位置名称 (如 "table1", "charging_station")
     * @param options 导航选项
     * @return 导航结果
     */
    virtual NavigationResult navigateTo(
        const std::string& location_name,
        const NavigationOptions& options = NavigationOptions{}) = 0;

    /**
     * @brief 通过多个路径点导航
     * @param waypoints 路径点列表
     * @param options 导航选项
     * @return 导航结果
     */
    virtual NavigationResult navigateThroughPoses(
        const std::vector<geometry_msgs::msg::PoseStamped>& waypoints,
        const NavigationOptions& options = NavigationOptions{}) = 0;

    /**
     * @brief 巡逻模式
     * @param waypoints 巡逻路径点
     * @param loop_count 循环次数 (-1表示无限)
     * @param options 导航选项
     * @return 导航结果
     */
    virtual NavigationResult patrol(
        const std::vector<geometry_msgs::msg::PoseStamped>& waypoints,
        int loop_count = -1,
        const NavigationOptions& options = NavigationOptions{}) = 0;

    // ==================== 相对运动 ====================

    /**
     * @brief 相对移动
     * @param delta_x X方向增量 (m)
     * @param delta_y Y方向增量 (m)
     * @param delta_yaw 角度增量 (rad)
     * @param options 导航选项
     * @return 导航结果
     */
    virtual NavigationResult moveRel(
        double delta_x,
        double delta_y,
        double delta_yaw,
        const NavigationOptions& options = NavigationOptions{}) = 0;

    /**
     * @brief 前进
     * @param distance 距离 (m)
     * @param options 导航选项
     * @return 导航结果
     */
    virtual NavigationResult moveForward(
        double distance,
        const NavigationOptions& options = NavigationOptions{}) = 0;

    /**
     * @brief 后退
     * @param distance 距离 (m)
     * @param options 导航选项
     * @return 导航结果
     */
    virtual NavigationResult moveBackward(
        double distance,
        const NavigationOptions& options = NavigationOptions{}) = 0;

    /**
     * @brief 旋转
     * @param angle 角度 (rad, 正值为逆时针)
     * @param options 导航选项
     * @return 导航结果
     */
    virtual NavigationResult rotate(
        double angle,
        const NavigationOptions& options = NavigationOptions{}) = 0;

    // ==================== 速度控制 ====================

    /**
     * @brief 设置速度
     * @param linear_x 线速度 (m/s)
     * @param angular_z 角速度 (rad/s)
     * @return 操作结果
     */
    virtual OperationResult setVelocity(
        double linear_x,
        double angular_z) = 0;

    /**
     * @brief 停止
     * @param deceleration 减速度 (m/s²)
     * @return 操作结果
     */
    virtual OperationResult stop(double deceleration = 0.0) = 0;

    // ==================== 状态查询 ====================

    /**
     * @brief 获取当前里程计
     * @return 里程计信息
     */
    virtual nav_msgs::msg::Odometry getOdometry() = 0;

    /**
     * @brief 获取当前位姿
     * @return 当前位姿
     */
    virtual geometry_msgs::msg::PoseStamped getCurrentPose() = 0;

    /**
     * @brief 获取当前速度
     * @param linear_x 输出线速度
     * @param angular_z 输出角速度
     */
    virtual void getCurrentVelocity(
        double& linear_x,
        double& angular_z) = 0;

    /**
     * @brief 获取到目标的距离
     * @return 距离 (m)
     */
    virtual double getDistanceToGoal() = 0;

    /**
     * @brief 获取到目标的角度
     * @return 角度 (rad)
     */
    virtual double getAngleToGoal() = 0;

    // ==================== 位置管理 ====================

    /**
     * @brief 注册命名位置
     * @param name 位置名称
     * @param pose 位姿
     * @return 是否成功
     */
    virtual bool registerLocation(
        const std::string& name,
        const geometry_msgs::msg::PoseStamped& pose) = 0;

    /**
     * @brief 获取命名位置
     * @param name 位置名称
     * @return 位姿
     */
    virtual std::optional<geometry_msgs::msg::PoseStamped> getLocation(
        const std::string& name) = 0;

    /**
     * @brief 获取所有命名位置
     * @return 位置名称列表
     */
    virtual std::vector<std::string> getLocationNames() = 0;

    /**
     * @brief 保存当前位置
     * @param name 位置名称
     * @return 是否成功
     */
    virtual bool saveCurrentLocation(const std::string& name) = 0;

    // ==================== 避障 ====================

    /**
     * @brief 是否正在避障
     * @return 是否避障中
     */
    virtual bool isAvoidingObstacle() = 0;

    /**
     * @brief 设置避障使能
     * @param enable 是否启用
     * @return 操作结果
     */
    virtual OperationResult setObstacleAvoidance(bool enable) = 0;

    /**
     * @brief 是否启用避障
     * @return 是否启用
     */
    virtual bool isObstacleAvoidanceEnabled() = 0;

    // ==================== 电池与状态 ====================

    /**
     * @brief 获取电池电量
     * @return 电量 (0.0-1.0)
     */
    virtual double getBatteryLevel() = 0;

    /**
     * @brief 是否需要充电
     * @return 是否需要充电
     */
    virtual bool needsCharging() = 0;

    /**
     * @brief 导航到充电站
     * @param options 导航选项
     * @return 导航结果
     */
    virtual NavigationResult goToChargingStation(
        const NavigationOptions& options = NavigationOptions{}) = 0;
};

/**
 * @brief AGV能力接口智能指针
 */
using IAGVCapabilityPtr = std::shared_ptr<IAGVCapability>;

} // namespace capability
} // namespace agv_robot

#endif // AGV_ROBOT_CAPABILITY_BASE_AGV_CAPABILITY_INTERFACE_HPP_
