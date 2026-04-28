/**
 * @file agv_planner.hpp
 * @brief AGV路径规划器
 */

#ifndef AGV_ROBOT_PLANNING_AGV_PLANNER_HPP_
#define AGV_ROBOT_PLANNING_AGV_PLANNER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "agv_robot_common/types.hpp"

namespace agv_robot {
namespace planning {

/**
 * @brief 规划模式
 */
enum class PlanningMode {
    GLOBAL,     // 全局规划
    LOCAL,      // 局部规划
    HYBRID      // 混合规划
};

/**
 * @brief 规划选项
 */
struct AGVPlanningOptions {
    PlanningMode mode{PlanningMode::HYBRID};
    double goal_tolerance{0.1};          // 目标容差 (m)
    double yaw_tolerance{0.1};           // 角度容差 (rad)
    bool allow_unknown{false};           // 是否允许未知区域
    double inflation_radius{0.3};        // 膨胀半径 (m)
};

/**
 * @brief AGV路径规划器
 * 负责生成AGV的导航路径
 */
class AGVPlanner : public rclcpp::Node {
public:
    explicit AGVPlanner(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~AGVPlanner() override;

    // ==================== 初始化 ====================
    
    /**
     * @brief 初始化规划器
     * @return 是否成功
     */
    bool initialize();
    
    // ==================== 全局规划 ====================
    
    /**
     * @brief 规划到目标点的路径
     * @param start 起始位姿
     * @param goal 目标位姿
     * @param options 规划选项
     * @return 路径
     */
    std::optional<nav_msgs::msg::Path> planPath(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal,
        const AGVPlanningOptions& options = AGVPlanningOptions{});
    
    /**
     * @brief 规划通过多个路径点
     * @param waypoints 路径点
     * @param options 规划选项
     * @return 路径
     */
    std::optional<nav_msgs::msg::Path> planThroughWaypoints(
        const std::vector<geometry_msgs::msg::PoseStamped>& waypoints,
        const AGVPlanningOptions& options = AGVPlanningOptions{});
    
    /**
     * @brief 重新规划路径
     * @param current_pose 当前位姿
     * @param goal 目标位姿
     * @return 新路径
     */
    std::optional<nav_msgs::msg::Path> replanPath(
        const geometry_msgs::msg::PoseStamped& current_pose,
        const geometry_msgs::msg::PoseStamped& goal);
    
    // ==================== 局部规划 ====================
    
    /**
     * @brief 局部路径规划 (避障)
     * @param global_path 全局路径
     * @param current_pose 当前位姿
     * @param obstacles 障碍物列表
     * @return 局部路径
     */
    std::optional<nav_msgs::msg::Path> planLocalPath(
        const nav_msgs::msg::Path& global_path,
        const geometry_msgs::msg::PoseStamped& current_pose,
        const std::vector<Obstacle>& obstacles);
    
    /**
     * @brief 计算速度命令
     * @param current_pose 当前位姿
     * @param target_pose 目标位姿
     * @param current_velocity 当前速度
     * @return 速度命令
     */
    geometry_msgs::msg::Twist computeVelocityCommand(
        const geometry_msgs::msg::PoseStamped& current_pose,
        const geometry_msgs::msg::PoseStamped& target_pose,
        const geometry_msgs::msg::Twist& current_velocity);
    
    // ==================== 地图管理 ====================
    
    /**
     * @brief 更新代价地图
     * @param costmap 代价地图
     */
    void updateCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap);
    
    /**
     * @brief 更新障碍物
     * @param obstacles 障碍物列表
     */
    void updateObstacles(const std::vector<Obstacle>& obstacles);
    
    /**
     * @brief 清除障碍物
     */
    void clearObstacles();
    
    // ==================== 路径处理 ====================
    
    /**
     * @brief 平滑路径
     * @param path 原始路径
     * @return 平滑后的路径
     */
    nav_msgs::msg::Path smoothPath(const nav_msgs::msg::Path& path);
    
    /**
     * @brief 简化路径
     * @param path 原始路径
     * @param tolerance 容差
     * @return 简化后的路径
     */
    nav_msgs::msg::Path simplifyPath(const nav_msgs::msg::Path& path,
                                      double tolerance = 0.1);
    
    /**
     * @brief 插值路径
     * @param path 原始路径
     * @param resolution 分辨率 (m)
     * @return 插值后的路径
     */
    nav_msgs::msg::Path interpolatePath(const nav_msgs::msg::Path& path,
                                         double resolution = 0.05);
    
    /**
     * @brief 检查路径可行性
     * @param path 路径
     * @return 是否可行
     */
    bool checkPathFeasibility(const nav_msgs::msg::Path& path);
    
    // ==================== 查询 ====================
    
    /**
     * @brief 获取最近的路径点索引
     * @param path 路径
     * @param pose 当前位姿
     * @return 索引
     */
    size_t getClosestPointIndex(const nav_msgs::msg::Path& path,
                                 const geometry_msgs::msg::PoseStamped& pose);
    
    /**
     * @brief 计算路径长度
     * @param path 路径
     * @return 长度 (m)
     */
    double computePathLength(const nav_msgs::msg::Path& path);
    
    /**
     * @brief 估计到达时间
     * @param path 路径
     * @param average_velocity 平均速度
     * @return 时间 (秒)
     */
    double estimateArrivalTime(const nav_msgs::msg::Path& path,
                                double average_velocity = 0.5);

private:
    // 代价地图
    nav_msgs::msg::OccupancyGrid::SharedPtr costmap_;
    
    // 障碍物
    std::vector<Obstacle> obstacles_;
    
    // 参数
    double robot_radius_{0.3};
    double safety_margin_{0.1};
    double max_linear_velocity_{1.0};
    double max_angular_velocity_{1.0};
    
    // 状态
    bool is_initialized_{false};
    
    // 日志
    rclcpp::Logger logger_{rclcpp::get_logger("AGVPlanner")};
    
    // 订阅
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
};

} // namespace planning
} // namespace agv_robot

#endif // AGV_ROBOT_PLANNING_AGV_PLANNER_HPP_