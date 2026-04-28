/**
 * @file agv_planner_interface.hpp
 * @brief AGV规划器接口
 */

#ifndef AGV_ROBOT_PLANNING_BASE_AGV_PLANNER_INTERFACE_HPP_
#define AGV_ROBOT_PLANNING_BASE_AGV_PLANNER_INTERFACE_HPP_

#include <vector>
#include <memory>
#include <optional>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "agv_robot_planning/base/planner_interface.hpp"

namespace agv_robot {
namespace planning {

// 前向声明
struct AGVPlanningResult;

/**
 * @brief 规划模式
 */
enum class AGVPlanningMode {
    GLOBAL,     // 全局规划
    LOCAL,      // 局部规划
    HYBRID      // 混合规划
};

/**
 * @brief 规划选项
 */
struct AGVPlanningOptions {
    AGVPlanningMode mode{AGVPlanningMode::HYBRID};
    double planning_time{5.0};           // 规划时间限制 (秒)
    double goal_tolerance{0.1};          // 目标位置容差 (m)
    double yaw_tolerance{0.1};           // 目标角度容差 (rad)
    bool allow_unknown{false};           // 是否允许未知区域
    double inflation_radius{0.3};        // 膨胀半径 (m)
    double max_path_length{100.0};       // 最大路径长度 (m)
    bool use_dijkstra{false};            // 使用Dijkstra（否则A*）
    double cost_factor{3.0};             // 代价因子
};

/**
 * @brief 路径质量评估
 */
struct PathQuality {
    double length{0.0};                  // 路径长度
    double smoothness{0.0};              // 平滑度
    double clearance{0.0};               // 与障碍物的最小距离
    int num_turns{0};                    // 转弯次数
    double estimated_time{0.0};          // 预估时间
};

/**
 * @brief AGV规划结果
 */
struct AGVPlanningResult : public PlanningResult {
    std::optional<nav_msgs::msg::Path> path;      // 规划路径
    std::optional<nav_msgs::msg::Path> local_path; // 局部路径（混合模式）
    PathQuality quality;                           // 路径质量
    int replan_count{0};                           // 重规划次数
    
    /**
     * @brief 获取有效路径
     * @return 路径（优先局部路径，其次全局路径）
     */
    std::optional<nav_msgs::msg::Path> getEffectivePath() const {
        if (local_path && !local_path->poses.empty()) {
            return local_path;
        }
        return path;
    }
};

/**
 * @brief 障碍物信息
 */
struct ObstacleInfo {
    geometry_msgs::msg::Pose pose;       // 位置
    double radius{0.0};                  // 半径
    double velocity{0.0};                // 速度
    double direction{0.0};               // 方向
};

/**
 * @brief 动态障碍物列表
 */
using DynamicObstacles = std::vector<ObstacleInfo>;

/**
 * @brief AGV规划器接口
 * 
 * 定义AGV路径规划的标准接口，支持：
 * - 全局路径规划
 * - 局部路径规划（避障）
 * - 混合规划
 * - 动态重规划
 */
class IAGVPlanner : public IPlanner {
public:
    ~IAGVPlanner() override = default;

    // ==================== 全局规划 ====================
    
    /**
     * @brief 规划到目标点的路径
     * @param start 起始位姿
     * @param goal 目标位姿
     * @param options 规划选项
     * @return 规划结果
     */
    virtual AGVPlanningResult planPath(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal,
        const AGVPlanningOptions& options = AGVPlanningOptions{}) = 0;
    
    /**
     * @brief 规划通过多个路径点
     * @param start 起始位姿
     * @param waypoints 路径点列表
     * @param options 规划选项
     * @return 规划结果
     */
    virtual AGVPlanningResult planThroughWaypoints(
        const geometry_msgs::msg::PoseStamped& start,
        const std::vector<geometry_msgs::msg::PoseStamped>& waypoints,
        const AGVPlanningOptions& options = AGVPlanningOptions{}) = 0;
    
    /**
     * @brief 规划覆盖路径（全覆盖清扫等）
     * @param start 起始位姿
     * @param area_bounds 区域边界
     * @param options 规划选项
     * @return 规划结果
     */
    virtual AGVPlanningResult planCoveragePath(
        const geometry_msgs::msg::PoseStamped& start,
        const std::vector<geometry_msgs::msg::PoseStamped>& area_bounds,
        const AGVPlanningOptions& options = AGVPlanningOptions{}) = 0;
    
    // ==================== 局部规划 ====================
    
    /**
     * @brief 局部路径规划（避障）
     * @param global_path 全局路径
     * @param current_pose 当前位姿
     * @param current_velocity 当前速度
     * @param obstacles 障碍物列表
     * @param options 规划选项
     * @return 规划结果
     */
    virtual AGVPlanningResult planLocalPath(
        const nav_msgs::msg::Path& global_path,
        const geometry_msgs::msg::PoseStamped& current_pose,
        const geometry_msgs::msg::Twist& current_velocity,
        const DynamicObstacles& obstacles,
        const AGVPlanningOptions& options = AGVPlanningOptions{}) = 0;
    
    /**
     * @brief 快速避障（紧急情况）
     * @param current_pose 当前位姿
     * @param current_velocity 当前速度
     * @param obstacles 障碍物列表
     * @return 紧急路径
     */
    virtual std::optional<nav_msgs::msg::Path> planEmergencyAvoidance(
        const geometry_msgs::msg::PoseStamped& current_pose,
        const geometry_msgs::msg::Twist& current_velocity,
        const DynamicObstacles& obstacles) = 0;
    
    // ==================== 重规划 ====================
    
    /**
     * @brief 重新规划路径
     * @param current_pose 当前位姿
     * @param goal 目标位姿
     * @param options 规划选项
     * @return 规划结果
     */
    virtual AGVPlanningResult replanPath(
        const geometry_msgs::msg::PoseStamped& current_pose,
        const geometry_msgs::msg::PoseStamped& goal,
        const AGVPlanningOptions& options = AGVPlanningOptions{}) = 0;
    
    /**
     * @brief 检查是否需要重规划
     * @param current_path 当前路径
     * @param current_pose 当前位姿
     * @param obstacles 障碍物
     * @return 是否需要重规划
     */
    virtual bool needReplan(
        const nav_msgs::msg::Path& current_path,
        const geometry_msgs::msg::PoseStamped& current_pose,
        const DynamicObstacles& obstacles) = 0;
    
    // ==================== 地图管理 ====================
    
    /**
     * @brief 更新全局地图
     * @param map 占用栅格地图
     * @return 是否成功
     */
    virtual bool updateGlobalMap(const nav_msgs::msg::OccupancyGrid& map) = 0;
    
    /**
     * @brief 更新局部地图
     * @param map 局部栅格地图
     * @return 是否成功
     */
    virtual bool updateLocalMap(const nav_msgs::msg::OccupancyGrid& map) = 0;
    
    /**
     * @brief 清除地图
     * @return 是否成功
     */
    virtual bool clearMap() = 0;
    
    // ==================== 路径处理 ====================
    
    /**
     * @brief 平滑路径
     * @param path 原始路径
     * @return 平滑后的路径
     */
    virtual nav_msgs::msg::Path smoothPath(const nav_msgs::msg::Path& path) = 0;
    
    /**
     * @brief 裁剪路径（从当前位置到目标）
     * @param path 原始路径
     * @param current_pose 当前位姿
     * @param lookahead_distance 前瞻距离
     * @return 裁剪后的路径
     */
    virtual nav_msgs::msg::Path prunePath(
        const nav_msgs::msg::Path& path,
        const geometry_msgs::msg::PoseStamped& current_pose,
        double lookahead_distance) = 0;
    
    /**
     * @brief 评估路径质量
     * @param path 路径
     * @return 质量评估
     */
    virtual PathQuality evaluatePathQuality(const nav_msgs::msg::Path& path) = 0;
    
    // ==================== 查询 ====================
    
    /**
     * @brief 检查位姿有效性
     * @param pose 位姿
     * @return 是否有效
     */
    virtual bool isPoseValid(const geometry_msgs::msg::PoseStamped& pose) = 0;
    
    /**
     * @brief 获取最近的有效位姿
     * @param pose 查询位姿
     * @param max_distance 最大搜索距离
     * @return 有效位姿
     */
    virtual std::optional<geometry_msgs::msg::PoseStamped> getNearestValidPose(
        const geometry_msgs::msg::PoseStamped& pose,
        double max_distance = 1.0) = 0;
    
    /**
     * @brief 获取地图边界
     * @return 地图边界 [min_x, max_x, min_y, max_y]
     */
    virtual std::vector<double> getMapBounds() const = 0;
    
    /**
     * @brief 获取地图分辨率
     * @return 分辨率 (m/cell)
     */
    virtual double getMapResolution() const = 0;
};

/**
 * @brief AGV规划器接口智能指针
 */
using IAGVPlannerPtr = std::shared_ptr<IAGVPlanner>;

} // namespace planning
} // namespace agv_robot

#endif // AGV_ROBOT_PLANNING_BASE_AGV_PLANNER_INTERFACE_HPP_
