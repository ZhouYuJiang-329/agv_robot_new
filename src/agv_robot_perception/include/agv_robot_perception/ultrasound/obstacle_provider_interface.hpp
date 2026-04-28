/**
 * @file obstacle_provider_interface.hpp
 * @brief 障碍物提供者接口
 *
 * 职责：
 * 1. 提供障碍物的抽象接口
 * 2. 可被 UltrasoundFusion 或其他传感器实现
 */

#ifndef AGV_ROBOT_PERCEPTION_ULTRASOUND_OBSTACLE_PROVIDER_INTERFACE_HPP_
#define AGV_ROBOT_PERCEPTION_ULTRASOUND_OBSTACLE_PROVIDER_INTERFACE_HPP_

#include <vector>
#include <optional>
#include "agv_robot_common/types.hpp"

namespace agv_robot {
namespace perception {

/**
 * @brief 障碍物查询选项
 */
struct ObstacleQueryOptions {
    double max_distance{5.0};           // 最大距离
    double angle_start{-M_PI};          // 起始角度
    double angle_end{M_PI};             // 结束角度
    double min_confidence{0.5};         // 最小置信度
};

/**
 * @brief 障碍物提供者接口
 */
class IObstacleProvider {
public:
    virtual ~IObstacleProvider() = default;

    /**
     * @brief 获取障碍物列表
     * @param options 查询选项
     * @return 障碍物列表
     */
    virtual std::vector<Obstacle> getObstacles(
        const ObstacleQueryOptions& options = ObstacleQueryOptions{}) = 0;

    /**
     * @brief 获取最近的障碍物
     * @return 障碍物
     */
    virtual std::optional<Obstacle> getNearestObstacle() = 0;

    /**
     * @brief 获取指定方向的障碍物
     * @param angle 方向（弧度）
     * @param angle_tolerance 角度容差
     * @return 障碍物
     */
    virtual std::optional<Obstacle> getObstacleAtAngle(
        double angle,
        double angle_tolerance = 0.3) = 0;

    /**
     * @brief 检查指定方向是否有障碍物
     * @param angle 方向
     * @param distance_threshold 距离阈值
     * @param angle_tolerance 角度容差
     * @return 是否有障碍物
     */
    virtual bool hasObstacleAtAngle(
        double angle,
        double distance_threshold = 0.5,
        double angle_tolerance = 0.3) = 0;

    /**
     * @brief 获取指定方向的安全距离
     * @param angle 方向
     * @param max_check_distance 最大检查距离
     * @return 到最近障碍物的距离
     */
    virtual double getClearanceAtAngle(
        double angle,
        double max_check_distance = 5.0) = 0;
};

/**
 * @brief 障碍物提供者接口智能指针
 */
using IObstacleProviderPtr = std::shared_ptr<IObstacleProvider>;

} // namespace perception
} // namespace agv_robot

#endif // AGV_ROBOT_PERCEPTION_ULTRASOUND_OBSTACLE_PROVIDER_INTERFACE_HPP_
