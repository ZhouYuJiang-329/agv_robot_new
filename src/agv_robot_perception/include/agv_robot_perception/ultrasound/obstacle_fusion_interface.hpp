/**
 * @file obstacle_fusion_interface.hpp
 * @brief 障碍物融合接口
 *
 * 职责：
 * 1. 将多个超声传感器的读数融合为障碍物列表
 * 2. 处理传感器之间的重叠区域
 * 3. 生成统一的障碍物表示
 */

#ifndef AGV_ROBOT_PERCEPTION_ULTRASOUND_OBSTACLE_FUSION_INTERFACE_HPP_
#define AGV_ROBOT_PERCEPTION_ULTRASOUND_OBSTACLE_FUSION_INTERFACE_HPP_

#include <vector>
#include "agv_robot_common/types.hpp"
#include "agv_robot_perception/ultrasound/ultrasound_driver_interface.hpp"

namespace agv_robot {
namespace perception {

/**
 * @brief 融合配置
 */
struct FusionConfig {
    double min_obstacle_distance{0.1};      // 最小障碍物距离
    double max_obstacle_distance{5.0};      // 最大障碍物距离
    double obstacle_merge_threshold{0.3};   // 障碍物合并阈值 (m)
    bool use_confidence_weighting{true};    // 是否使用置信度加权
};

/**
 * @brief 障碍物融合接口
 */
class IObstacleFusion {
public:
    virtual ~IObstacleFusion() = default;

    /**
     * @brief 初始化
     * @param config 融合配置
     * @return 是否成功
     */
    virtual bool initialize(const FusionConfig& config = FusionConfig{}) = 0;

    /**
     * @brief 融合超声读数为障碍物
     * @param readings 超声读数
     * @param sensor_configs 传感器配置
     * @return 障碍物列表
     */
    virtual std::vector<Obstacle> fuse(
        const std::vector<UltrasoundReading>& readings,
        const std::vector<UltrasoundSensorConfig>& sensor_configs) = 0;

    /**
     * @brief 更新配置
     * @param config 新配置
     */
    virtual void updateConfig(const FusionConfig& config) = 0;

    /**
     * @brief 获取最后融合时间
     * @return 时间戳
     */
    virtual double getLastFusionTime() const = 0;
};

/**
 * @brief 障碍物融合接口智能指针
 */
using IObstacleFusionPtr = std::shared_ptr<IObstacleFusion>;

} // namespace perception
} // namespace agv_robot

#endif // AGV_ROBOT_PERCEPTION_ULTRASOUND_OBSTACLE_FUSION_INTERFACE_HPP_
