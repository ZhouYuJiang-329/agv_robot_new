/**
 * @file obstacle_filter_interface.hpp
 * @brief 障碍物滤波接口
 *
 * 职责：
 * 1. 对障碍物数据进行时序滤波
 * 2. 去除噪声和抖动
 * 3. 平滑障碍物轨迹
 */

#ifndef AGV_ROBOT_PERCEPTION_ULTRASOUND_OBSTACLE_FILTER_INTERFACE_HPP_
#define AGV_ROBOT_PERCEPTION_ULTRASOUND_OBSTACLE_FILTER_INTERFACE_HPP_

#include <vector>
#include "agv_robot_common/types.hpp"

namespace agv_robot {
namespace perception {

/**
 * @brief 滤波配置
 */
struct FilterConfig {
    double position_noise{0.05};            // 位置噪声 (m)
    double velocity_noise{0.1};             // 速度噪声 (m/s)
    double process_noise{0.01};             // 过程噪声
    double outlier_threshold{3.0};          // 异常值阈值 (标准差倍数)
    int min_detection_count{3};             // 最小检测次数
    double max_age{1.0};                    // 最大跟踪年龄 (s)
};

/**
 * @brief 障碍物滤波接口
 */
class IObstacleFilter {
public:
    virtual ~IObstacleFilter() = default;

    /**
     * @brief 初始化
     * @param config 滤波配置
     * @return 是否成功
     */
    virtual bool initialize(const FilterConfig& config = FilterConfig{}) = 0;

    /**
     * @brief 滤波障碍物
     * @param raw_obstacles 原始障碍物
     * @param timestamp 当前时间戳
     * @return 滤波后的障碍物
     */
    virtual std::vector<Obstacle> filter(
        const std::vector<Obstacle>& raw_obstacles,
        double timestamp) = 0;

    /**
     * @brief 重置滤波器
     */
    virtual void reset() = 0;

    /**
     * @brief 更新配置
     * @param config 新配置
     */
    virtual void updateConfig(const FilterConfig& config) = 0;

    /**
     * @brief 获取跟踪的障碍物数量
     * @return 数量
     */
    virtual size_t getTrackedCount() const = 0;
};

/**
 * @brief 障碍物滤波接口智能指针
 */
using IObstacleFilterPtr = std::shared_ptr<IObstacleFilter>;

} // namespace perception
} // namespace agv_robot

#endif // AGV_ROBOT_PERCEPTION_ULTRASOUND_OBSTACLE_FILTER_INTERFACE_HPP_
