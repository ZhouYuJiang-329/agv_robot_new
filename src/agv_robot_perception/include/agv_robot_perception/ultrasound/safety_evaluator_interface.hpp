/**
 * @file safety_evaluator_interface.hpp
 * @brief 安全评估接口
 *
 * 职责：
 * 1. 评估运动方向的安全性
 * 2. 计算安全速度
 * 3. 提供安全决策建议
 */

#ifndef AGV_ROBOT_PERCEPTION_ULTRASOUND_SAFETY_EVALUATOR_INTERFACE_HPP_
#define AGV_ROBOT_PERCEPTION_ULTRASOUND_SAFETY_EVALUATOR_INTERFACE_HPP_

#include <vector>
#include "agv_robot_common/types.hpp"

namespace agv_robot {
namespace perception {

/**
 * @brief 安全评估配置
 */
struct SafetyConfig {
    double safety_margin{0.2};              // 安全余量 (m)
    double max_deceleration{1.0};           // 最大减速度 (m/s^2)
    double reaction_time{0.2};              // 反应时间 (s)
    double min_clearance{0.3};              // 最小间隙 (m)
    double slowdown_factor{0.5};            // 减速因子
};

/**
 * @brief 安全评估结果
 */
struct SafetyAssessment {
    bool is_safe{false};                    // 是否安全
    double recommended_velocity{0.0};       // 推荐速度
    double clearance{0.0};                  // 间隙距离
    std::string reason;                     // 评估原因
    double risk_level{0.0};                 // 风险等级 (0.0-1.0)
};

/**
 * @brief 安全评估接口
 */
class ISafetyEvaluator {
public:
    virtual ~ISafetyEvaluator() = default;

    /**
     * @brief 初始化
     * @param config 安全配置
     * @return 是否成功
     */
    virtual bool initialize(const SafetyConfig& config = SafetyConfig{}) = 0;

    /**
     * @brief 评估运动安全性
     * @param obstacles 障碍物列表
     * @param direction 运动方向 (rad)
     * @param desired_velocity 期望速度
     * @return 安全评估结果
     */
    virtual SafetyAssessment assess(
        const std::vector<Obstacle>& obstacles,
        double direction,
        double desired_velocity) = 0;

    /**
     * @brief 检查是否可以移动
     * @param obstacles 障碍物列表
     * @param direction 方向
     * @param required_distance 需要的距离
     * @return 是否可以移动
     */
    virtual bool canMove(
        const std::vector<Obstacle>& obstacles,
        double direction,
        double required_distance) = 0;

    /**
     * @brief 计算安全速度
     * @param obstacles 障碍物列表
     * @param direction 方向
     * @param desired_velocity 期望速度
     * @return 安全速度
     */
    virtual double computeSafeVelocity(
        const std::vector<Obstacle>& obstacles,
        double direction,
        double desired_velocity) = 0;

    /**
     * @brief 获取所有安全方向
     * @param obstacles 障碍物列表
     * @param required_clearance 需要的间隙
     * @return 安全方向列表
     */
    virtual std::vector<double> getSafeDirections(
        const std::vector<Obstacle>& obstacles,
        double required_clearance = 0.5) = 0;

    /**
     * @brief 更新配置
     * @param config 新配置
     */
    virtual void updateConfig(const SafetyConfig& config) = 0;
};

/**
 * @brief 安全评估接口智能指针
 */
using ISafetyEvaluatorPtr = std::shared_ptr<ISafetyEvaluator>;

} // namespace perception
} // namespace agv_robot

#endif // AGV_ROBOT_PERCEPTION_ULTRASOUND_SAFETY_EVALUATOR_INTERFACE_HPP_
