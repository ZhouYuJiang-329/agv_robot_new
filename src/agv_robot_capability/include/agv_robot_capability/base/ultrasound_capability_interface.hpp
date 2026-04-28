/**
 * @file ultrasound_capability_interface.hpp
 * @brief 超声能力接口
 */

#ifndef AGV_ROBOT_CAPABILITY_BASE_ULTRASOUND_CAPABILITY_INTERFACE_HPP_
#define AGV_ROBOT_CAPABILITY_BASE_ULTRASOUND_CAPABILITY_INTERFACE_HPP_

#include <vector>
#include <optional>
#include "agv_robot_common/types.hpp"
#include "agv_robot_capability/base/capability_interface.hpp"

namespace agv_robot {
namespace capability {

// 前向声明
namespace perception {
class UltrasoundFusion;
}

/**
 * @brief 超声能力接口
 *
 * 提供高层次的超声感知接口
 * 这是应用程序直接调用的接口
 */
class IUltrasoundCapability : public ICapability {
public:
    ~IUltrasoundCapability() override = default;

    // ==================== 初始化 ====================

    /**
     * @brief 初始化（带超声融合处理器）
     * @param fusion 超声融合处理器
     * @return 是否成功
     */
    virtual bool initialize(
        const std::shared_ptr<perception::UltrasoundFusion>& fusion) = 0;

    // ==================== 距离测量 ====================

    /**
     * @brief 获取前方距离
     * @return 距离 (m)
     */
    virtual double getFrontDistance() = 0;

    /**
     * @brief 获取后方距离
     * @return 距离 (m)
     */
    virtual double getRearDistance() = 0;

    /**
     * @brief 获取左侧距离
     * @return 距离 (m)
     */
    virtual double getLeftDistance() = 0;

    /**
     * @brief 获取右侧距离
     * @return 距离 (m)
     */
    virtual double getRightDistance() = 0;

    /**
     * @brief 获取指定方向的距离
     * @param angle 角度 (rad, 0=前方, 逆时针为正)
     * @param angle_tolerance 角度容差
     * @return 距离 (m)
     */
    virtual double getDistanceAtAngle(
        double angle,
        double angle_tolerance = 0.3) = 0;

    /**
     * @brief 获取所有传感器距离
     * @return 距离数组
     */
    virtual std::vector<double> getAllDistances() = 0;

    /**
     * @brief 获取最近的障碍物距离
     * @return 距离 (m)
     */
    virtual double getNearestObstacleDistance() = 0;

    /**
     * @brief 获取最近的障碍物方向
     * @return 方向 (rad)
     */
    virtual double getNearestObstacleDirection() = 0;

    // ==================== 障碍物检测 ====================

    /**
     * @brief 检查前方是否有障碍物
     * @param threshold 距离阈值 (m)
     * @return 是否有障碍物
     */
    virtual bool hasObstacleFront(double threshold = 0.5) = 0;

    /**
     * @brief 检查后方是否有障碍物
     * @param threshold 距离阈值
     * @return 是否有障碍物
     */
    virtual bool hasObstacleRear(double threshold = 0.5) = 0;

    /**
     * @brief 检查左侧是否有障碍物
     * @param threshold 距离阈值
     * @return 是否有障碍物
     */
    virtual bool hasObstacleLeft(double threshold = 0.5) = 0;

    /**
     * @brief 检查右侧是否有障碍物
     * @param threshold 距离阈值
     * @return 是否有障碍物
     */
    virtual bool hasObstacleRight(double threshold = 0.5) = 0;

    /**
     * @brief 检查是否有障碍物
     * @param threshold 距离阈值
     * @return 是否有障碍物
     */
    virtual bool hasObstacleAnywhere(double threshold = 0.5) = 0;

    /**
     * @brief 检查指定方向是否有障碍物
     * @param angle 角度 (rad)
     * @param threshold 距离阈值
     * @param angle_tolerance 角度容差
     * @return 是否有障碍物
     */
    virtual bool hasObstacleAtAngle(
        double angle,
        double threshold = 0.5,
        double angle_tolerance = 0.3) = 0;

    /**
     * @brief 检查指定角度范围内是否有障碍物
     * @param angle_start 起始角度
     * @param angle_end 结束角度
     * @param threshold 距离阈值
     * @return 是否有障碍物
     */
    virtual bool hasObstacleInRange(
        double angle_start,
        double angle_end,
        double threshold = 0.5) = 0;

    // ==================== 安全区域检查 ====================

    /**
     * @brief 检查是否可以安全移动
     * @param direction 移动方向 (rad)
     * @param required_distance 需要的安全距离
     * @return 是否可以移动
     */
    virtual bool canMoveSafely(
        double direction,
        double required_distance = 0.5) = 0;

    /**
     * @brief 获取安全移动速度
     * @param desired_direction 期望方向
     * @param desired_velocity 期望速度
     * @return 安全速度
     */
    virtual double getSafeVelocity(
        double desired_direction,
        double desired_velocity) = 0;

    /**
     * @brief 获取所有安全移动方向
     * @param required_distance 需要的安全距离
     * @return 安全方向列表
     */
    virtual std::vector<double> getSafeDirections(
        double required_distance = 0.5) = 0;

    // ==================== 障碍物信息 ====================

    /**
     * @brief 获取障碍物列表
     * @return 障碍物列表
     */
    virtual std::vector<Obstacle> getObstacles() = 0;

    /**
     * @brief 获取指定方向的障碍物
     * @param angle 角度
     * @param angle_tolerance 角度容差
     * @return 障碍物
     */
    virtual std::optional<Obstacle> getObstacleAtAngle(
        double angle,
        double angle_tolerance = 0.3) = 0;

    /**
     * @brief 获取最近的障碍物
     * @return 障碍物
     */
    virtual std::optional<Obstacle> getNearestObstacle() = 0;

    // ==================== 配置 ====================

    /**
     * @brief 设置安全距离
     * @param distance 安全距离 (m)
     */
    virtual void setSafetyDistance(double distance) = 0;

    /**
     * @brief 获取安全距离
     * @return 安全距离
     */
    virtual double getSafetyDistance() const = 0;

    /**
     * @brief 设置检测范围
     * @param min_range 最小范围 (m)
     * @param max_range 最大范围 (m)
     */
    virtual void setDetectionRange(double min_range, double max_range) = 0;

    /**
     * @brief 启用/禁用传感器
     * @param sensor_id 传感器ID
     * @param enable 是否启用
     * @return 是否成功
     */
    virtual bool setSensorEnabled(
        size_t sensor_id,
        bool enable) = 0;

    /**
     * @brief 获取传感器数量
     * @return 传感器数量
     */
    virtual size_t getSensorCount() const = 0;
};

/**
 * @brief 超声能力接口智能指针
 */
using IUltrasoundCapabilityPtr = std::shared_ptr<IUltrasoundCapability>;

} // namespace capability
} // namespace agv_robot

#endif // AGV_ROBOT_CAPABILITY_BASE_ULTRASOUND_CAPABILITY_INTERFACE_HPP_
