/**
 * @file perception_capability_interface.hpp
 * @brief 感知能力接口
 *
 * 设计原则：
 * 1. 为TaskManager提供统一的感知接口
 * 2. 隐藏视觉/超声的实现细节
 * 3. 支持多种检测算法和传感器类型的替换
 * 4. 便于测试时mock
 */

#ifndef AGV_ROBOT_PERCEPTION_BASE_PERCEPTION_CAPABILITY_INTERFACE_HPP_
#define AGV_ROBOT_PERCEPTION_BASE_PERCEPTION_CAPABILITY_INTERFACE_HPP_

#include <vector>
#include <optional>
#include <string>
#include <geometry_msgs/msg/pose.hpp>
#include "agv_robot_common/types.hpp"

namespace agv_robot {
namespace perception {

/**
 * @brief 检测选项
 */
struct DetectionOptions {
    std::vector<std::string> target_classes;    // 目标类别（空表示所有）
    float confidence_threshold{0.5f};           // 置信度阈值
    double max_distance{2.0};                   // 最大检测距离 (m)
    bool require_3d{true};                      // 是否需要3D信息
};

/**
 * @brief 障碍物查询选项
 */
struct ObstacleQueryOptions {
    double max_distance{5.0};                   // 最大距离
    double angle_range{M_PI};                   // 角度范围（前方扇形）
    double min_height{0.0};                     // 最小高度
    double max_height{2.0};                     // 最大高度
};

/**
 * @brief 感知能力接口
 *
 * 这是TaskManager直接使用的接口
 * 隐藏了视觉和超声的具体实现
 */
class IPerceptionCapability {
public:
    virtual ~IPerceptionCapability() = default;

    // ==================== 生命周期 ====================

    /**
     * @brief 初始化
     * @param config 配置参数
     * @return 是否成功
     */
    virtual bool initialize(const std::string& config = "") = 0;

    /**
     * @brief 反初始化
     */
    virtual void deinitialize() = 0;

    // ==================== 目标检测 ====================

    /**
     * @brief 检测所有物体
     * @param options 检测选项
     * @return 检测结果列表
     */
    virtual std::vector<Detection> detectObjects(
        const DetectionOptions& options = DetectionOptions{}) = 0;

    /**
     * @brief 检测特定类别的物体
     * @param object_class 物体类别
     * @param options 检测选项
     * @return 检测结果列表
     */
    virtual std::vector<Detection> detectObjects(
        const std::string& object_class,
        const DetectionOptions& options = DetectionOptions{}) = 0;

    /**
     * @brief 检测特定ID的物体
     * @param object_id 物体ID
     * @return 检测结果
     */
    virtual std::optional<Detection> detectObjectById(
        const std::string& object_id) = 0;

    /**
     * @brief 检测最近的物体
     * @param object_class 物体类别（可选）
     * @return 检测结果
     */
    virtual std::optional<Detection> detectNearestObject(
        const std::string& object_class = "") = 0;

    // ==================== 位姿估计 ====================

    /**
     * @brief 估计检测到的物体的位姿
     * @param detection 检测结果
     * @return 3D位姿
     */
    virtual std::optional<geometry_msgs::msg::Pose> estimatePose(
        const Detection& detection) = 0;

    /**
     * @brief 检测并估计位姿（便捷方法）
     * @param object_class 物体类别
     * @return 位姿列表
     */
    virtual std::vector<geometry_msgs::msg::Pose> detectAndEstimatePoses(
        const std::string& object_class = "") = 0;

    // ==================== 障碍物检测 ====================

    /**
     * @brief 获取所有障碍物
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
     * @brief 检查指定方向是否有障碍物
     * @param direction 方向（弧度，0为前方）
     * @param distance_threshold 距离阈值
     * @param angle_tolerance 角度容差
     * @return 是否有障碍物
     */
    virtual bool hasObstacleInDirection(
        double direction,
        double distance_threshold = 0.5,
        double angle_tolerance = 0.3) = 0;

    /**
     * @brief 获取指定方向的安全距离
     * @param direction 方向
     * @param max_check_distance 最大检查距离
     * @return 到最近障碍物的距离
     */
    virtual double getClearanceInDirection(
        double direction,
        double max_check_distance = 5.0) = 0;

    // ==================== 场景理解 ====================

    /**
     * @brief 获取场景描述
     * @return 场景描述字符串
     */
    virtual std::string getSceneDescription() = 0;

    /**
     * @brief 检查场景是否安全（用于导航）
     * @param direction 期望移动方向
     * @param required_clearance 需要的安全距离
     * @return 是否安全
     */
    virtual bool isSceneSafe(
        double direction = 0.0,
        double required_clearance = 0.5) = 0;

    // ==================== 传感器状态 ====================

    /**
     * @brief 是否就绪
     * @return 是否就绪
     */
    virtual bool isReady() const = 0;

    /**
     * @brief 获取最后错误信息
     * @return 错误信息
     */
    virtual std::string getLastError() const = 0;
};

/**
 * @brief 感知能力接口智能指针
 */
using IPerceptionCapabilityPtr = std::shared_ptr<IPerceptionCapability>;

} // namespace perception
} // namespace agv_robot

#endif // AGV_ROBOT_PERCEPTION_BASE_PERCEPTION_CAPABILITY_INTERFACE_HPP_
