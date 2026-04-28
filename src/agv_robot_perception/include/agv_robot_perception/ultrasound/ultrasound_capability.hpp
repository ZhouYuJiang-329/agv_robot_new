/**
 * @file ultrasound_capability.hpp
 * @brief 超声能力实现
 *
 * 职责：
 * 1. 组合 UltrasoundDriver + Fusion + Filter + Safety
 * 2. 对外提供统一的障碍物检测接口
 * 3. 实现 IPerceptionCapability 和 IObstacleProvider
 */

#ifndef AGV_ROBOT_PERCEPTION_ULTRASOUND_ULTRASOUND_CAPABILITY_HPP_
#define AGV_ROBOT_PERCEPTION_ULTRASOUND_ULTRASOUND_CAPABILITY_HPP_

#include "agv_robot_perception/base/perception_capability_interface.hpp"
#include "agv_robot_perception/ultrasound/ultrasound_driver_interface.hpp"
#include "agv_robot_perception/ultrasound/obstacle_provider_interface.hpp"

namespace agv_robot {
namespace perception {

// 前向声明
class IObstacleFusion;
class IObstacleFilter;
class ISafetyEvaluator;

/**
 * @brief 超声能力实现
 *
 * 实现 IPerceptionCapability 和 IObstacleProvider 接口
 * 内部组合 Driver + Fusion + Filter + Safety
 */
class UltrasoundCapability :
    public IPerceptionCapability,
    public IObstacleProvider {
public:
    UltrasoundCapability();
    ~UltrasoundCapability() override;

    // ==================== 初始化 ====================

    bool initialize(const std::string& config = "") override;

    void deinitialize() override;

    /**
     * @brief 设置驱动
     * @param driver 超声驱动
     */
    void setDriver(const IUltrasoundDriverPtr& driver);

    /**
     * @brief 设置融合器
     * @param fusion 障碍物融合器
     */
    void setFusion(const std::shared_ptr<IObstacleFusion>& fusion);

    /**
     * @brief 设置滤波器
     * @param filter 障碍物滤波器
     */
    void setFilter(const std::shared_ptr<IObstacleFilter>& filter);

    /**
     * @brief 设置安全评估器
     * @param safety 安全评估器
     */
    void setSafetyEvaluator(const std::shared_ptr<ISafetyEvaluator>& safety);

    // ==================== IPerceptionCapability 实现 ====================

    // 目标检测（超声不支持，返回空）
    std::vector<Detection> detectObjects(
        const DetectionOptions& options = DetectionOptions{}) override;

    std::vector<Detection> detectObjects(
        const std::string& object_class,
        const DetectionOptions& options = DetectionOptions{}) override;

    std::optional<Detection> detectObjectById(
        const std::string& object_id) override;

    std::optional<Detection> detectNearestObject(
        const std::string& object_class = "") override;

    // 位姿估计（超声不支持，返回空）
    std::optional<geometry_msgs::msg::Pose> estimatePose(
        const Detection& detection) override;

    std::vector<geometry_msgs::msg::Pose> detectAndEstimatePoses(
        const std::string& object_class = "") override;

    // 障碍物检测
    std::vector<Obstacle> getObstacles(
        const ObstacleQueryOptions& options = ObstacleQueryOptions{}) override;

    std::optional<Obstacle> getNearestObstacle() override;

    bool hasObstacleInDirection(
        double direction,
        double distance_threshold = 0.5,
        double angle_tolerance = 0.3) override;

    double getClearanceInDirection(
        double direction,
        double max_check_distance = 5.0) override;

    // 场景理解
    std::string getSceneDescription() override;

    bool isSceneSafe(
        double direction = 0.0,
        double required_clearance = 0.5) override;

    // 状态
    bool isReady() const override;

    std::string getLastError() const override;

    // ==================== IObstacleProvider 实现 ====================

    std::vector<Obstacle> getObstacles(
        const ObstacleQueryOptions& options) override;

    std::optional<Obstacle> getObstacleAtAngle(
        double angle,
        double angle_tolerance = 0.3) override;

    bool hasObstacleAtAngle(
        double angle,
        double distance_threshold = 0.5,
        double angle_tolerance = 0.3) override;

    double getClearanceAtAngle(
        double angle,
        double max_check_distance = 5.0) override;

    // ==================== 超声特有接口 ====================

    /**
     * @brief 获取前方距离
     * @return 距离 (m)
     */
    double getFrontDistance();

    /**
     * @brief 获取后方距离
     * @return 距离 (m)
     */
    double getRearDistance();

    /**
     * @brief 获取左侧距离
     * @return 距离 (m)
     */
    double getLeftDistance();

    /**
     * @brief 获取右侧距离
     * @return 距离 (m)
     */
    double getRightDistance();

    /**
     * @brief 获取所有传感器距离
     * @return 距离数组
     */
    std::vector<double> getAllDistances();

    /**
     * @brief 检查前方是否有障碍物
     * @param threshold 距离阈值
     * @return 是否有障碍物
     */
    bool hasObstacleFront(double threshold = 0.5);

    /**
     * @brief 检查是否可以安全移动
     * @param direction 移动方向
     * @param required_distance 需要的安全距离
     * @return 是否可以移动
     */
    bool canMoveSafely(double direction, double required_distance = 0.5);

    /**
     * @brief 获取安全移动速度
     * @param desired_direction 期望方向
     * @param desired_velocity 期望速度
     * @return 安全速度
     */
    double getSafeVelocity(double desired_direction, double desired_velocity);

    /**
     * @brief 设置安全距离
     * @param distance 安全距离 (m)
     */
    void setSafetyDistance(double distance);

    /**
     * @brief 获取安全距离
     * @return 安全距离
     */
    double getSafetyDistance() const;

private:
    // 组件
    IUltrasoundDriverPtr driver_;
    std::shared_ptr<IObstacleFusion> fusion_;
    std::shared_ptr<IObstacleFilter> filter_;
    std::shared_ptr<ISafetyEvaluator> safety_;

    // 状态
    bool initialized_{false};
    std::string last_error_;
    double safety_distance_{0.5};

    // 缓存
    std::vector<UltrasoundReading> last_readings_;
    std::vector<Obstacle> last_obstacles_;

    // 辅助函数
    void updateReadings();
    void updateObstacles();
};

} // namespace perception
} // namespace agv_robot

#endif // AGV_ROBOT_PERCEPTION_ULTRASOUND_ULTRASOUND_CAPABILITY_HPP_
