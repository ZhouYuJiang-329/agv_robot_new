/**
 * @file ultrasound_capability.hpp
 * @brief 超声能力抽象层
 */

#ifndef AGV_ROBOT_CAPABILITY_ULTRASOUND_CAPABILITY_HPP_
#define AGV_ROBOT_CAPABILITY_ULTRASOUND_CAPABILITY_HPP_

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include "agv_robot_common/types.hpp"

namespace agv_robot {
namespace capability {

// 前向声明
namespace perception {
class UltrasoundFusion;
}

/**
 * @brief 超声能力类
 * 提供高层次的超声感知接口
 */
class UltrasoundCapability : public rclcpp::Node {
public:
    explicit UltrasoundCapability(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~UltrasoundCapability() override;

    // ==================== 初始化 ====================
    
    /**
     * @brief 初始化能力层
     * @param fusion 超声融合处理器
     * @return 是否成功
     */
    bool initialize(const std::shared_ptr<perception::UltrasoundFusion>& fusion);
    
    // ==================== 距离测量 ====================
    
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
     * @brief 获取指定方向的距离
     * @param angle 角度 (rad, 0=前方)
     * @param angle_tolerance 角度容差
     * @return 距离 (m)
     */
    double getDistanceAtAngle(double angle, double angle_tolerance = 0.3);
    
    /**
     * @brief 获取所有传感器距离
     * @return 距离数组
     */
    std::vector<double> getAllDistances();
    
    // ==================== 障碍物检测 ====================
    
    /**
     * @brief 检查前方是否有障碍物
     * @param threshold 距离阈值 (m)
     * @return 是否有障碍物
     */
    bool hasObstacleFront(double threshold = 0.5);
    
    /**
     * @brief 检查后方是否有障碍物
     * @param threshold 距离阈值
     * @return 是否有障碍物
     */
    bool hasObstacleRear(double threshold = 0.5);
    
    /**
     * @brief 检查是否有障碍物
     * @param threshold 距离阈值
     * @return 是否有障碍物
     */
    bool hasObstacleAnywhere(double threshold = 0.5);
    
    /**
     * @brief 获取最近的障碍物
     * @return 障碍物信息
     */
    std::optional<Obstacle> getNearestObstacle();
    
    /**
     * @brief 获取指定范围内的障碍物
     * @param angle_start 起始角度
     * @param angle_end 结束角度
     * @param max_distance 最大距离
     * @return 障碍物列表
     */
    std::vector<Obstacle> getObstaclesInRange(double angle_start,
                                               double angle_end,
                                               double max_distance);
    
    // ==================== 安全功能 ====================
    
    /**
     * @brief 检查是否可以前进
     * @param required_distance 需要的距离
     * @return 是否可以前进
     */
    bool canMoveForward(double required_distance = 0.3);
    
    /**
     * @brief 检查是否可以后退
     * @param required_distance 需要的距离
     * @return 是否可以后退
     */
    bool canMoveBackward(double required_distance = 0.3);
    
    /**
     * @brief 获取安全速度限制
     * @param desired_velocity 期望速度
     * @return 安全速度
     */
    double getSafeVelocity(double desired_velocity);
    
    // ==================== 路径规划辅助 ====================
    
    /**
     * @brief 获取可通过的间隙
     * @param robot_width 机器人宽度
     * @return 间隙信息 (角度范围)
     */
    std::vector<std::pair<double, double>> findPassableGaps(double robot_width);
    
    /**
     * @brief 获取最佳通过方向
     * @param target_direction 目标方向
     * @param robot_width 机器人宽度
     * @return 最佳方向
     */
    double findBestPassingDirection(double target_direction, double robot_width);
    
    // ==================== 扫描功能 ====================
    
    /**
     * @brief 执行全向扫描
     * @return 扫描结果
     */
    std::vector<Obstacle> performFullScan();
    
    /**
     * @brief 扫描特定区域
     * @param angle_start 起始角度
     * @param angle_end 结束角度
     * @return 扫描结果
     */
    std::vector<Obstacle> scanSector(double angle_start, double angle_end);

private:
    // 超声融合处理器
    std::shared_ptr<perception::UltrasoundFusion> fusion_;
    
    // 状态
    bool is_initialized_{false};
    
    // 参数
    double safety_margin_{0.1};  // 安全余量 (m)
    double max_range_{4.0};      // 最大测量范围 (m)
    double min_range_{0.05};     // 最小测量范围 (m)
    
    // 日志
    rclcpp::Logger logger_{rclcpp::get_logger("UltrasoundCapability")};
};

} // namespace capability
} // namespace agv_robot

#endif // AGV_ROBOT_CAPABILITY_ULTRASOUND_CAPABILITY_HPP_