/**
 * @file ultrasound_fusion.hpp
 * @brief 超声数据融合
 */

#ifndef AGV_ROBOT_PERCEPTION_ULTRASOUND_FUSION_HPP_
#define AGV_ROBOT_PERCEPTION_ULTRASOUND_FUSION_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "agv_robot_common/types.hpp"
#include "agv_robot_hal/ultrasound/ultrasound_interface.hpp"

namespace agv_robot {
namespace perception {

/**
 * @brief 超声融合处理器
 * 负责多超声传感器数据融合和障碍物检测
 */
class UltrasoundFusion : public rclcpp::Node {
public:
    explicit UltrasoundFusion(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~UltrasoundFusion() override;

    // ==================== 初始化 ====================
    
    /**
     * @brief 初始化融合器
     * @param ultrasound 超声接口
     * @return 是否成功
     */
    bool initialize(const std::shared_ptr<hal::UltrasoundInterface>& ultrasound);
    
    /**
     * @brief 配置传感器
     * @param configs 传感器配置列表
     */
    void configureSensors(const std::vector<UltrasoundSensorConfig>& configs);
    
    // ==================== 数据获取 ====================
    
    /**
     * @brief 获取融合后的障碍物列表
     * @return 障碍物列表
     */
    std::vector<Obstacle> getObstacles();
    
    /**
     * @brief 获取指定方向的障碍物
     * @param angle 角度 (rad, 0=前方)
     * @param angle_tolerance 角度容差
     * @return 障碍物
     */
    std::optional<Obstacle> getObstacleAtAngle(double angle,
                                                double angle_tolerance = 0.3);
    
    /**
     * @brief 获取指定范围内的障碍物
     * @param angle_start 起始角度
     * @param angle_end 结束角度
     * @return 障碍物列表
     */
    std::vector<Obstacle> getObstaclesInRange(double angle_start,
                                               double angle_end);
    
    /**
     * @brief 获取最近障碍物
     * @return 最近障碍物
     */
    std::optional<Obstacle> getNearestObstacle();
    
    /**
     * @brief 获取指定方向的距离
     * @param angle 角度
     * @param angle_tolerance 角度容差
     * @return 距离 (m)
     */
    double getDistanceAtAngle(double angle, double angle_tolerance = 0.3);
    
    /**
     * @brief 获取所有传感器距离
     * @return 距离数组
     */
    std::vector<double> getAllDistances();
    
    // ==================== 障碍物地图 ====================
    
    /**
     * @brief 获取局部障碍物地图
     * @return 占用栅格地图
     */
    nav_msgs::msg::OccupancyGrid getObstacleMap();
    
    /**
     * @brief 获取障碍物点云
     * @return 点云
     */
    sensor_msgs::msg::PointCloud2 getObstaclePointCloud();
    
    /**
     * @brief 更新障碍物地图
     */
    void updateObstacleMap();
    
    // ==================== 安全功能 ====================
    
    /**
     * @brief 检查是否有障碍物
     * @param angle_start 起始角度
     * @param angle_end 结束角度
     * @param distance_threshold 距离阈值
     * @return 是否有障碍物
     */
    bool hasObstacle(double angle_start,
                     double angle_end,
                     double distance_threshold);
    
    /**
     * @brief 检查是否可以移动
     * @param direction 方向 (rad)
     * @param required_distance 需要距离
     * @return 是否可以移动
     */
    bool canMove(double direction, double required_distance);
    
    /**
     * @brief 获取安全速度
     * @param desired_direction 期望方向
     * @param desired_velocity 期望速度
     * @return 安全速度
     */
    double getSafeVelocity(double desired_direction, double desired_velocity);
    
    // ==================== 路径规划辅助 ====================
    
    /**
     * @brief 查找可通过的间隙
     * @param robot_width 机器人宽度
     * @return 间隙列表 (角度范围)
     */
    std::vector<std::pair<double, double>> findPassableGaps(double robot_width);
    
    /**
     * @brief 查找最佳通过方向
     * @param target_direction 目标方向
     * @param robot_width 机器人宽度
     * @return 最佳方向
     */
    double findBestDirection(double target_direction, double robot_width);
    
    // ==================== 滤波处理 ====================
    
    /**
     * @brief 设置滤波参数
     * @param window_size 滑动窗口大小
     * @param outlier_threshold 异常值阈值
     */
    void setFilterParams(size_t window_size, double outlier_threshold);
    
    /**
     * @brief 应用中值滤波
     * @param data 输入数据
     * @return 滤波后数据
     */
    std::vector<double> applyMedianFilter(const std::vector<double>& data);
    
    /**
     * @brief 应用卡尔曼滤波
     * @param measurement 测量值
     * @param sensor_id 传感器ID
     * @return 滤波后值
     */
    double applyKalmanFilter(double measurement, uint8_t sensor_id);

private:
    // 超声接口
    std::shared_ptr<hal::UltrasoundInterface> ultrasound_;
    
    // 传感器配置
    std::vector<UltrasoundSensorConfig> sensor_configs_;
    
    // 障碍物数据
    std::vector<Obstacle> obstacles_;
    nav_msgs::msg::OccupancyGrid obstacle_map_;
    
    // 历史数据 (用于滤波)
    std::vector<std::vector<double>> distance_history_;
    size_t history_window_size_{5};
    
    // 卡尔曼滤波器状态
    struct KalmanState {
        double x{0.0};      // 状态估计
        double p{1.0};      // 估计误差协方差
        double q{0.01};     // 过程噪声
        double r{0.1};      // 测量噪声
    };
    std::vector<KalmanState> kalman_states_;
    
    // 参数
    double outlier_threshold_{0.5};  // m
    double max_range_{4.0};          // m
    double min_range_{0.05};         // m
    double map_resolution_{0.05};    // m
    double map_size_{5.0};           // m (正方形)
    
    // 状态
    bool is_initialized_{false};
    
    // 日志
    rclcpp::Logger logger_{rclcpp::get_logger("UltrasoundFusion")};
    
    // 发布
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    
    // 处理循环
    rclcpp::TimerBase::SharedPtr fusion_timer_;
    void fusionLoop();
    
    // 辅助函数
    void updateObstacles();
    geometry_msgs::msg::Point sensorToPoint(
        const UltrasoundSensorConfig& config,
        double distance);
    bool isOutlier(double value, const std::vector<double>& history);
};

} // namespace perception
} // namespace agv_robot

#endif // AGV_ROBOT_PERCEPTION_ULTRASOUND_FUSION_HPP_
