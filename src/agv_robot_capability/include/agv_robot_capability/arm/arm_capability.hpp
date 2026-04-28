/**
 * @file arm_capability.hpp
 * @brief 机械臂能力抽象层
 */

#ifndef AGV_ROBOT_CAPABILITY_ARM_CAPABILITY_HPP_
#define AGV_ROBOT_CAPABILITY_ARM_CAPABILITY_HPP_

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include "agv_robot_common/types.hpp"

namespace agv_robot {
namespace capability {

// 前向声明
namespace planning {
class ArmMotionPlanner;
}
namespace control {
class ArmController;
}

/**
 * @brief 机械臂能力类
 * 提供高层次的机械臂操作接口，封装规划和控制细节
 */
class ArmCapability : public rclcpp::Node {
public:
    explicit ArmCapability(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~ArmCapability() override;

    // ==================== 初始化 ====================
    
    /**
     * @brief 初始化能力层
     * @param planner 运动规划器
     * @param controller 控制器
     * @return 是否成功
     */
    bool initialize(
        const std::shared_ptr<planning::ArmMotionPlanner>& planner,
        const std::shared_ptr<control::ArmController>& controller);
    
    // ==================== 关节空间运动 ====================
    
    /**
     * @brief 关节空间运动到目标位置
     * @param joint_positions 目标关节位置 (rad)
     * @param velocity 速度缩放 (0.0-1.0)
     * @param acceleration 加速度缩放 (0.0-1.0)
     * @param async 是否异步执行
     * @return 是否成功
     */
    bool moveJ(const std::vector<double>& joint_positions,
               double velocity = 0.5,
               double acceleration = 0.5,
               bool async = false);
    
    /**
     * @brief 关节空间相对运动
     * @param joint_deltas 关节增量 (rad)
     * @param velocity 速度缩放
     * @param acceleration 加速度缩放
     * @return 是否成功
     */
    bool moveJRel(const std::vector<double>& joint_deltas,
                  double velocity = 0.5,
                  double acceleration = 0.5);
    
    // ==================== 笛卡尔空间运动 ====================
    
    /**
     * @brief 直线运动到目标位姿
     * @param target_pose 目标位姿
     * @param velocity 速度 (m/s)
     * @param acceleration 加速度 (m/s^2)
     * @param async 是否异步执行
     * @return 是否成功
     */
    bool moveL(const geometry_msgs::msg::Pose& target_pose,
               double velocity = 0.1,
               double acceleration = 0.5,
               bool async = false);
    
    /**
     * @brief 圆弧运动
     * @param via_point 中间点
     * @param target_pose 目标点
     * @param velocity 速度
     * @param acceleration 加速度
     * @return 是否成功
     */
    bool moveC(const geometry_msgs::msg::Pose& via_point,
               const geometry_msgs::msg::Pose& target_pose,
               double velocity = 0.1,
               double acceleration = 0.5);
    
    /**
     * @brief 相对运动
     * @param delta_x X方向增量 (m)
     * @param delta_y Y方向增量 (m)
     * @param delta_z Z方向增量 (m)
     * @param velocity 速度
     * @return 是否成功
     */
    bool moveRel(double delta_x, double delta_y, double delta_z,
                 double velocity = 0.1);
    
    // ==================== 姿态调整 ====================
    
    /**
     * @brief 旋转末端执行器
     * @param roll 绕X轴旋转 (rad)
     * @param pitch 绕Y轴旋转 (rad)
     * @param yaw 绕Z轴旋转 (rad)
     * @param velocity 速度
     * @return 是否成功
     */
    bool rotateEndEffector(double roll, double pitch, double yaw,
                           double velocity = 0.1);
    
    // ==================== 手爪控制 ====================
    
    /**
     * @brief 打开手爪
     * @param velocity 速度 (0.0-1.0)
     * @return 是否成功
     */
    bool openGripper(double velocity = 1.0);
    
    /**
     * @brief 关闭手爪
     * @param force 力矩 (N)
     * @param velocity 速度
     * @return 是否成功
     */
    bool closeGripper(double force = 10.0, double velocity = 1.0);
    
    /**
     * @brief 设置手爪位置
     * @param position 位置 (0.0=打开, 1.0=闭合)
     * @param force 力矩
     * @return 是否成功
     */
    bool setGripper(double position, double force = 10.0);
    
    // ==================== 轨迹执行 ====================
    
    /**
     * @brief 执行预规划轨迹
     * @param trajectory 轨迹
     * @param async 是否异步
     * @return 是否成功
     */
    bool executeTrajectory(const moveit_msgs::msg::RobotTrajectory& trajectory,
                           bool async = false);
    
    /**
     * @brief 停止当前运动
     * @return 是否成功
     */
    bool stop();
    
    /**
     * @brief 暂停当前运动
     * @return 是否成功
     */
    bool pause();
    
    /**
     * @brief 恢复当前运动
     * @return 是否成功
     */
    bool resume();
    
    // ==================== 状态获取 ====================
    
    /**
     * @brief 获取当前关节状态
     * @return 关节状态
     */
    sensor_msgs::msg::JointState getCurrentJointState() const;
    
    /**
     * @brief 获取当前末端位姿
     * @return 位姿
     */
    geometry_msgs::msg::Pose getCurrentPose() const;
    
    /**
     * @brief 检查是否正在运动
     * @return 是否运动中
     */
    bool isMoving() const;
    
    /**
     * @brief 等待运动完成
     * @param timeout 超时时间 (秒)
     * @return 是否成功完成
     */
    bool waitForCompletion(double timeout = 30.0);
    
    // ==================== 高级功能 ====================
    
    /**
     * @brief 伺服模式运动 (实时控制)
     * @param twist 速度命令
     * @return 是否成功
     */
    bool servo(const geometry_msgs::msg::TwistStamped& twist);
    
    /**
     * @brief 力控制模式
     * @param force 目标力
     * @return 是否成功
     */
    bool forceControl(const geometry_msgs::msg::Wrench& force);
    
    /**
     * @brief 搜索运动 (用于插孔等)
     * @param direction 搜索方向
     * @param max_distance 最大搜索距离
     * @param force_threshold 力阈值
     * @return 是否成功
     */
    bool searchMotion(const geometry_msgs::msg::Vector3& direction,
                      double max_distance,
                      double force_threshold);

private:
    // 规划器
    std::shared_ptr<planning::ArmMotionPlanner> planner_;
    
    // 控制器
    std::shared_ptr<control::ArmController> controller_;
    
    // MoveIt 接口
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    
    // 参数
    std::string planning_group_;
    double default_velocity_;
    double default_acceleration_;
    
    // 状态
    bool is_initialized_{false};
    bool is_moving_{false};
    
    // 日志
    rclcpp::Logger logger_{rclcpp::get_logger("ArmCapability")};
};

} // namespace capability
} // namespace agv_robot

#endif // AGV_ROBOT_CAPABILITY_ARM_CAPABILITY_HPP_