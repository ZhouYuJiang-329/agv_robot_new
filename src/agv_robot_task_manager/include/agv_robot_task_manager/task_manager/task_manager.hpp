/**
 * @file task_manager.hpp
 * @brief 任务管理器
 */

#ifndef AGV_ROBOT_TASK_MANAGER_TASK_MANAGER_HPP_
#define AGV_ROBOT_TASK_MANAGER_TASK_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <queue>
#include <memory>
#include "agv_robot_common/types.hpp"
#include "agv_robot_msgs/action/pick_place.hpp"
#include "agv_robot_msgs/action/patrol.hpp"
#include "agv_robot_msgs/action/move_arm.hpp"

namespace agv_robot {
namespace task_manager {

// 前向声明
namespace capability {
class ArmCapability;
class AGVCapability;
class VisionCapability;
class UltrasoundCapability;
}

/**
 * @brief 任务类型
 */
enum class TaskType {
    PICK_PLACE,     // 抓取放置
    PATROL,         // 巡逻
    MOVE_ARM,       // 机械臂运动
    NAVIGATE,       // 导航
    INSPECTION,     // 检测
    CUSTOM          // 自定义
};

/**
 * @brief 任务优先级
 */
enum class TaskPriority {
    LOW = 0,
    NORMAL = 1,
    HIGH = 2,
    CRITICAL = 3
};

/**
 * @brief 任务结构
 */
struct Task {
    std::string id;
    TaskType type;
    TaskPriority priority;
    std::string description;
    std::map<std::string, std::any> parameters;
    TaskStatus status;
    std::chrono::steady_clock::time_point created_time;
    std::chrono::steady_clock::time_point start_time;
    std::chrono::steady_clock::time_point end_time;
    std::string error_message;
};

/**
 * @brief 任务比较器 (用于优先级队列)
 */
struct TaskComparator {
    bool operator()(const std::shared_ptr<Task>& a,
                    const std::shared_ptr<Task>& b) const {
        return static_cast<int>(a->priority) < static_cast<int>(b->priority);
    }
};

/**
 * @brief 任务管理器
 * 负责任务的调度、执行和管理
 */
class TaskManager : public rclcpp::Node {
public:
    explicit TaskManager(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~TaskManager() override;

    // ==================== 初始化 ====================
    
    /**
     * @brief 初始化任务管理器
     * @param arm 机械臂能力
     * @param agv AGV能力
     * @param vision 视觉能力
     * @param ultrasound 超声能力
     * @return 是否成功
     */
    bool initialize(
        const std::shared_ptr<capability::ArmCapability>& arm,
        const std::shared_ptr<capability::AGVCapability>& agv,
        const std::shared_ptr<capability::VisionCapability>& vision,
        const std::shared_ptr<capability::UltrasoundCapability>& ultrasound);
    
    // ==================== 任务提交 ====================
    
    /**
     * @brief 提交抓取放置任务
     * @param object_id 物体ID
     * @param object_pose 物体位姿
     * @param place_pose 放置位姿
     * @param priority 优先级
     * @return 任务ID
     */
    std::string submitPickPlaceTask(
        const std::string& object_id,
        const geometry_msgs::msg::Pose& object_pose,
        const geometry_msgs::msg::Pose& place_pose,
        TaskPriority priority = TaskPriority::NORMAL);
    
    /**
     * @brief 提交巡逻任务
     * @param waypoints 路径点
     * @param loop_count 循环次数
     * @param priority 优先级
     * @return 任务ID
     */
    std::string submitPatrolTask(
        const std::vector<geometry_msgs::msg::PoseStamped>& waypoints,
        int loop_count = 1,
        TaskPriority priority = TaskPriority::NORMAL);
    
    /**
     * @brief 提交导航任务
     * @param target_pose 目标位姿
     * @param priority 优先级
     * @return 任务ID
     */
    std::string submitNavigateTask(
        const geometry_msgs::msg::PoseStamped& target_pose,
        TaskPriority priority = TaskPriority::NORMAL);
    
    /**
     * @brief 提交机械臂运动任务
     * @param target_type 目标类型 ("joint" or "cartesian")
     * @param target 目标位置
     * @param priority 优先级
     * @return 任务ID
     */
    std::string submitArmMotionTask(
        const std::string& target_type,
        const geometry_msgs::msg::Pose& target,
        TaskPriority priority = TaskPriority::NORMAL);
    
    /**
     * @brief 提交自定义任务
     * @param description 任务描述
     * @param executor 执行函数
     * @param priority 优先级
     * @return 任务ID
     */
    std::string submitCustomTask(
        const std::string& description,
        std::function<bool()> executor,
        TaskPriority priority = TaskPriority::NORMAL);
    
    // ==================== 任务管理 ====================
    
    /**
     * @brief 取消任务
     * @param task_id 任务ID
     * @return 是否成功
     */
    bool cancelTask(const std::string& task_id);
    
    /**
     * @brief 暂停任务队列
     */
    void pauseQueue();
    
    /**
     * @brief 恢复任务队列
     */
    void resumeQueue();
    
    /**
     * @brief 清空任务队列
     */
    void clearQueue();
    
    /**
     * @brief 获取任务状态
     * @param task_id 任务ID
     * @return 任务状态
     */
    TaskStatus getTaskStatus(const std::string& task_id);
    
    /**
     * @brief 获取任务信息
     * @param task_id 任务ID
     * @return 任务信息
     */
    std::optional<Task> getTaskInfo(const std::string& task_id);
    
    /**
     * @brief 获取所有任务列表
     * @return 任务列表
     */
    std::vector<Task> getAllTasks();
    
    /**
     * @brief 获取活跃任务列表
     * @return 活跃任务列表
     */
    std::vector<Task> getActiveTasks();
    
    // ==================== 状态获取 ====================
    
    /**
     * @brief 是否正在执行任务
     * @return 是否执行中
     */
    bool isExecuting() const { return is_executing_; }
    
    /**
     * @brief 获取当前任务ID
     * @return 当前任务ID
     */
    std::string getCurrentTaskId() const { return current_task_id_; }
    
    /**
     * @brief 等待任务完成
     * @param task_id 任务ID
     * @param timeout 超时时间 (秒)
     * @return 是否成功完成
     */
    bool waitForTask(const std::string& task_id, double timeout = 60.0);
    
    // ==================== 紧急处理 ====================
    
    /**
     * @brief 紧急停止所有任务
     */
    void emergencyStop();
    
    /**
     * @brief 恢复从急停状态
     */
    void recoverFromEmergencyStop();

private:
    // 能力引用
    std::shared_ptr<capability::ArmCapability> arm_;
    std::shared_ptr<capability::AGVCapability> agv_;
    std::shared_ptr<capability::VisionCapability> vision_;
    std::shared_ptr<capability::UltrasoundCapability> ultrasound_;
    
    // 任务队列
    std::priority_queue<std::shared_ptr<Task>,
                        std::vector<std::shared_ptr<Task>>,
                        TaskComparator> task_queue_;
    
    // 任务映射
    std::unordered_map<std::string, std::shared_ptr<Task>> tasks_;
    
    // 当前任务
    std::string current_task_id_;
    std::shared_ptr<Task> current_task_;
    
    // 状态
    bool is_initialized_{false};
    bool is_executing_{false};
    bool queue_paused_{false};
    bool emergency_stop_{false};
    
    // 任务ID计数器
    std::atomic<uint64_t> task_id_counter_{0};
    
    // 执行线程
    std::thread execution_thread_;
    std::atomic<bool> stop_thread_{false};
    
    // 互斥锁
    mutable std::mutex queue_mutex_;
    mutable std::mutex tasks_mutex_;
    std::condition_variable queue_cv_;
    
    // 日志
    rclcpp::Logger logger_{rclcpp::get_logger("TaskManager")};
    
    // Action servers
    rclcpp_action::Server<agv_robot_msgs::action::PickPlace>::SharedPtr pick_place_server_;
    rclcpp_action::Server<agv_robot_msgs::action::Patrol>::SharedPtr patrol_server_;
    rclcpp_action::Server<agv_robot_msgs::action::MoveArm>::SharedPtr move_arm_server_;
    
    // 辅助函数
    std::string generateTaskId();
    void executionLoop();
    bool executeTask(const std::shared_ptr<Task>& task);
    
    // 任务执行器
    bool executePickPlace(const std::shared_ptr<Task>& task);
    bool executePatrol(const std::shared_ptr<Task>& task);
    bool executeNavigate(const std::shared_ptr<Task>& task);
    bool executeArmMotion(const std::shared_ptr<Task>& task);
    
    // Action callbacks
    rclcpp_action::GoalResponse handlePickPlaceGoal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const agv_robot_msgs::action::PickPlace::Goal> goal);
    rclcpp_action::CancelResponse handlePickPlaceCancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<agv_robot_msgs::action::PickPlace>> goal_handle);
    void executePickPlaceAction(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<agv_robot_msgs::action::PickPlace>> goal_handle);
};

} // namespace task_manager
} // namespace agv_robot

#endif // AGV_ROBOT_TASK_MANAGER_TASK_MANAGER_HPP_
