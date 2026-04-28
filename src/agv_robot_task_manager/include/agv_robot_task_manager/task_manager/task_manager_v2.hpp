/**
 * @file task_manager_v2.hpp
 * @brief 任务管理器 V2
 *
 * 架构：
 * 1. TaskManager = TaskScheduler + TaskExecutor + ROS封装
 * 2. 所有任务实现 ITask 接口
 * 3. 使用 TaskContext 传递能力和参数
 * 4. 支持原子任务和组合任务
 * 5. 与ROS解耦，便于测试
 */

#ifndef AGV_ROBOT_TASK_MANAGER_TASK_MANAGER_TASK_MANAGER_V2_HPP_
#define AGV_ROBOT_TASK_MANAGER_TASK_MANAGER_V2_HPP_

#include <rclcpp/rclcpp.hpp>

#include "agv_robot_task_manager/base/task_interface.hpp"
#include "agv_robot_task_manager/base/task_context.hpp"
#include "agv_robot_task_manager/base/task_scheduler_interface.hpp"
#include "agv_robot_task_manager/base/task_executor_interface.hpp"

namespace agv_robot {
namespace task_manager {

/**
 * @brief 任务管理器 V2
 *
 * 整合调度器、执行器和ROS接口
 */
class TaskManagerV2 : public rclcpp::Node {
public:
    explicit TaskManagerV2(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~TaskManagerV2() override;

    // ==================== 初始化 ====================

    /**
     * @brief 初始化
     * @param scheduler 任务调度器
     * @param executor 任务执行器
     * @return 是否成功
     */
    bool initialize(const ITaskSchedulerPtr& scheduler,
                    const ITaskExecutorPtr& executor);

    /**
     * @brief 设置能力接口
     * @param arm 机械臂能力
     * @param agv AGV能力
     * @param vision 视觉能力
     * @param ultrasound 超声能力
     * @param perception 感知能力
     */
    void setCapabilities(
        const capability::IArmCapabilityPtr& arm,
        const capability::IAGVCapabilityPtr& agv,
        const capability::IVisionCapabilityPtr& vision,
        const capability::IUltrasoundCapabilityPtr& ultrasound,
        const perception::IPerceptionCapabilityPtr& perception);

    // ==================== 任务提交 ====================

    /**
     * @brief 提交任务
     * @param task 任务
     * @return 任务ID
     */
    std::string submitTask(const ITaskPtr& task);

    /**
     * @brief 提交导航任务
     * @param target_pose 目标位姿
     * @param priority 优先级
     * @return 任务ID
     */
    std::string submitNavigationTask(
        const geometry_msgs::msg::PoseStamped& target_pose,
        TaskPriority priority = TaskPriority::NORMAL);

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
     * @brief 提交检测并抓取任务
     * @param object_class 物体类别
     * @param priority 优先级
     * @return 任务ID
     */
    std::string submitDetectAndPickTask(
        const std::string& object_class,
        TaskPriority priority = TaskPriority::NORMAL);

    // ==================== 任务管理 ====================

    /**
     * @brief 取消任务
     * @param task_id 任务ID
     * @return 是否成功
     */
    bool cancelTask(const std::string& task_id);

    /**
     * @brief 暂停
     */
    void pause();

    /**
     * @brief 恢复
     */
    void resume();

    /**
     * @brief 清空队列
     */
    void clearQueue();

    /**
     * @brief 获取任务状态
     * @param task_id 任务ID
     * @return 状态
     */
    TaskState getTaskState(const std::string& task_id);

    /**
     * @brief 获取任务进度
     * @param task_id 任务ID
     * @return 进度
     */
    TaskProgress getTaskProgress(const std::string& task_id);

    /**
     * @brief 等待任务完成
     * @param task_id 任务ID
     * @param timeout 超时时间
     * @return 是否成功完成
     */
    bool waitForTask(const std::string& task_id, double timeout = 60.0);

    // ==================== 紧急处理 ====================

    /**
     * @brief 紧急停止
     */
    void emergencyStop();

    /**
     * @brief 是否紧急停止中
     * @return 是否急停
     */
    bool isEmergencyStopped() const;

    // ==================== 查询 ====================

    /**
     * @brief 获取所有任务
     * @return 任务列表
     */
    std::vector<ITaskPtr> getAllTasks() const;

    /**
     * @brief 获取队列中的任务
     * @return 任务列表
     */
    std::vector<ITaskPtr> getQueuedTasks() const;

    /**
     * @brief 获取正在执行的任务
     * @return 任务列表
     */
    std::vector<ITaskPtr> getRunningTasks() const;

    /**
     * @brief 获取任务
     * @param task_id 任务ID
     * @return 任务
     */
    ITaskPtr getTask(const std::string& task_id) const;

    /**
     * @brief 获取任务上下文
     * @return 上下文
     */
    TaskContextPtr getContext() const { return context_; }

private:
    // 核心组件
    ITaskSchedulerPtr scheduler_;
    ITaskExecutorPtr executor_;
    TaskContextPtr context_;

    // 状态
    bool initialized_{false};
    bool emergency_stopped_{false};

    // ROS2通信
    rclcpp::Service<agv_robot_msgs::srv::SubmitTask>::SharedPtr submit_task_service_;
    rclcpp::Service<agv_robot_msgs::srv::CancelTask>::SharedPtr cancel_task_service_;
    rclcpp::Service<agv_robot_msgs::srv::GetTaskStatus>::SharedPtr get_status_service_;

    rclcpp::Publisher<agv_robot_msgs::msg::TaskStatus>::SharedPtr task_status_pub_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    // 回调函数
    void submitTaskServiceCallback(
        const std::shared_ptr<agv_robot_msgs::srv::SubmitTask::Request> request,
        std::shared_ptr<agv_robot_msgs::srv::SubmitTask::Response> response);

    void cancelTaskServiceCallback(
        const std::shared_ptr<agv_robot_msgs::srv::CancelTask::Request> request,
        std::shared_ptr<agv_robot_msgs::srv::CancelTask::Response> response);

    void getTaskStatusServiceCallback(
        const std::shared_ptr<agv_robot_msgs::srv::GetTaskStatus::Request> request,
        std::shared_ptr<agv_robot_msgs::srv::GetTaskStatus::Response> response);

    void statusTimerCallback();

    // 辅助函数
    void onTaskStateChanged(TaskState old_state, TaskState new_state);
    void onTaskProgressUpdated(const TaskProgress& progress);
};

} // namespace task_manager
} // namespace agv_robot

#endif // AGV_ROBOT_TASK_MANAGER_TASK_MANAGER_TASK_MANAGER_V2_HPP_
