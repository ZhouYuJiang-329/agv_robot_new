/**
 * @file task_scheduler_interface.hpp
 * @brief 任务调度器接口
 *
 * 职责：
 * 1. 任务队列管理
 * 2. 任务优先级调度
 * 3. 资源分配
 * 4. 并发控制
 */

#ifndef AGV_ROBOT_TASK_MANAGER_BASE_TASK_SCHEDULER_INTERFACE_HPP_
#define AGV_ROBOT_TASK_MANAGER_BASE_TASK_SCHEDULER_INTERFACE_HPP_

#include "agv_robot_task_manager/base/task_interface.hpp"
#include "agv_robot_task_manager/base/task_executor_interface.hpp"

namespace agv_robot {
namespace task_manager {

/**
 * @brief 调度策略
 */
enum class SchedulingPolicy {
    FIFO,           // 先进先出
    PRIORITY,       // 优先级
    EARLIEST_DEADLINE,  // 最早截止时间
    ROUND_ROBIN,    // 轮询
    CUSTOM          // 自定义
};

/**
 * @brief 调度配置
 */
struct SchedulerConfig {
    SchedulingPolicy policy{SchedulingPolicy::PRIORITY};
    size_t max_concurrent_tasks{1};         // 最大并发任务数
    size_t max_queue_size{100};             // 最大队列长度
    bool preempt_lower_priority{false};     // 是否抢占低优先级任务
    std::chrono::milliseconds scheduling_interval{100};  // 调度间隔
};

/**
 * @brief 任务调度器接口
 */
class ITaskScheduler {
public:
    virtual ~ITaskScheduler() = default;

    /**
     * @brief 初始化调度器
     * @param config 调度配置
     * @param executor 任务执行器
     * @return 是否成功
     */
    virtual bool initialize(
        const SchedulerConfig& config,
        const ITaskExecutorPtr& executor) = 0;

    /**
     * @brief 提交任务
     * @param task 任务
     * @return 是否成功加入队列
     */
    virtual bool submit(const ITaskPtr& task) = 0;

    /**
     * @brief 取消任务
     * @param task_id 任务ID
     * @return 是否成功
     */
    virtual bool cancel(const std::string& task_id) = 0;

    /**
     * @brief 暂停调度
     */
    virtual void pause() = 0;

    /**
     * @brief 恢复调度
     */
    virtual void resume() = 0;

    /**
     * @brief 清空队列
     * @param include_running 是否包括运行中的任务
     */
    virtual void clear(bool include_running = false) = 0;

    /**
     * @brief 获取队列中的任务
     * @return 任务列表
     */
    virtual std::vector<ITaskPtr> getQueuedTasks() const = 0;

    /**
     * @brief 获取正在执行的任务
     * @return 任务列表
     */
    virtual std::vector<ITaskPtr> getRunningTasks() const = 0;

    /**
     * @brief 获取所有任务
     * @return 任务列表
     */
    virtual std::vector<ITaskPtr> getAllTasks() const = 0;

    /**
     * @brief 获取任务
     * @param task_id 任务ID
     * @return 任务
     */
    virtual ITaskPtr getTask(const std::string& task_id) const = 0;

    /**
     * @brief 获取队列长度
     * @return 队列长度
     */
    virtual size_t getQueueSize() const = 0;

    /**
     * @brief 是否正在运行
     * @return 是否运行中
     */
    virtual bool isRunning() const = 0;

    /**
     * @brief 是否暂停
     * @return 是否暂停
     */
    virtual bool isPaused() const = 0;

    /**
     * @brief 紧急停止
     */
    virtual void emergencyStop() = 0;

    /**
     * @brief 等待任务完成
     * @param task_id 任务ID
     * @param timeout 超时时间
     * @return 是否完成
     */
    virtual bool waitForTask(
        const std::string& task_id,
        std::chrono::milliseconds timeout = std::chrono::milliseconds(-1)) = 0;

    /**
     * @brief 设置调度策略
     * @param policy 策略
     */
    virtual void setPolicy(SchedulingPolicy policy) = 0;

    /**
     * @brief 更新配置
     * @param config 新配置
     */
    virtual void updateConfig(const SchedulerConfig& config) = 0;
};

/**
 * @brief 任务调度器接口智能指针
 */
using ITaskSchedulerPtr = std::shared_ptr<ITaskScheduler>;

} // namespace task_manager
} // namespace agv_robot

#endif // AGV_ROBOT_TASK_MANAGER_BASE_TASK_SCHEDULER_INTERFACE_HPP_
