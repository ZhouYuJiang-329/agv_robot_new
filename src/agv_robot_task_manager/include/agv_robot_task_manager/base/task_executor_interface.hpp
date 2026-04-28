/**
 * @file task_executor_interface.hpp
 * @brief 任务执行器接口
 *
 * 职责：
 * 1. 定义任务执行策略
 * 2. 支持同步/异步/并行执行
 * 3. 处理执行异常和重试
 */

#ifndef AGV_ROBOT_TASK_MANAGER_BASE_TASK_EXECUTOR_INTERFACE_HPP_
#define AGV_ROBOT_TASK_MANAGER_BASE_TASK_EXECUTOR_INTERFACE_HPP_

#include "agv_robot_task_manager/base/task_interface.hpp"
#include "agv_robot_task_manager/base/task_context.hpp"

namespace agv_robot {
namespace task_manager {

/**
 * @brief 执行选项
 */
struct ExecutionOptions {
    bool async{false};                      // 是否异步执行
    int max_retries{0};                     // 最大重试次数
    std::chrono::milliseconds retry_delay{1000};  // 重试延迟
    bool continue_on_failure{false};        // 失败是否继续（用于组合任务）
    std::chrono::milliseconds timeout{60000};     // 超时时间
};

/**
 * @brief 任务执行器接口
 *
 * 负责任务的具体执行
 */
class ITaskExecutor {
public:
    virtual ~ITaskExecutor() = default;

    /**
     * @brief 初始化执行器
     * @param context 任务上下文
     * @return 是否成功
     */
    virtual bool initialize(const TaskContextPtr& context) = 0;

    /**
     * @brief 执行任务
     * @param task 任务
     * @param options 执行选项
     * @return 执行结果
     */
    virtual TaskResult execute(
        const ITaskPtr& task,
        const ExecutionOptions& options = ExecutionOptions{}) = 0;

    /**
     * @brief 取消当前执行
     * @return 是否成功
     */
    virtual bool cancel() = 0;

    /**
     * @brief 暂停当前执行
     * @return 是否成功
     */
    virtual bool pause() = 0;

    /**
     * @brief 恢复当前执行
     * @return 是否成功
     */
    virtual bool resume() = 0;

    /**
     * @brief 是否正在执行
     * @return 是否执行中
     */
    virtual bool isExecuting() const = 0;

    /**
     * @brief 获取当前执行的任务
     * @return 当前任务
     */
    virtual ITaskPtr getCurrentTask() const = 0;

    /**
     * @brief 获取执行器名称
     * @return 名称
     */
    virtual std::string getName() const = 0;
};

/**
 * @brief 任务执行器接口智能指针
 */
using ITaskExecutorPtr = std::shared_ptr<ITaskExecutor>;

} // namespace task_manager
} // namespace agv_robot

#endif // AGV_ROBOT_TASK_MANAGER_BASE_TASK_EXECUTOR_INTERFACE_HPP_
