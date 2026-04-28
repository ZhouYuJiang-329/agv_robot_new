/**
 * @file atomic_task.hpp
 * @brief 原子任务基类
 *
 * 职责：
 * 1. 提供原子任务的默认实现
 * 2. 管理任务状态转换
 * 3. 处理回调通知
 */

#ifndef AGV_ROBOT_TASK_MANAGER_TASKS_ATOMIC_TASK_HPP_
#define AGV_ROBOT_TASK_MANAGER_TASKS_ATOMIC_TASK_HPP_

#include "agv_robot_task_manager/base/task_interface.hpp"
#include "agv_robot_task_manager/base/task_context.hpp"

namespace agv_robot {
namespace task_manager {

/**
 * @brief 原子任务基类
 *
 * 所有原子任务（无子任务）的基类
 */
class AtomicTask : public ITask {
public:
    AtomicTask(const std::string& name,
               const std::string& type,
               TaskPriority priority = TaskPriority::NORMAL);
    ~AtomicTask() override = default;

    // ==================== ITask 实现 ====================

    std::string getId() const override { return id_; }
    std::string getName() const override { return name_; }
    std::string getType() const override { return type_; }
    std::string getDescription() const override { return description_; }
    TaskPriority getPriority() const override { return priority_; }
    void setPriority(TaskPriority priority) override { priority_ = priority; }

    bool initialize(const TaskContextPtr& context) override;
    TaskResult execute() override;
    bool cancel() override;
    bool pause() override;
    bool resume() override;
    bool reset() override;

    TaskState getState() const override { return state_; }
    TaskProgress getProgress() const override { return progress_; }

    bool isCancellable() const override { return true; }
    bool isPausable() const override { return false; }

    void setProgressCallback(std::function<void(const TaskProgress&)> callback) override {
        progress_callback_ = callback;
    }

    void setStateCallback(std::function<void(TaskState, TaskState)> callback) override {
        state_callback_ = callback;
    }

    bool hasChildren() const override { return false; }
    std::vector<ITaskPtr> getChildren() const override { return {}; }
    void addChild(const ITaskPtr& /*child*/) override {
        throw std::runtime_error("Atomic task cannot have children");
    }

protected:
    // ==================== 子类必须实现 ====================

    /**
     * @brief 执行具体任务逻辑
     * @return 执行结果
     */
    virtual TaskResult doExecute() = 0;

    /**
     * @brief 取消任务逻辑
     * @return 是否成功
     */
    virtual bool doCancel() { return true; }

    // ==================== 辅助函数 ====================

    /**
     * @brief 更新进度
     * @param percent 百分比
     * @param stage 阶段
     * @param detail 详情
     */
    void updateProgress(float percent,
                        const std::string& stage = "",
                        const std::string& detail = "");

    /**
     * @brief 设置状态
     * @param new_state 新状态
     */
    void setState(TaskState new_state);

    /**
     * @brief 获取上下文
     * @return 上下文
     */
    TaskContextPtr getContext() const { return context_.lock(); }

    /**
     * @brief 检查是否被取消
     * @return 是否取消
     */
    bool isCancelled() const { return cancelled_; }

private:
    std::string id_;
    std::string name_;
    std::string type_;
    std::string description_;
    TaskPriority priority_;
    TaskState state_{TaskState::PENDING};
    TaskProgress progress_;

    std::weak_ptr<TaskContext> context_;
    std::atomic<bool> cancelled_{false};

    std::function<void(const TaskProgress&)> progress_callback_;
    std::function<void(TaskState, TaskState)> state_callback_;

    static std::atomic<uint64_t> id_counter_;

    std::string generateId();
};

} // namespace task_manager
} // namespace agv_robot

#endif // AGV_ROBOT_TASK_MANAGER_TASKS_ATOMIC_TASK_HPP_
