/**
 * @file composite_task.hpp
 * @brief 组合任务基类
 *
 * 职责：
 * 1. 管理子任务列表
 * 2. 定义子任务执行策略（顺序/并行/选择）
 * 3. 聚合子任务进度和状态
 */

#ifndef AGV_ROBOT_TASK_MANAGER_TASKS_COMPOSITE_TASK_HPP_
#define AGV_ROBOT_TASK_MANAGER_TASKS_COMPOSITE_TASK_HPP_

#include "agv_robot_task_manager/base/task_interface.hpp"
#include "agv_robot_task_manager/base/task_context.hpp"

namespace agv_robot {
namespace task_manager {

/**
 * @brief 组合策略
 */
enum class CompositePolicy {
    SEQUENCE,       // 顺序执行（一个失败则全部失败）
    SEQUENCE_CONTINUE_ON_FAILURE,  // 顺序执行（失败继续）
    PARALLEL,       // 并行执行
    PARALLEL_ALL,   // 并行执行（等待全部完成）
    SELECTOR        // 选择器（一个成功则成功）
};

/**
 * @brief 组合任务基类
 *
 * 包含多个子任务的任务
 */
class CompositeTask : public ITask {
public:
    CompositeTask(const std::string& name,
                  const std::string& type,
                  CompositePolicy policy = CompositePolicy::SEQUENCE,
                  TaskPriority priority = TaskPriority::NORMAL);
    ~CompositeTask() override = default;

    // ==================== ITask 实现 ====================

    std::string getId() const override { return id_; }
    std::string getName() const override { return name_; }
    std::string getType() const override { return type_; }
    std::string getDescription() const override { return description_; }
    TaskPriority getPriority() const override { return priority_; }
    void setPriority(TaskPriority priority) override;

    bool initialize(const TaskContextPtr& context) override;
    TaskResult execute() override;
    bool cancel() override;
    bool pause() override;
    bool resume() override;
    bool reset() override;

    TaskState getState() const override { return state_; }
    TaskProgress getProgress() const override;

    bool isCancellable() const override { return true; }
    bool isPausable() const override { return false; }

    void setProgressCallback(std::function<void(const TaskProgress&)> callback) override {
        progress_callback_ = callback;
    }

    void setStateCallback(std::function<void(TaskState, TaskState)> callback) override {
        state_callback_ = callback;
    }

    bool hasChildren() const override { return !children_.empty(); }
    std::vector<ITaskPtr> getChildren() const override { return children_; }
    void addChild(const ITaskPtr& child) override;

    // ==================== 组合任务特有 ====================

    /**
     * @brief 设置组合策略
     * @param policy 策略
     */
    void setPolicy(CompositePolicy policy) { policy_ = policy; }

    /**
     * @brief 获取组合策略
     * @return 策略
     */
    CompositePolicy getPolicy() const { return policy_; }

    /**
     * @brief 插入子任务
     * @param index 位置
     * @param child 子任务
     */
    void insertChild(size_t index, const ITaskPtr& child);

    /**
     * @brief 移除子任务
     * @param index 位置
     */
    void removeChild(size_t index);

    /**
     * @brief 清空子任务
     */
    void clearChildren();

    /**
     * @brief 获取子任务数量
     * @return 数量
     */
    size_t getChildCount() const { return children_.size(); }

protected:
    // ==================== 子类可重写 ====================

    /**
     * @brief 执行前准备
     * @return 是否成功
     */
    virtual bool onBeforeExecute() { return true; }

    /**
     * @brief 执行后清理
     * @param result 执行结果
     */
    virtual void onAfterExecute(const TaskResult& result) {}

    /**
     * @brief 子任务完成回调
     * @param child 子任务
     * @param result 执行结果
     * @return 是否继续执行
     */
    virtual bool onChildCompleted(const ITaskPtr& child, const TaskResult& result) {
        return result.success;
    }

    // ==================== 辅助函数 ====================

    TaskContextPtr getContext() const { return context_.lock(); }
    void updateProgress();
    void setState(TaskState new_state);

private:
    std::string id_;
    std::string name_;
    std::string type_;
    std::string description_;
    TaskPriority priority_;
    CompositePolicy policy_;
    TaskState state_{TaskState::PENDING};

    std::vector<ITaskPtr> children_;
    std::weak_ptr<TaskContext> context_;

    std::function<void(const TaskProgress&)> progress_callback_;
    std::function<void(TaskState, TaskState)> state_callback_;

    static std::atomic<uint64_t> id_counter_;

    std::string generateId();

    // 执行策略
    TaskResult executeSequence(bool continue_on_failure);
    TaskResult executeParallel(bool wait_all);
    TaskResult executeSelector();
};

} // namespace task_manager
} // namespace agv_robot

#endif // AGV_ROBOT_TASK_MANAGER_TASKS_COMPOSITE_TASK_HPP_
