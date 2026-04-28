/**
 * @file task_interface.hpp
 * @brief 任务接口定义
 *
 * 设计原则：
 * 1. 任务是可执行的最小单元
 * 2. 支持同步和异步执行
 * 3. 可取消、可暂停、可恢复
 * 4. 支持子任务（组合模式）
 */

#ifndef AGV_ROBOT_TASK_MANAGER_BASE_TASK_INTERFACE_HPP_
#define AGV_ROBOT_TASK_MANAGER_BASE_TASK_INTERFACE_HPP_

#include <string>
#include <memory>
#include <functional>
#include <any>
#include <chrono>
#include "agv_robot_common/types.hpp"

namespace agv_robot {
namespace task_manager {

// 前向声明
class TaskContext;

/**
 * @brief 任务状态
 */
enum class TaskState {
    PENDING,        // 等待执行
    RUNNING,        // 执行中
    PAUSED,         // 已暂停
    COMPLETED,      // 已完成
    CANCELLED,      // 已取消
    FAILED,         // 失败
    TIMEOUT         // 超时
};

/**
 * @brief 任务优先级
 */
enum class TaskPriority {
    IDLE = 0,       // 空闲任务
    LOW = 1,        // 低优先级
    NORMAL = 2,     // 普通优先级
    HIGH = 3,       // 高优先级
    CRITICAL = 4    // 关键任务
};

/**
 * @brief 任务结果
 */
struct TaskResult {
    bool success{false};
    std::string message;
    std::any data;
    
    static TaskResult Success(const std::string& msg = "") {
        return TaskResult{true, msg, {}};
    }
    
    static TaskResult Failure(const std::string& msg) {
        return TaskResult{false, msg, {}};
    }
};

/**
 * @brief 任务进度
 */
struct TaskProgress {
    float percent{0.0f};                    // 进度百分比 (0-100)
    std::string stage;                      // 当前阶段
    std::string detail;                     // 详细信息
    std::chrono::milliseconds elapsed;      // 已用时间
    std::chrono::milliseconds estimated;    // 预计剩余时间
};

/**
 * @brief 任务接口
 *
 * 所有任务的基类接口
 */
class ITask {
public:
    virtual ~ITask() = default;

    // ==================== 基本信息 ====================

    /**
     * @brief 获取任务ID
     * @return 任务ID
     */
    virtual std::string getId() const = 0;

    /**
     * @brief 获取任务名称
     * @return 任务名称
     */
    virtual std::string getName() const = 0;

    /**
     * @brief 获取任务类型
     * @return 任务类型
     */
    virtual std::string getType() const = 0;

    /**
     * @brief 获取任务描述
     * @return 描述
     */
    virtual std::string getDescription() const = 0;

    /**
     * @brief 获取优先级
     * @return 优先级
     */
    virtual TaskPriority getPriority() const = 0;

    /**
     * @brief 设置优先级
     * @param priority 优先级
     */
    virtual void setPriority(TaskPriority priority) = 0;

    // ==================== 生命周期 ====================

    /**
     * @brief 初始化任务
     * @param context 任务上下文
     * @return 是否成功
     */
    virtual bool initialize(const std::shared_ptr<TaskContext>& context) = 0;

    /**
     * @brief 执行任务
     * @return 执行结果
     */
    virtual TaskResult execute() = 0;

    /**
     * @brief 取消任务
     * @return 是否成功
     */
    virtual bool cancel() = 0;

    /**
     * @brief 暂停任务
     * @return 是否成功
     */
    virtual bool pause() = 0;

    /**
     * @brief 恢复任务
     * @return 是否成功
     */
    virtual bool resume() = 0;

    /**
     * @brief 重置任务
     * @return 是否成功
     */
    virtual bool reset() = 0;

    // ==================== 状态查询 ====================

    /**
     * @brief 获取当前状态
     * @return 状态
     */
    virtual TaskState getState() const = 0;

    /**
     * @brief 获取进度
     * @return 进度信息
     */
    virtual TaskProgress getProgress() const = 0;

    /**
     * @brief 是否可取消
     * @return 是否可取消
     */
    virtual bool isCancellable() const = 0;

    /**
     * @brief 是否可暂停
     * @return 是否可暂停
     */
    virtual bool isPausable() const = 0;

    /**
     * @brief 是否完成
     * @return 是否完成
     */
    virtual bool isCompleted() const {
        return getState() == TaskState::COMPLETED;
    }

    /**
     * @brief 是否失败
     * @return 是否失败
     */
    virtual bool isFailed() const {
        return getState() == TaskState::FAILED;
    }

    // ==================== 回调 ====================

    /**
     * @brief 设置进度回调
     * @param callback 回调函数
     */
    virtual void setProgressCallback(
        std::function<void(const TaskProgress&)> callback) = 0;

    /**
     * @brief 设置状态变化回调
     * @param callback 回调函数
     */
    virtual void setStateCallback(
        std::function<void(TaskState, TaskState)> callback) = 0;

    // ==================== 组合模式 ====================

    /**
     * @brief 是否有子任务
     * @return 是否有子任务
     */
    virtual bool hasChildren() const = 0;

    /**
     * @brief 获取子任务
     * @return 子任务列表
     */
    virtual std::vector<std::shared_ptr<ITask>> getChildren() const = 0;

    /**
     * @brief 添加子任务
     * @param child 子任务
     */
    virtual void addChild(const std::shared_ptr<ITask>& child) = 0;

    /**
     * @brief 是否是原子任务（无子任务）
     * @return 是否原子
     */
    virtual bool isAtomic() const {
        return !hasChildren();
    }
};

/**
 * @brief 任务接口智能指针
 */
using ITaskPtr = std::shared_ptr<ITask>;

} // namespace task_manager
} // namespace agv_robot

#endif // AGV_ROBOT_TASK_MANAGER_BASE_TASK_INTERFACE_HPP_
