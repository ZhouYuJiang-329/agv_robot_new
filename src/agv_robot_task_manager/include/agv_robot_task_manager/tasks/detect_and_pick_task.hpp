/**
 * @file detect_and_pick_task.hpp
 * @brief 检测并抓取任务
 */

#ifndef AGV_ROBOT_TASK_MANAGER_TASKS_DETECT_AND_PICK_TASK_HPP_
#define AGV_ROBOT_TASK_MANAGER_TASKS_DETECT_AND_PICK_TASK_HPP_

#include "agv_robot_task_manager/tasks/composite_task.hpp"

namespace agv_robot {
namespace task_manager {

/**
 * @brief 检测并抓取任务
 *
 * 组合任务：
 * 1. 检测物体
 * 2. 估计位姿
 * 3. 抓取
 */
class DetectAndPickTask : public CompositeTask {
public:
    explicit DetectAndPickTask(const std::string& object_class,
                               TaskPriority priority = TaskPriority::NORMAL);
    ~DetectAndPickTask() override = default;

private:
    std::string object_class_;

    void buildSubTasks();
};

} // namespace task_manager
} // namespace agv_robot

#endif // AGV_ROBOT_TASK_MANAGER_TASKS_DETECT_AND_PICK_TASK_HPP_
