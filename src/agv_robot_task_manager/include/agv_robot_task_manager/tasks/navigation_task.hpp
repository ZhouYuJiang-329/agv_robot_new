/**
 * @file navigation_task.hpp
 * @brief 导航任务
 */

#ifndef AGV_ROBOT_TASK_MANAGER_TASKS_NAVIGATION_TASK_HPP_
#define AGV_ROBOT_TASK_MANAGER_TASKS_NAVIGATION_TASK_HPP_

#include "agv_robot_task_manager/tasks/atomic_task.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace agv_robot {
namespace task_manager {

/**
 * @brief 导航任务
 */
class NavigationTask : public AtomicTask {
public:
    NavigationTask(const geometry_msgs::msg::PoseStamped& target_pose,
                   TaskPriority priority = TaskPriority::NORMAL);
    ~NavigationTask() override = default;

protected:
    TaskResult doExecute() override;
    bool doCancel() override;

private:
    geometry_msgs::msg::PoseStamped target_pose_;
};

} // namespace task_manager
} // namespace agv_robot

#endif // AGV_ROBOT_TASK_MANAGER_TASKS_NAVIGATION_TASK_HPP_
