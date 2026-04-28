/**
 * @file pick_place_task.hpp
 * @brief 抓取放置任务
 */

#ifndef AGV_ROBOT_TASK_MANAGER_TASKS_PICK_PLACE_TASK_HPP_
#define AGV_ROBOT_TASK_MANAGER_TASKS_PICK_PLACE_TASK_HPP_

#include "agv_robot_task_manager/tasks/composite_task.hpp"
#include <geometry_msgs/msg/pose.hpp>

namespace agv_robot {
namespace task_manager {

/**
 * @brief 抓取放置任务
 *
 * 组合任务：
 * 1. 移动到物体上方
 * 2. 下降
 * 3. 抓取
 * 4. 上升
 * 5. 移动到放置点上方
 * 6. 下降
 * 7. 释放
 * 8. 上升
 */
class PickPlaceTask : public CompositeTask {
public:
    PickPlaceTask(const std::string& object_id,
                  const geometry_msgs::msg::Pose& object_pose,
                  const geometry_msgs::msg::Pose& place_pose,
                  TaskPriority priority = TaskPriority::NORMAL);
    ~PickPlaceTask() override = default;

private:
    std::string object_id_;
    geometry_msgs::msg::Pose object_pose_;
    geometry_msgs::msg::Pose place_pose_;

    void buildSubTasks();
};

} // namespace task_manager
} // namespace agv_robot

#endif // AGV_ROBOT_TASK_MANAGER_TASKS_PICK_PLACE_TASK_HPP_
