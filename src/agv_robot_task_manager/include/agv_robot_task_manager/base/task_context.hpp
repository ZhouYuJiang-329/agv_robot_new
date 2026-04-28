/**
 * @file task_context.hpp
 * @brief 任务上下文
 *
 * 职责：
 * 1. 为任务提供执行环境
 * 2. 包含所有能力接口的引用
 * 3. 提供参数传递机制
 * 4. 支持任务间数据共享
 */

#ifndef AGV_ROBOT_TASK_MANAGER_BASE_TASK_CONTEXT_HPP_
#define AGV_ROBOT_TASK_MANAGER_BASE_TASK_CONTEXT_HPP_

#include <memory>
#include <any>
#include <unordered_map>
#include <string>

// Capability 接口
#include "agv_robot_capability/base/arm_capability_interface.hpp"
#include "agv_robot_capability/base/agv_capability_interface.hpp"
#include "agv_robot_capability/base/vision_capability_interface.hpp"
#include "agv_robot_capability/base/ultrasound_capability_interface.hpp"
#include "agv_robot_perception/base/perception_capability_interface.hpp"

namespace agv_robot {
namespace task_manager {

/**
 * @brief 任务上下文
 *
 * 任务执行时访问资源和参数的上下文
 */
class TaskContext {
public:
    TaskContext();
    ~TaskContext() = default;

    // ==================== 能力接口 ====================

    void setArmCapability(const capability::IArmCapabilityPtr& arm) {
        arm_ = arm;
    }

    void setAGVCapability(const capability::IAGVCapabilityPtr& agv) {
        agv_ = agv;
    }

    void setVisionCapability(const capability::IVisionCapabilityPtr& vision) {
        vision_ = vision;
    }

    void setUltrasoundCapability(const capability::IUltrasoundCapabilityPtr& ultrasound) {
        ultrasound_ = ultrasound;
    }

    void setPerceptionCapability(const perception::IPerceptionCapabilityPtr& perception) {
        perception_ = perception;
    }

    capability::IArmCapabilityPtr getArmCapability() const { return arm_.lock(); }
    capability::IAGVCapabilityPtr getAGVCapability() const { return agv_.lock(); }
    capability::IVisionCapabilityPtr getVisionCapability() const { return vision_.lock(); }
    capability::IUltrasoundCapabilityPtr getUltrasoundCapability() const { return ultrasound_.lock(); }
    perception::IPerceptionCapabilityPtr getPerceptionCapability() const { return perception_.lock(); }

    // ==================== 参数存储 ====================

    /**
     * @brief 设置参数
     * @param key 键
     * @param value 值
     */
    template<typename T>
    void setParam(const std::string& key, const T& value) {
        params_[key] = value;
    }

    /**
     * @brief 获取参数
     * @param key 键
     * @return 值
     */
    template<typename T>
    std::optional<T> getParam(const std::string& key) const {
        auto it = params_.find(key);
        if (it != params_.end()) {
            try {
                return std::any_cast<T>(it->second);
            } catch (...) {
                return std::nullopt;
            }
        }
        return std::nullopt;
    }

    /**
     * @brief 获取参数（带默认值）
     * @param key 键
     * @param default_value 默认值
     * @return 值
     */
    template<typename T>
    T getParamOr(const std::string& key, const T& default_value) const {
        auto value = getParam<T>(key);
        return value.value_or(default_value);
    }

    /**
     * @brief 检查参数是否存在
     * @param key 键
     * @return 是否存在
     */
    bool hasParam(const std::string& key) const {
        return params_.find(key) != params_.end();
    }

    /**
     * @brief 清除参数
     * @param key 键
     */
    void clearParam(const std::string& key) {
        params_.erase(key);
    }

    /**
     * @brief 清除所有参数
     */
    void clearAllParams() {
        params_.clear();
    }

    // ==================== 共享数据（任务间）====================

    /**
     * @brief 设置共享数据
     * @param key 键
     * @param value 值
     */
    template<typename T>
    void setSharedData(const std::string& key, const T& value) {
        shared_data_[key] = value;
    }

    /**
     * @brief 获取共享数据
     * @param key 键
     * @return 值
     */
    template<typename T>
    std::optional<T> getSharedData(const std::string& key) const {
        auto it = shared_data_.find(key);
        if (it != shared_data_.end()) {
            try {
                return std::any_cast<T>(it->second);
            } catch (...) {
                return std::nullopt;
            }
        }
        return std::nullopt;
    }

    /**
     * @brief 清除共享数据
     * @param key 键
     */
    void clearSharedData(const std::string& key) {
        shared_data_.erase(key);
    }

    // ==================== 任务元数据 ====================

    void setTaskId(const std::string& id) { task_id_ = id; }
    std::string getTaskId() const { return task_id_; }

    void setTaskName(const std::string& name) { task_name_ = name; }
    std::string getTaskName() const { return task_name_; }

    void setTimeout(double seconds) { timeout_seconds_ = seconds; }
    double getTimeout() const { return timeout_seconds_; }

private:
    // 能力接口（弱引用，避免循环依赖）
    std::weak_ptr<capability::IArmCapability> arm_;
    std::weak_ptr<capability::IAGVCapability> agv_;
    std::weak_ptr<capability::IVisionCapability> vision_;
    std::weak_ptr<capability::IUltrasoundCapability> ultrasound_;
    std::weak_ptr<perception::IPerceptionCapability> perception_;

    // 参数存储
    std::unordered_map<std::string, std::any> params_;

    // 共享数据
    std::unordered_map<std::string, std::any> shared_data_;

    // 任务元数据
    std::string task_id_;
    std::string task_name_;
    double timeout_seconds_{60.0};
};

/**
 * @brief 任务上下文智能指针
 */
using TaskContextPtr = std::shared_ptr<TaskContext>;

} // namespace task_manager
} // namespace agv_robot

#endif // AGV_ROBOT_TASK_MANAGER_BASE_TASK_CONTEXT_HPP_
