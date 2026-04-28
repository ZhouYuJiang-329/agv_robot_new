/**
 * @file capability_interface.hpp
 * @brief 能力层基础接口
 *
 * 设计原则：
 * 1. 能力接口与ROS解耦，便于单元测试
 * 2. 封装Planning和Control层，提供高层次API
 * 3. 支持组合而非继承
 */

#ifndef AGV_ROBOT_CAPABILITY_BASE_CAPABILITY_INTERFACE_HPP_
#define AGV_ROBOT_CAPABILITY_BASE_CAPABILITY_INTERFACE_HPP_

#include <string>
#include <memory>

namespace agv_robot {
namespace capability {

/**
 * @brief 能力状态
 */
enum class CapabilityState {
    UNINITIALIZED,  // 未初始化
    INITIALIZING,   // 初始化中
    READY,          // 就绪
    BUSY,           // 忙碌
    ERROR,          // 错误
    DISABLED        // 禁用
};

/**
 * @brief 能力状态信息
 */
struct CapabilityStatus {
    CapabilityState state{CapabilityState::UNINITIALIZED};
    std::string error_message;
    std::string current_operation;
    double progress{0.0};  // 0.0-1.0

    bool isReady() const {
        return state == CapabilityState::READY;
    }
    bool isBusy() const {
        return state == CapabilityState::BUSY;
    }
    bool hasError() const {
        return state == CapabilityState::ERROR;
    }
};

/**
 * @brief 操作结果
 */
struct OperationResult {
    bool success{false};
    std::string message;
    std::string operation_id;
    double execution_time{0.0};  // 执行时间（秒）

    operator bool() const { return success; }
};

/**
 * @brief 能力基础接口
 *
 * 所有能力类的抽象基类
 * 注意：此接口不依赖ROS，可在纯C++环境中测试
 */
class ICapability {
public:
    virtual ~ICapability() = default;

    // ==================== 生命周期管理 ====================

    /**
     * @brief 初始化能力
     * @param config 配置参数（JSON或配置文件路径）
     * @return 是否成功
     */
    virtual bool initialize(const std::string& config = "") = 0;

    /**
     * @brief 反初始化
     * @return 是否成功
     */
    virtual bool deinitialize() = 0;

    /**
     * @brief 启用能力
     * @return 是否成功
     */
    virtual bool enable() = 0;

    /**
     * @brief 禁用能力
     * @return 是否成功
     */
    virtual bool disable() = 0;

    // ==================== 状态查询 ====================

    /**
     * @brief 获取能力状态
     * @return 状态信息
     */
    virtual CapabilityStatus getStatus() const = 0;

    /**
     * @brief 是否就绪
     * @return 是否就绪
     */
    virtual bool isReady() const {
        return getStatus().isReady();
    }

    /**
     * @brief 是否忙碌
     * @return 是否忙碌
     */
    virtual bool isBusy() const {
        return getStatus().isBusy();
    }

    /**
     * @brief 获取能力名称
     * @return 名称
     */
    virtual std::string getName() const = 0;

    /**
     * @brief 获取能力描述
     * @return 描述
     */
    virtual std::string getDescription() const = 0;

    // ==================== 取消操作 ====================

    /**
     * @brief 取消当前操作
     * @return 是否成功
     */
    virtual bool cancelCurrentOperation() = 0;

    /**
     * @brief 等待当前操作完成
     * @param timeout 超时时间（秒，0表示无限等待）
     * @return 是否成功完成
     */
    virtual bool waitForCompletion(double timeout = 0.0) = 0;

    // ==================== 健康检查 ====================

    /**
     * @brief 执行健康检查
     * @return 健康状态
     */
    virtual bool healthCheck() = 0;

    /**
     * @brief 获取最后错误信息
     * @return 错误信息
     */
    virtual std::string getLastError() const = 0;
};

/**
 * @brief 能力接口智能指针
 */
using ICapabilityPtr = std::shared_ptr<ICapability>;

} // namespace capability
} // namespace agv_robot

#endif // AGV_ROBOT_CAPABILITY_BASE_CAPABILITY_INTERFACE_HPP_
