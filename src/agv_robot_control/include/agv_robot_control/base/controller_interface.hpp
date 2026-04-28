/**
 * @file controller_interface.hpp
 * @brief 控制器基础接口
 * 
 * 设计原则：
 * 1. 控制器接口与ROS解耦，便于单元测试
 * 2. 生命周期管理统一化
 * 3. 支持组合而非继承
 */

#ifndef AGV_ROBOT_CONTROL_BASE_CONTROLLER_INTERFACE_HPP_
#define AGV_ROBOT_CONTROL_BASE_CONTROLLER_INTERFACE_HPP_

#include <string>
#include <memory>

namespace agv_robot {
namespace control {

/**
 * @brief 控制器状态
 */
enum class ControllerState {
    UNINITIALIZED,  // 未初始化
    INITIALIZED,    // 已初始化
    RUNNING,        // 运行中
    PAUSED,         // 已暂停
    ERROR,          // 错误状态
    STOPPED         // 已停止
};

/**
 * @brief 控制器状态信息
 */
struct ControllerStatus {
    ControllerState state{ControllerState::UNINITIALIZED};
    std::string error_message;
    double cycle_time_ms{0.0};  // 控制周期
    uint64_t cycle_count{0};     // 控制周期计数
    
    bool isRunning() const { 
        return state == ControllerState::RUNNING; 
    }
    bool hasError() const { 
        return state == ControllerState::ERROR; 
    }
};

/**
 * @brief 控制器基础接口
 * 
 * 所有控制器的抽象基类，定义统一的生命周期管理
 * 注意：此接口不依赖ROS，可在纯C++环境中测试
 */
class IController {
public:
    virtual ~IController() = default;

    // ==================== 生命周期管理 ====================
    
    /**
     * @brief 初始化控制器
     * @param config 配置参数（JSON或结构化配置）
     * @return 是否成功
     */
    virtual bool initialize(const std::string& config = "") = 0;
    
    /**
     * @brief 启动控制器
     * @return 是否成功
     */
    virtual bool start() = 0;
    
    /**
     * @brief 停止控制器
     * @return 是否成功
     */
    virtual bool stop() = 0;
    
    /**
     * @brief 暂停控制
     * @return 是否成功
     */
    virtual bool pause() = 0;
    
    /**
     * @brief 恢复控制
     * @return 是否成功
     */
    virtual bool resume() = 0;
    
    /**
     * @brief 紧急停止
     * @return 是否成功
     */
    virtual bool emergencyStop() = 0;
    
    /**
     * @brief 重置控制器
     * @return 是否成功
     */
    virtual bool reset() = 0;
    
    // ==================== 状态查询 ====================
    
    /**
     * @brief 获取控制器状态
     * @return 状态信息
     */
    virtual ControllerStatus getStatus() const = 0;
    
    /**
     * @brief 是否正在运行
     * @return 是否运行中
     */
    virtual bool isRunning() const { 
        return getStatus().isRunning(); 
    }
    
    /**
     * @brief 获取控制器名称
     * @return 名称
     */
    virtual std::string getName() const = 0;
    
    // ==================== 控制周期 ====================
    
    /**
     * @brief 执行一个控制周期
     * 
     * 由上层调用，以固定频率执行控制逻辑
     * @param dt 时间间隔（秒）
     * @return 是否成功
     */
    virtual bool update(double dt) = 0;
    
    /**
     * @brief 获取控制周期（秒）
     * @return 控制周期
     */
    virtual double getControlPeriod() const = 0;
};

/**
 * @brief 控制器接口智能指针类型
 */
using IControllerPtr = std::shared_ptr<IController>;

} // namespace control
} // namespace agv_robot

#endif // AGV_ROBOT_CONTROL_BASE_CONTROLLER_INTERFACE_HPP_
