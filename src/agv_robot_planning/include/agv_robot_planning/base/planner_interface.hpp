/**
 * @file planner_interface.hpp
 * @brief 规划器基础接口
 * 
 * 设计原则：
 * 1. 规划器接口与ROS解耦，便于单元测试和算法替换
 * 2. 支持多种规划算法（RRT, A*, OMPL, CHOMP等）
 * 3. 支持仿真和真实环境切换
 */

#ifndef AGV_ROBOT_PLANNING_BASE_PLANNER_INTERFACE_HPP_
#define AGV_ROBOT_PLANNING_BASE_PLANNER_INTERFACE_HPP_

#include <string>
#include <memory>
#include <optional>

namespace agv_robot {
namespace planning {

/**
 * @brief 规划器状态
 */
enum class PlannerState {
    UNINITIALIZED,  // 未初始化
    INITIALIZING,   // 初始化中
    READY,          // 就绪
    PLANNING,       // 规划中
    ERROR           // 错误
};

/**
 * @brief 规划结果状态
 */
enum class PlanningResultStatus {
    SUCCESS,            // 成功
    TIMEOUT,            // 超时
    NO_SOLUTION,        // 无解
    INVALID_START,      // 无效起始状态
    INVALID_GOAL,       // 无效目标状态
    COLLISION_DETECTED, // 检测到碰撞
    PLANNER_ERROR       // 规划器错误
};

/**
 * @brief 规划统计信息
 */
struct PlanningStatistics {
    double planning_time_ms{0.0};       // 规划耗时
    int iterations{0};                   // 迭代次数
    int vertices_explored{0};            // 探索的顶点数
    double path_length{0.0};             // 路径长度
    double path_cost{0.0};               // 路径代价
    double clearance{0.0};               // 最小间隙
};

/**
 * @brief 规划结果基类
 */
struct PlanningResult {
    PlanningResultStatus status{PlanningResultStatus::PLANNER_ERROR};
    PlanningStatistics statistics;
    std::string error_message;
    
    bool isSuccess() const { 
        return status == PlanningResultStatus::SUCCESS; 
    }
};

/**
 * @brief 规划器基础接口
 * 
 * 所有规划器的抽象基类
 * 注意：此接口不依赖ROS，可在纯C++环境中测试
 */
class IPlanner {
public:
    virtual ~IPlanner() = default;

    // ==================== 生命周期管理 ====================
    
    /**
     * @brief 初始化规划器
     * @param config 配置参数（JSON或配置文件路径）
     * @return 是否成功
     */
    virtual bool initialize(const std::string& config = "") = 0;
    
    /**
     * @brief 终止规划器
     * @return 是否成功
     */
    virtual bool terminate() = 0;
    
    // ==================== 状态查询 ====================
    
    /**
     * @brief 获取规划器状态
     * @return 状态
     */
    virtual PlannerState getState() const = 0;
    
    /**
     * @brief 是否就绪
     * @return 是否就绪
     */
    virtual bool isReady() const { 
        return getState() == PlannerState::READY; 
    }
    
    /**
     * @brief 是否正在规划
     * @return 是否规划中
     */
    virtual bool isPlanning() const { 
        return getState() == PlannerState::PLANNING; 
    }
    
    /**
     * @brief 获取规划器名称
     * @return 名称
     */
    virtual std::string getName() const = 0;
    
    /**
     * @brief 获取规划器类型
     * @return 类型描述
     */
    virtual std::string getType() const = 0;
    
    // ==================== 取消操作 ====================
    
    /**
     * @brief 取消当前规划
     * @return 是否成功
     */
    virtual bool cancelPlanning() = 0;
    
    // ==================== 配置 ====================
    
    /**
     * @brief 设置规划超时时间
     * @param timeout_seconds 超时时间（秒）
     */
    virtual void setPlanningTimeout(double timeout_seconds) = 0;
    
    /**
     * @brief 获取规划超时时间
     * @return 超时时间（秒）
     */
    virtual double getPlanningTimeout() const = 0;
};

/**
 * @brief 规划器接口智能指针
 */
using IPlannerPtr = std::shared_ptr<IPlanner>;

} // namespace planning
} // namespace agv_robot

#endif // AGV_ROBOT_PLANNING_BASE_PLANNER_INTERFACE_HPP_
