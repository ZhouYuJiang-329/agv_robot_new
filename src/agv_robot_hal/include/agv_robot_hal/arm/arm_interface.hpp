/**
 * @file arm_interface.hpp
 * @brief 机械臂硬件抽象接口
 */

#ifndef AGV_ROBOT_HAL_ARM_INTERFACE_HPP_
#define AGV_ROBOT_HAL_ARM_INTERFACE_HPP_

#include <array>
#include <string>
#include <vector>

namespace agv_robot {
namespace hal {

/**
 * @brief 机械臂接口
 * 定义机械臂的通用操作接口
 */
class ArmInterface {
public:
    virtual ~ArmInterface() = default;

    /**
     * @brief 连接到机器人控制器
     * @return true=连接成功，false=连接失败
     */
    virtual bool connect() = 0;

    /**
     * @brief 断开与机器人控制器的连接
     */
    virtual void disconnect() = 0;

    /**
     * @brief 检查是否已连接
     * @return true=已连接，false=未连接
     */
    virtual bool isConnected() const = 0;

    /**
     * @brief 激活机器人（上电、设置模式等）
     * @return true=激活成功，false=激活失败
     */
    virtual bool activate() = 0;

    /**
     * @brief 停用机器人（停止运动、断电等）
     */
    virtual void deactivate() = 0;

    /**
     * @brief 读取当前关节位置
     * @return 6 个关节的角度数组（弧度）
     */
    virtual std::array<double, 6> getJointPositions() = 0;

    /**
     * @brief 读取当前笛卡尔位置
     * @return 6 个笛卡尔坐标系数据 （x, y, z, rx, ry, rz）
     */
    virtual std::array<double, 6> getPosturePositions() = 0;

    /**
     * @brief 发送关节空间运动命令
     * @param joints 6 个关节的目标角度（弧度）
     * @return true=命令发送成功，false=发送失败
     */
    virtual bool sendJointCommand(const std::array<double, 6>& joints) = 0;

    /**
     * @brief 发送笛卡尔空间运动命令
     * @param points 路径点数组，每个点包含 {x, y, z, rx, ry, rz}
     * @param move_type 运动类型："joint" 或 "linear"
     * @param speed 运动速度（mm/s）
     * @return true=命令发送成功，false=发送失败
     */
    virtual bool sendCartesianCommand(const std::vector<std::array<double, 6>>& points,
                                      const std::string& move_type, double speed) = 0;

    /**
     * @brief 等待运动完成
     * @param timeout_ms 超时时间（毫秒）
     * @return true=运动完成，false=超时或出错
     */
    virtual bool waitForMotionComplete(int timeout_ms = 10000) = 0;

    /**
     * @brief 检查是否有运动正在执行
     * @return true=有运动正在执行，false=空闲
     */
    virtual bool isMotionExecuting() const = 0;

    /**
     * @brief 获取当前运动命令 ID
     */
    virtual const std::string& getCurrentCommandId() const = 0;
};

} // namespace hal
} // namespace agv_robot

#endif // AGV_ROBOT_HAL_ARM_INTERFACE_HPP_
