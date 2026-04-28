# AGV Robot 完整架构文档

## 1. 整体架构图

```
┌─────────────────────────────────────────────────────────────────────────────────────────┐
│                                    Application Layer                                     │
│                              (用户应用 / 上位机 / Web界面)                                │
└─────────────────────────────────────────────────────────────────────────────────────────┘
                                           │
                                           ▼
┌─────────────────────────────────────────────────────────────────────────────────────────┐
│                                    ROS2 Interface Layer                                  │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────────┐  │
│  │   /task_manager │  │  /capability    │  │  /perception    │  │  /planning_control  │  │
│  │   (Action/Srv)  │  │  (Action/Srv)   │  │  (Topic/Srv)    │  │  (Action/Srv)       │  │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘  └─────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────────────────────┘
                                           │
                                           ▼
┌─────────────────────────────────────────────────────────────────────────────────────────┐
│                                    Task Manager Layer                                    │
│  ┌─────────────────────────────────────────────────────────────────────────────────┐    │
│  │                         TaskManagerV2 (ROS Node)                                 │    │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────┐ │    │
│  │  │ ITaskScheduler│  │ ITaskExecutor│  │ TaskContext │  │ ROS Services/Actions    │ │    │
│  │  │ (调度器)     │  │ (执行器)     │  │ (上下文)    │  │                         │ │    │
│  │  └─────────────┘  └─────────────┘  └─────────────┘  └─────────────────────────┘ │    │
│  └─────────────────────────────────────────────────────────────────────────────────┘    │
│                                           │                                             │
│  ┌────────────────────────────────────────┼─────────────────────────────────────────┐   │
│  │                                        ▼                                         │   │
│  │  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────┐  │   │
│  │  │   ITask         │◄─┤  AtomicTask     │  │ CompositeTask   │  │ 具体任务类   │  │   │
│  │  │   (任务接口)    │  │ (原子任务基类)  │◄─┤ (组合任务基类)  │◄─┤             │  │   │
│  │  └─────────────────┘  └─────────────────┘  └─────────────────┘  └─────────────┘  │   │
│  │                                                                    │             │   │
│  │                                                                    ▼             │   │
│  │                                                       ┌─────────────────────┐    │   │
│  │                                                       │ NavigationTask      │    │   │
│  │                                                       │ PickPlaceTask       │    │   │
│  │                                                       │ DetectAndPickTask   │    │   │
│  │                                                       │ PatrolTask          │    │   │
│  │                                                       └─────────────────────┘    │   │
│  └──────────────────────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────────────────┘
                                           │
                                           ▼
┌─────────────────────────────────────────────────────────────────────────────────────────┐
│                                    Capability Layer                                      │
│  ┌─────────────────────────────────────────────────────────────────────────────────┐    │
│  │                         CapabilityNode (ROS Node)                                │    │
│  │  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────┐ │    │
│  │  │ IArmCapability  │  │ IAGVCapability  │  │IVisionCapability│  │IUltrasound..│ │    │
│  │  │  (机械臂能力)   │  │  (AGV能力)      │  │  (视觉能力)     │  │ (超声能力)  │ │    │
│  │  └────────┬────────┘  └────────┬────────┘  └────────┬────────┘  └──────┬──────┘ │    │
│  │           │                    │                    │                   │        │    │
│  │           ▼                    ▼                    ▼                   ▼        │    │
│  │  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────┐ │    │
│  │  │  ArmCapability  │  │  AGVCapability  │  │ VisionCapability│  │UltrasoundCap│ │    │
│  │  │  (实现类)       │  │  (实现类)       │  │  (实现类)       │  │ (实现类)    │ │    │
│  │  └────────┬────────┘  └────────┬────────┘  └────────┬────────┘  └──────┬──────┘ │    │
│  └───────────┼────────────────────┼────────────────────┼──────────────────┼────────┘    │
└──────────────┼────────────────────┼────────────────────┼──────────────────┼─────────────┘
               │                    │                    │                  │
               ▼                    ▼                    ▼                  ▼
┌─────────────────────────────────────────────────────────────────────────────────────────┐
│                                    Perception Layer                                      │
│  ┌─────────────────────────────────────────────────────────────────────────────────┐    │
│  │                        PerceptionNode (ROS Node)                                 │    │
│  │  ┌─────────────────────────────────────────────────────────────────────────────┐ │    │
│  │  │                    IPerceptionCapability (统一感知接口)                      │ │    │
│  │  └─────────────────────────┬───────────────────────────────────────────────────┘ │    │
│  │                            │                                                    │    │
│  │           ┌────────────────┴────────────────┐                                   │    │
│  │           ▼                                 ▼                                   │    │
│  │  ┌─────────────────────┐      ┌─────────────────────┐                          │    │
│  │  │  VisionCapability   │      │ UltrasoundCapability│                          │    │
│  │  │  (视觉能力实现)      │      │  (超声能力实现)      │                          │    │
│  │  └─────────┬───────────┘      └─────────┬───────────┘                          │    │
│  │            │                            │                                       │    │
│  │  ┌─────────┼─────────┐        ┌─────────┼─────────┐                            │    │
│  │  ▼         ▼         ▼        ▼         ▼         ▼                            │    │
│  │ ICamera IDetector IPose   IUltrasound IObstacle IObstacle ISafety              │    │
│  │         Estimator          Driver     Fusion    Filter    Evaluator            │    │
│  └─────────────────────────────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────────────────────────────┘
               │                    │
               ▼                    ▼
┌─────────────────────────────────────────────────────────────────────────────────────────┐
│                              Planning & Control Layer                                    │
│  ┌──────────────────────────────┐  ┌──────────────────────────────┐                     │
│  │      Planning Layer          │  │       Control Layer          │                     │
│  │  ┌────────────────────────┐  │  │  ┌────────────────────────┐  │                     │
│  │  │    IPlanner (基类)     │  │  │  │   IController (基类)   │  │                     │
│  │  │  ├─ initialize()       │  │  │  │  ├─ initialize()       │  │                     │
│  │  │  ├─ terminate()        │  │  │  │  ├─ start()            │  │                     │
│  │  │  └─ cancelPlanning()   │  │  │  │  ├─ stop()             │  │                     │
│  │  └──────────┬─────────────┘  │  │  └──────────┬─────────────┘  │                     │
│  │             │                │  │             │                │                     │
│  │  ┌──────────┴──────────┐     │  │  ┌──────────┴──────────┐     │                     │
│  │  ▼                     ▼     │  │  ▼                     ▼     │                     │
│  │ IArmPlanner        IAGVPlanner│  │ IArmController    IAGVController│                 │
│  │  ├─ plan()           planPath()│  │  ├─ executeTrajectory() trackPath()│              │
│  │  └─ planToConfig()   planToPose()│ │  └─ stopMotion()    stopMotion()│                │
│  │                                │  │                                │                     │
│  │  ┌────────────────────────┐    │  │  ┌────────────────────────┐    │                     │
│  │  │  ArmPlanningNode       │    │  │  │  ArmControlNode        │    │                     │
│  │  │  AGVPlanningNode       │    │  │  │  AGVControlNode        │    │                     │
│  │  └────────────────────────┘    │  │  └────────────────────────┘    │                     │
│  └──────────────────────────────┘  └──────────────────────────────┘                     │
└─────────────────────────────────────────────────────────────────────────────────────────┘
               │                    │
               ▼                    ▼
┌─────────────────────────────────────────────────────────────────────────────────────────┐
│                                    HAL Layer (硬件抽象层)                                 │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────────┐  │
│  │  IArmInterface  │  │  IAGVInterface  │  │ ICameraInterface│  │IUltrasoundInterface │  │
│  │  (机械臂硬件接口)│  │  (AGV硬件接口)  │  │ (相机硬件接口)  │  │  (超声硬件接口)     │  │
│  └────────┬────────┘  └────────┬────────┘  └────────┬────────┘  └──────────┬──────────┘  │
│           │                    │                    │                      │             │
│           ▼                    ▼                    ▼                      ▼             │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────────┐  │
│  │  EtherCATDriver │  │   CANDriver     │  │  RealSenseDriver│  │  SerialUltrasound   │  │
│  │  (机械臂驱动)   │  │   (AGV驱动)     │  │  (相机驱动)     │  │    (超声驱动)       │  │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘  └─────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────────────────────┘
                                           │
                                           ▼
┌─────────────────────────────────────────────────────────────────────────────────────────┐
│                                    Hardware Layer                                        │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────────┐  │
│  │   机械臂 (UR/   │  │   AGV底盘       │  │   相机 (RealSense│  │   超声传感器阵列    │  │
│  │   Elite/Effort) │  │  (CAN总线控制)  │  │   /ZED/普通USB) │  │   (8-12个传感器)    │  │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘  └─────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────────────────────┘
```

## 2. 分层职责说明

| 层级 | 职责 | 核心接口/类 | 设计模式 |
|------|------|------------|----------|
| **Application** | 用户交互、业务逻辑编排 | - | - |
| **ROS2 Interface** | ROS2通信封装、服务暴露 | Action/Service/Topic | 适配器模式 |
| **Task Manager** | 任务调度、执行管理、生命周期 | ITask, ITaskScheduler, ITaskExecutor | 策略模式、组合模式 |
| **Capability** | 高层次能力抽象、业务API | IArmCapability, IAGVCapability, IVisionCapability | 外观模式 |
| **Perception** | 感知数据处理、传感器融合 | IPerceptionCapability, IDetector, IPoseEstimator | 策略模式、责任链 |
| **Planning** | 运动规划、路径规划 | IPlanner, IArmPlanner, IAGVPlanner | 策略模式 |
| **Control** | 运动控制、轨迹跟踪 | IController, IArmController, IAGVController | 策略模式 |
| **HAL** | 硬件抽象、设备驱动接口 | IArmInterface, IAGVInterface, ICameraInterface | 桥接模式 |
| **Hardware** | 实际硬件设备 | - | - |

## 3. 完整文件目录树

```
agv_robot_new/
├── src/
│   ├── agv_robot_common/                    # 公共库
│   │   └── include/agv_robot_common/
│   │       ├── types.hpp                    # 公共数据类型
│   │       ├── utils.hpp                    # 工具函数
│   │       └── exceptions.hpp               # 异常定义
│   │
│   ├── agv_robot_msgs/                      # ROS消息定义
│   │   ├── msg/
│   │   │   ├── JointState.msg
│   │   │   ├── CartesianPose.msg
│   │   │   ├── Detection.msg
│   │   │   ├── DetectionArray.msg
│   │   │   ├── Obstacle.msg
│   │   │   ├── ObstacleArray.msg
│   │   │   ├── Path.msg
│   │   │   ├── Trajectory.msg
│   │   │   ├── TaskStatus.msg
│   │   │   └── PerceptionStatus.msg
│   │   ├── srv/
│   │   │   ├── MoveArm.srv
│   │   │   ├── NavigateToPose.srv
│   │   │   ├── DetectObject.srv
│   │   │   ├── EstimatePose.srv
│   │   │   ├── GetObstacles.srv
│   │   │   ├── CheckSafety.srv
│   │   │   ├── SubmitTask.srv
│   │   │   ├── CancelTask.srv
│   │   │   └── GetTaskStatus.srv
│   │   └── action/
│   │       ├── PickPlace.action
│   │       ├── Patrol.action
│   │       ├── MoveArm.action
│   │       └── Navigate.action
│   │
│   ├── agv_robot_hal/                       # 硬件抽象层
│   │   └── include/agv_robot_hal/
│   │       ├── base/
│   │       │   └── hardware_interface.hpp   # IHardwareInterface
│   │       ├── arm/
│   │       │   └── arm_interface.hpp        # IArmInterface
│   │       ├── agv/
│   │       │   └── agv_interface.hpp        # IAGVInterface
│   │       ├── camera/
│   │       │   └── camera_interface.hpp     # ICameraInterface
│   │       └── ultrasound/
│   │           └── ultrasound_interface.hpp # IUltrasoundInterface
│   │
│   ├── agv_robot_drivers/                   # 设备驱动层
│   │   └── include/agv_robot_drivers/
│   │       ├── arm/
│   │       │   └── ethercat_driver.hpp      # EtherCATDriver
│   │       └── agv/
│   │           └── can_driver.hpp           # CANDriver
│   │
│   ├── agv_robot_control/                   # 控制层
│   │   └── include/agv_robot_control/
│   │       ├── base/
│   │       │   ├── controller_interface.hpp       # IController
│   │       │   ├── arm_controller_interface.hpp   # IArmController
│   │       │   └── agv_controller_interface.hpp   # IAGVController
│   │       ├── arm/
│   │       │   └── arm_controller.hpp             # ArmController
│   │       ├── agv/
│   │       │   └── agv_controller.hpp             # AGVController
│   │       └── nodes/
│   │           ├── arm_control_node.hpp           # ArmControlNode
│   │           └── agv_control_node.hpp           # AGVControlNode
│   │
│   ├── agv_robot_planning/                  # 规划层
│   │   └── include/agv_robot_planning/
│   │       ├── base/
│   │       │   ├── planner_interface.hpp          # IPlanner
│   │       │   ├── arm_planner_interface.hpp      # IArmPlanner
│   │       │   └── agv_planner_interface.hpp      # IAGVPlanner
│   │       ├── arm/
│   │       │   └── arm_motion_planner.hpp         # ArmMotionPlanner
│   │       ├── agv/
│   │       │   └── agv_planner.hpp                # AGVPlanner
│   │       └── nodes/
│   │           ├── arm_planning_node.hpp          # ArmPlanningNode
│   │           └── agv_planning_node.hpp          # AGVPlanningNode
│   │
│   ├── agv_robot_perception/                # 感知层
│   │   └── include/agv_robot_perception/
│   │       ├── base/
│   │       │   └── perception_capability_interface.hpp  # IPerceptionCapability
│   │       ├── vision/
│   │       │   ├── camera_interface.hpp           # ICamera
│   │       │   ├── detector_interface.hpp         # IDetector
│   │       │   ├── pose_estimator_interface.hpp   # IPoseEstimator
│   │       │   ├── vision_capability.hpp          # VisionCapability
│   │       │   └── vision_processor.hpp           # VisionProcessor (旧)
│   │       ├── ultrasound/
│   │       │   ├── ultrasound_driver_interface.hpp    # IUltrasoundDriver
│   │       │   ├── obstacle_provider_interface.hpp    # IObstacleProvider
│   │       │   ├── obstacle_fusion_interface.hpp      # IObstacleFusion
│   │       │   ├── obstacle_filter_interface.hpp      # IObstacleFilter
│   │       │   ├── safety_evaluator_interface.hpp     # ISafetyEvaluator
│   │       │   ├── ultrasound_capability.hpp          # UltrasoundCapability
│   │       │   └── ultrasound_fusion.hpp              # UltrasoundFusion (旧)
│   │       └── nodes/
│   │           └── perception_node.hpp          # PerceptionNode
│   │
│   ├── agv_robot_capability/                # 能力层
│   │   └── include/agv_robot_capability/
│   │       ├── base/
│   │       │   ├── capability_interface.hpp         # ICapability
│   │       │   ├── arm_capability_interface.hpp     # IArmCapability
│   │       │   ├── agv_capability_interface.hpp     # IAGVCapability
│   │       │   ├── vision_capability_interface.hpp  # IVisionCapability
│   │       │   └── ultrasound_capability_interface.hpp  # IUltrasoundCapability
│   │       ├── arm/
│   │       │   └── arm_capability.hpp               # ArmCapability
│   │       ├── agv/
│   │       │   └── agv_capability.hpp               # AGVCapability
│   │       ├── vision/
│   │       │   └── vision_capability.hpp            # VisionCapability (旧)
│   │       ├── ultrasound/
│   │       │   └── ultrasound_capability.hpp        # UltrasoundCapability (旧)
│   │       └── nodes/
│   │           └── capability_node.hpp              # CapabilityNode
│   │
│   └── agv_robot_task_manager/              # 任务管理层
│       └── include/agv_robot_task_manager/
│           ├── base/
│           │   ├── task_interface.hpp           # ITask
│           │   ├── task_context.hpp             # TaskContext
│           │   ├── task_executor_interface.hpp  # ITaskExecutor
│           │   └── task_scheduler_interface.hpp # ITaskScheduler
│           ├── tasks/
│           │   ├── atomic_task.hpp              # AtomicTask
│           │   ├── composite_task.hpp           # CompositeTask
│           │   ├── navigation_task.hpp          # NavigationTask
│           │   ├── pick_place_task.hpp          # PickPlaceTask
│           │   └── detect_and_pick_task.hpp     # DetectAndPickTask
│           └── task_manager/
│               ├── task_manager.hpp             # TaskManager (旧)
│               └── task_manager_v2.hpp          # TaskManagerV2
│
├── agv_robot_bringup/                       # 启动配置
│   ├── launch/
│   │   ├── robot.launch.py                  # 完整启动
│   │   ├── simulation.launch.py             # 仿真启动
│   │   └── hardware.launch.py               # 硬件启动
│   └── config/
│       ├── robot_params.yaml                # 机器人参数
│       ├── controllers.yaml                 # 控制器配置
│       └── planners.yaml                    # 规划器配置
│
└── ARCHITECTURE.md                          # 本文档
```

## 4. 完整继承关系图

### 4.1 Task Manager 继承关系

```
                        ┌─────────────────┐
                        │    ITask        │◄────────────────────────────┐
                        │   (接口)        │                             │
                        └────────┬────────┘                             │
                                 │                                      │
              ┌──────────────────┼──────────────────┐                   │
              │                  │                  │                   │
              ▼                  ▼                  ▼                   │
    ┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐        │
    │   AtomicTask    │ │ CompositeTask   │ │  其他ITask实现   │        │
    │   (抽象基类)    │ │  (抽象基类)     │ │                │        │
    └────────┬────────┘ └────────┬────────┘ └─────────────────┘        │
             │                   │                                      │
    ┌────────┴────────┐          │                                      │
    ▼                 ▼          ▼                                      │
┌──────────┐  ┌──────────────┐ ┌──────────────────┐                     │
│Navigation│  │ MoveArmTask  │ │  PickPlaceTask   │                     │
│  Task    │  │              │ │  (Composite)     │                     │
└──────────┘  └──────────────┘ └──────────────────┘                     │
                               │                                       │
                               ▼                                       │
                    ┌─────────────────────┐                            │
                    │ DetectAndPickTask   │                            │
                    │    (Composite)      │                            │
                    └─────────────────────┘                            │
                                                                        │
┌───────────────────────────────────────────────────────────────────────┘
│
│   ITask 核心接口:
│   ├─ initialize(context) -> bool
│   ├─ execute() -> TaskResult
│   ├─ cancel() -> bool
│   ├─ pause() -> bool
│   ├─ resume() -> bool
│   ├─ getState() -> TaskState
│   ├─ getProgress() -> TaskProgress
│   ├─ hasChildren() -> bool
│   └─ addChild(task)
│
```

### 4.2 Capability 继承关系

```
                        ┌─────────────────┐
                        │   ICapability   │◄────────────────────────────┐
                        │   (基础接口)    │                             │
                        └────────┬────────┘                             │
                                 │                                      │
        ┌────────────────────────┼────────────────────────┐             │
        │                        │                        │             │
        ▼                        ▼                        ▼             │
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐      │
│ IArmCapability  │    │ IAGVCapability  │    │IVisionCapability│      │
│  (机械臂能力)   │    │  (AGV能力)      │    │  (视觉能力)     │      │
└────────┬────────┘    └────────┬────────┘    └────────┬────────┘      │
         │                      │                      │               │
         ▼                      ▼                      ▼               │
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐      │
│  ArmCapability  │    │  AGVCapability  │    │ VisionCapability│      │
│   (实现类)      │    │   (实现类)      │    │  (实现类)       │      │
└─────────────────┘    └─────────────────┘    └─────────────────┘      │
                                                                        │
                        ┌─────────────────┐                            │
                        │IUltrasoundCap.  │                            │
                        │ (超声能力)      │                            │
                        └────────┬────────┘                            │
                                 │                                     │
                                 ▼                                     │
                        ┌─────────────────┐                            │
                        │UltrasoundCap.   │                            │
                        │  (实现类)       │                            │
                        └─────────────────┘                            │
                                                                        │
┌───────────────────────────────────────────────────────────────────────┘
│
│   ICapability 核心接口:
│   ├─ initialize() -> bool
│   ├─ deinitialize() -> bool
│   ├─ enable() -> bool
│   ├─ disable() -> bool
│   ├─ getStatus() -> CapabilityStatus
│   ├─ cancelCurrentOperation() -> bool
│   └─ healthCheck() -> bool
│
```

### 4.3 Perception 继承关系

```
                    ┌─────────────────────────┐
                    │  IPerceptionCapability  │◄────────────────────────┐
                    │    (统一感知接口)       │                         │
                    └───────────┬─────────────┘                         │
                                │                                       │
                ┌───────────────┴───────────────┐                       │
                │                               │                       │
                ▼                               ▼                       │
    ┌─────────────────────┐       ┌─────────────────────┐              │
    │   VisionCapability  │       │ UltrasoundCapability│              │
    │    (视觉能力实现)    │       │  (超声能力实现)      │              │
    └──────────┬──────────┘       └──────────┬──────────┘              │
               │                             │                         │
    ┌──────────┼──────────┐       ┌──────────┼──────────┐              │
    ▼          ▼          ▼       ▼          ▼          ▼              │
 ICamera   IDetector  IPose   IUltrasound IObstacle IObstacle ISafety │
          Estimator            Driver     Fusion    Filter    Evaluator│
                                                                        │
    ┌───────────────────────────────────────────────────────────────────┘
    │
    │   Vision 组件接口:
    │   ├─ ICamera: getColorImage(), getDepthImage()
    │   ├─ IDetector: detect(image) -> vector<Detection>
    │   └─ IPoseEstimator: estimate(detection) -> Pose
    │
    │   Ultrasound 组件接口:
    │   ├─ IUltrasoundDriver: getAllReadings() -> vector<Reading>
    │   ├─ IObstacleFusion: fuse(readings) -> vector<Obstacle>
    │   ├─ IObstacleFilter: filter(obstacles) -> vector<Obstacle>
    │   └─ ISafetyEvaluator: assess(obstacles) -> SafetyAssessment
    │
```

### 4.4 Planning 继承关系

```
                        ┌─────────────────┐
                        │    IPlanner     │◄────────────────────────────┐
                        │   (规划器基类)   │                             │
                        └────────┬────────┘                             │
                                 │                                      │
              ┌──────────────────┼──────────────────┐                   │
              │                  │                  │                   │
              ▼                  ▼                  ▼                   │
    ┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐        │
    │  IArmPlanner    │ │  IAGVPlanner    │ │  其他IPlanner   │        │
    │  (机械臂规划)   │ │  (AGV规划)      │ │                │        │
    └────────┬────────┘ └────────┬────────┘ └─────────────────┘        │
             │                   │                                      │
             ▼                   ▼                                      │
    ┌─────────────────┐ ┌─────────────────┐                            │
    │ ArmMotionPlanner│ │    AGVPlanner   │                            │
    │   (实现类)      │ │    (实现类)     │                            │
    └─────────────────┘ └─────────────────┘                            │
                                                                        │
┌───────────────────────────────────────────────────────────────────────┘
│
│   IPlanner 核心接口:
│   ├─ initialize(config) -> bool
│   ├─ terminate() -> bool
│   ├─ getState() -> PlannerState
│   └─ cancelPlanning() -> bool
│
│   IArmPlanner 扩展:
│   ├─ plan(start, goal) -> optional<Trajectory>
│   ├─ planToConfig(start, goal) -> optional<Trajectory>
│   └─ planCartesianPath(waypoints) -> optional<Trajectory>
│
│   IAGVPlanner 扩展:
│   ├─ planPath(start, goal) -> optional<Path>
│   ├─ planWithObstacles(start, goal, obstacles) -> optional<Path>
│   └─ isPathValid(path) -> bool
│
```

### 4.5 Control 继承关系

```
                        ┌─────────────────┐
                        │   IController   │◄────────────────────────────┐
                        │   (控制器基类)   │                             │
                        └────────┬────────┘                             │
                                 │                                      │
              ┌──────────────────┼──────────────────┐                   │
              │                  │                  │                   │
              ▼                  ▼                  ▼                   │
    ┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐        │
    │ IArmController  │ │ IAGVController  │ │ 其他IController │        │
    │  (机械臂控制)   │ │  (AGV控制)      │ │                │        │
    └────────┬────────┘ └────────┬────────┘ └─────────────────┘        │
             │                   │                                      │
             ▼                   ▼                                      │
    ┌─────────────────┐ ┌─────────────────┐                            │
    │  ArmController  │ │  AGVController  │                            │
    │   (实现类)      │ │   (实现类)      │                            │
    └─────────────────┘ └─────────────────┘                            │
                                                                        │
┌───────────────────────────────────────────────────────────────────────┘
│
│   IController 核心接口:
│   ├─ initialize(config) -> bool
│   ├─ start() -> bool
│   ├─ stop() -> bool
│   ├─ pause() -> bool
│   ├─ resume() -> bool
│   ├─ emergencyStop() -> bool
│   ├─ getStatus() -> ControllerStatus
│   └─ update(dt) -> bool
│
│   IArmController 扩展:
│   ├─ executeTrajectory(traj) -> bool
│   ├─ stopMotion() -> bool
│   └─ setVelocity(vel) -> bool
│
│   IAGVController 扩展:
│   ├─ trackPath(path) -> bool
│   ├─ setVelocity(lin, ang) -> bool
│   └─ stopMotion() -> bool
│
```

### 4.6 HAL 继承关系

```
                        ┌─────────────────────┐
                        │ IHardwareInterface  │◄────────────────────────────────┐
                        │   (硬件接口基类)     │                                 │
                        └──────────┬──────────┘                                 │
                                   │                                            │
        ┌──────────────────────────┼──────────────────────────┐                 │
        │                          │                          │                 │
        ▼                          ▼                          ▼                 │
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐              │
│  IArmInterface  │    │  IAGVInterface  │    │ ICameraInterface│              │
│  (机械臂硬件)   │    │  (AGV硬件)      │    │  (相机硬件)     │              │
└────────┬────────┘    └────────┬────────┘    └────────┬────────┘              │
         │                      │                      │                       │
         ▼                      ▼                      ▼                       │
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐              │
│ EtherCATDriver  │    │   CANDriver     │    │ RealSenseDriver │              │
│   (实现类)      │    │   (实现类)      │    │   (实现类)      │              │
└─────────────────┘    └─────────────────┘    └─────────────────┘              │
                                                                                │
                        ┌─────────────────────┐                                │
                        │IUltrasoundInterface │                                │
                        │   (超声硬件接口)     │                                │
                        └──────────┬──────────┘                                │
                                   │                                           │
                                   ▼                                           │
                        ┌─────────────────────┐                                │
                        │ SerialUltrasound    │                                │
                        │    (实现类)          │                                │
                        └─────────────────────┘                                │
                                                                                │
┌───────────────────────────────────────────────────────────────────────────────┘
│
│   IHardwareInterface 核心接口:
│   ├─ initialize() -> bool
│   ├─ shutdown() -> bool
│   ├─ isConnected() -> bool
│   └─ getStatus() -> HardwareStatus
│
```

## 5. ROS2 节点关联图

```
┌─────────────────────────────────────────────────────────────────────────────────────────┐
│                              ROS2 Nodes Relationship                                     │
└─────────────────────────────────────────────────────────────────────────────────────────┘

  ┌─────────────────────┐
  │   /task_manager     │◄────────────────── Application
  │   (TaskManagerV2)   │                      (ROS2 Actions/Services)
  └──────────┬──────────┘
             │
             │ uses
             ▼
  ┌─────────────────────┐     ┌─────────────────────┐     ┌─────────────────────┐
  │   /capability_node  │◄───►│  /perception_node   │     │   /planning_nodes   │
  │   (CapabilityNode)  │     │  (PerceptionNode)   │     │  (Planning Nodes)   │
  └──────────┬──────────┘     └──────────┬──────────┘     └──────────┬──────────┘
             │                           │                           │
             │ calls                     │ provides                  │ plans
             ▼                           ▼                           ▼
  ┌─────────────────────┐     ┌─────────────────────┐     ┌─────────────────────┐
  │   /control_nodes    │     │   Sensor Topics     │     │   /control_nodes    │
  │  (Control Nodes)    │     │  /camera/image_raw  │     │  (Control Nodes)    │
  └──────────┬──────────┘     │  /ultrasound/dist   │     └──────────┬──────────┘
             │                └─────────────────────┘                │
             │ controls                                            │ controls
             ▼                                                      ▼
  ┌─────────────────────┐                                ┌─────────────────────┐
  │   Hardware Drivers  │                                │   Hardware Drivers  │
  │  (EtherCAT/CAN)     │                                │  (EtherCAT/CAN)     │
  └─────────────────────┘                                └─────────────────────┘


┌─────────────────────────────────────────────────────────────────────────────────────────┐
│                              ROS2 Topics & Services                                      │
└─────────────────────────────────────────────────────────────────────────────────────────┘

Task Manager Layer:
  Actions:
    - /task_manager/pick_place      (PickPlace.action)
    - /task_manager/patrol          (Patrol.action)
    - /task_manager/move_arm        (MoveArm.action)
  Services:
    - /task_manager/submit_task     (SubmitTask.srv)
    - /task_manager/cancel_task     (CancelTask.srv)
    - /task_manager/get_status      (GetTaskStatus.srv)
  Topics:
    - /task_manager/status          (TaskStatus.msg)

Capability Layer:
  Actions:
    - /capability/move_arm          (MoveArm.action)
    - /capability/navigate          (Navigate.action)
  Services:
    - /capability/pick              (PickObject.srv)
    - /capability/place             (PlaceObject.srv)
    - /capability/open_gripper      (GripperCommand.srv)

Perception Layer:
  Topics (Pub):
    - /perception/detections        (DetectionArray.msg)
    - /perception/obstacles         (ObstacleArray.msg)
    - /perception/status            (PerceptionStatus.msg)
  Services:
    - /perception/detect_object     (DetectObject.srv)
    - /perception/estimate_pose     (EstimatePose.srv)
    - /perception/get_obstacles     (GetObstacles.srv)
    - /perception/check_safety      (CheckSafety.srv)

Planning & Control Layer:
  Actions:
    - /arm_planning/plan_trajectory (PlanTrajectory.action)
    - /agv_planning/plan_path       (PlanPath.action)
    - /arm_control/execute          (ExecuteTrajectory.action)
    - /agv_control/track_path       (TrackPath.action)
  Topics:
    - /arm/joint_states             (JointState.msg)
    - /arm/cartesian_pose           (CartesianPose.msg)
    - /agv/odom                     (Odometry.msg)

Hardware Layer:
  Topics:
    - /camera/color/image_raw       (Image.msg)
    - /camera/depth/image_rect_raw  (Image.msg)
    - /camera/camera_info           (CameraInfo.msg)
    - /ultrasound/distances         (UltrasoundArray.msg)
```

## 6. 数据流向图

```
┌─────────────────────────────────────────────────────────────────────────────────────────┐
│                              Data Flow: MoveL Command                                    │
└─────────────────────────────────────────────────────────────────────────────────────────┘

User Command: arm.moveL(target_pose)

    │
    ▼
┌─────────────────────────────────────────────────────────────────────────────────────────┐
│ 1. Task Manager Layer                                                                    │
│    ┌─────────────────┐                                                                   │
│    │ TaskManagerV2   │  receives command                                                 │
│    │                 │  creates MoveArmTask (AtomicTask)                                 │
│    │ ITaskScheduler  │  adds to priority queue                                           │
│    │ ITaskExecutor   │  executes task                                                    │
│    └────────┬────────┘                                                                   │
└─────────────┼────────────────────────────────────────────────────────────────────────────┘
              │
              ▼
┌─────────────────────────────────────────────────────────────────────────────────────────┐
│ 2. Capability Layer                                                                      │
│    ┌─────────────────┐                                                                   │
│    │ IArmCapability  │  task calls moveL()                                               │
│    │                 │                                                                   │
│    │ ArmCapability   │  ┌─ validates parameters                                          │
│    │   (impl)        │  ├─ checks arm status                                             │
│    │                 │  └─ calls IArmPlanner::plan()                                     │
│    └────────┬────────┘                                                                   │
└─────────────┼────────────────────────────────────────────────────────────────────────────┘
              │
              ▼
┌─────────────────────────────────────────────────────────────────────────────────────────┐
│ 3. Planning Layer                                                                        │
│    ┌─────────────────┐                                                                   │
│    │  IArmPlanner    │  plans trajectory from current to target                          │
│    │                 │  ┌─ calculates IK for target                                      │
│    │ ArmMotionPlanner│  ├─ generates smooth trajectory                                   │
│    │   (impl)        │  └─ validates path (collision check)                              │
│    │                 │  returns Trajectory                                               │
│    └────────┬────────┘                                                                   │
└─────────────┼────────────────────────────────────────────────────────────────────────────┘
              │
              ▼
┌─────────────────────────────────────────────────────────────────────────────────────────┐
│ 4. Control Layer                                                                         │
│    ┌─────────────────┐                                                                   │
│    │ IArmController  │  receives trajectory                                              │
│    │                 │  ┌─ interpolates trajectory (position/velocity/effort)            │
│    │ ArmController   │  ├─ computes control commands (PID/Feedforward)                   │
│    │   (impl)        │  └─ sends to hardware interface                                   │
│    │                 │                                                                   │
│    └────────┬────────┘                                                                   │
└─────────────┼────────────────────────────────────────────────────────────────────────────┘
              │
              ▼
┌─────────────────────────────────────────────────────────────────────────────────────────┐
│ 5. HAL Layer                                                                             │
│    ┌─────────────────┐                                                                   │
│    │  IArmInterface  │  receives joint commands                                          │
│    │                 │  ┌─ converts to driver-specific format                             │
│    │ EtherCATDriver  │  ├─ handles communication protocol                                │
│    │   (impl)        │  └─ sends to hardware                                             │
│    │                 │                                                                   │
│    └────────┬────────┘                                                                   │
└─────────────┼────────────────────────────────────────────────────────────────────────────┘
              │
              ▼
┌─────────────────────────────────────────────────────────────────────────────────────────┐
│ 6. Hardware Layer                                                                        │
│    ┌─────────────────┐                                                                   │
│    │   机械臂电机     │  executes motion                                                   │
│    │   (伺服驱动)     │  ├─ position control loop                                         │
│    │                 │  ├─ velocity control loop                                         │
│    │                 │  └─ current/torque control loop                                    │
│    └─────────────────┘                                                                   │
└─────────────────────────────────────────────────────────────────────────────────────────┘
              │
              ▼ (feedback)
┌─────────────────────────────────────────────────────────────────────────────────────────┐
│ Feedback Flow (reverse):                                                                 │
│ Hardware → HAL (joint states) → Control (feedback) → Planning (monitoring)              │
│         → Capability (status) → Task Manager (progress) → User (completion)             │
└─────────────────────────────────────────────────────────────────────────────────────────┘
```

## 7. 设计模式汇总

| 层级 | 设计模式 | 应用场景 |
|------|----------|----------|
| **Task Manager** | 策略模式 | ITaskScheduler 支持多种调度策略 (FIFO/Priority/EDF) |
| | 组合模式 | CompositeTask 组合原子任务形成复杂任务 |
| | 模板方法 | AtomicTask 定义执行流程，子类实现 doExecute() |
| **Capability** | 外观模式 | IArmCapability 封装 Planning + Control 的复杂交互 |
| | 依赖注入 | Capability 通过构造函数接收 Planner 和 Controller |
| **Perception** | 策略模式 | IDetector 支持 YOLO/TensorRT/FakeDetector 切换 |
| | 责任链 | Ultrasound: Driver → Fusion → Filter → Safety |
| | 组合模式 | VisionCapability 组合 Camera + Detector + PoseEstimator |
| **Planning** | 策略模式 | IPlanner 支持不同规划算法 (RRT/A*/OMPL) |
| **Control** | 策略模式 | IController 支持不同控制算法 (PID/MPC/Feedforward) |
| **HAL** | 桥接模式 | IArmInterface 抽象与具体驱动实现分离 |
| **整体** | 依赖倒置 | 高层模块依赖接口，不依赖具体实现 |
| | 单一职责 | 每层只负责一个抽象层次的功能 |
| | 开闭原则 | 扩展新功能通过添加新实现类，不修改现有代码 |

## 8. 关键设计决策

### 8.1 为什么使用 Capability 层？

```
Without Capability Layer:
TaskManager ──► IArmPlanner + IArmController (需要管理两个接口)

With Capability Layer:
TaskManager ──► IArmCapability (统一接口，内部组合 Planner + Controller)

Benefits:
1. 简化 TaskManager 的依赖
2. 封装 Planning + Control 的协调逻辑
3. 提供高层次业务API (pick/place vs plan+execute)
4. 便于测试和mock
```

### 8.2 为什么 Task 使用组合模式？

```
PickPlaceTask (Composite)
├── MoveToPrePickTask (Atomic)
├── DescendTask (Atomic)
├── PickTask (Atomic)
├── AscendTask (Atomic)
├── MoveToPrePlaceTask (Atomic)
├── DescendTask (Atomic)
├── PlaceTask (Atomic)
└── AscendTask (Atomic)

Benefits:
1. 统一处理原子任务和组合任务
2. 支持复杂任务的层次化构建
3. 支持并行执行子任务
4. 便于任务复用和组合
```

### 8.3 为什么 Perception 使用策略模式？

```
VisionCapability
├── ICamera (RealSenseCamera / FakeCamera)
├── IDetector (YoloDetector / TensorRTDetector / FakeDetector)
└── IPoseEstimator (PnPEstimator / DLEstimator)

Benefits:
1. 运行时切换算法实现
2. 仿真/真实环境无缝切换
3. 便于A/B测试和性能对比
4. 支持不同硬件配置
```

## 9. 测试策略

```
┌─────────────────────────────────────────────────────────────────────────────────────────┐
│                              Testing Pyramid                                             │
└─────────────────────────────────────────────────────────────────────────────────────────┘

                    ┌─────────────────┐
                    │   E2E Tests     │  (Full system with hardware)
                    │  (10% of tests) │
                    └────────┬────────┘
                             │
                    ┌────────┴────────┐
                    │  Integration    │  (ROS nodes + simulators)
                    │    Tests        │
                    │  (20% of tests) │
                    └────────┬────────┘
                             │
            ┌────────────────┼────────────────┐
            │                │                │
    ┌───────┴───────┐ ┌──────┴──────┐ ┌──────┴──────┐
    │  Component    │ │  Component  │ │  Component  │
    │    Tests      │ │    Tests    │ │    Tests    │
    │  (40% of tests)│ │  (40%)      │ │  (40%)      │
    └───────┬───────┘ └──────┬──────┘ └──────┬──────┘
            │                │                │
    ┌───────┴───────┐ ┌──────┴──────┐ ┌──────┴──────┐
    │   Unit Tests  │ │   Unit Tests│ │   Unit Tests│
    │  (30% of tests)│ │  (30%)      │ │  (30%)      │
    └───────────────┘ └─────────────┘ └─────────────┘

Testability Features:
1. 所有接口可mock
2. TaskContext 使用弱引用，可注入mock对象
3. FakeDriver 支持无硬件测试
4. CompositeTask 支持部分子任务mock
```

---

**文档版本**: 1.0  
**最后更新**: 2025-04-28  
**作者**: Claude Code
