# IWR6843 人物融合跟随与 Nav2 接入设计

## 1. 文档目的

本文档定义 IWR6843ISK Long Range People Tracking、`rknn_yolov8_ros`、人物目标融合、
`person_following` 和 Nav2 之间的接口及实现过程。目标是让机器人在 RGB-D 视觉确认人物后，
将该人物与雷达轨迹绑定；人物离开相机视场但雷达仍持续提供同一条有效轨迹时，使用雷达实测
位置继续跟随。

本设计不依赖视觉和雷达同时丢失后的长期轨迹猜测。卡尔曼滤波只用于传感器融合、抑制噪声、
补偿消息间隔和估计当前运动状态，不用于在墙角后无观测地推测人物去向。

## 2. 设计边界

### 2.1 本阶段实现

- 解析 TI Long Range People Tracking UART TLV。
- 发布雷达目标位置、速度、加速度、临时轨迹 ID 和质量信息。
- 将视觉和雷达观测按原始测量时间转换到 `odom`。
- 由视觉确认 person，再把视觉人物与雷达轨迹连续匹配并锁定。
- 视觉离开视场后，只在锁定雷达轨迹持续有效时允许雷达接管。
- 输出统一人物位置、速度、跟踪状态、数据来源和有效性。
- 由 `person_following` 计算跟随位姿并通过 Nav2 `NavigateToPose` Action 更新目标。
- 将雷达人物作为局部动态障碍物的设计接口保留，并分阶段接入。
- 根据融合状态、人物距离和跟踪置信度向 Nav2 提供明确的速度限制。

### 2.2 本阶段不实现

- 雷达单独从环境中选择需要跟随的人。
- 将 TI `track_id` 当作人物永久身份。
- 视觉和雷达都失效后，沿速度方向预测数秒并继续导航。
- 在二维 costmap 中实现严格的带时间维度动态障碍物规划。
- 仅依靠普通 IWR6843 人物跟踪提供安全认证级防碰撞。
- 多人交叉场景下的外观 ReID。当前 YOLO 消息没有稳定的人物身份特征。

## 3. 核心设计结论

1. 视觉负责确认目标是 person，并决定“跟随谁”。
2. 雷达负责提供已确认目标的连续轨迹和运动状态。
3. `TRACKING_RADAR` 表示雷达仍在提供真实观测，不是无观测预测。
4. 两种传感器都没有新观测时，只允许很短的消息间隙容忍，不继续更新远期 Nav2 目标。
5. 融合滤波运行在 `odom`，导航目标在发送前转换到 `map`。
6. 动态导航目标和动态障碍物是两条独立数据链。
7. 标准 Nav2 `ObstacleLayer` 不使用目标速度和加速度；速度影响必须通过跟随策略、
   `SpeedLimit` 或专用动态障碍层显式实现。
8. TI People Tracking 通常面向固定雷达，移动平台上的速度定义和跟踪稳定性必须实测。

## 4. 系统架构

```text
RGB + aligned depth
        |
        v
rknn_yolov8_node -------------------- /yolo/detections
                                                |
                                                v
IWR6843 UART -> iwr6843_lrp_node -> /radar/tracks -> person_target_fusion_node
                                                |              |
                                                |              +-> /person_tracking/status
                                                |              +-> /person_tracking/markers
                                                |              +-> /person_tracking/dynamic_tracks
                                                |              |
                                                |              v
                                                +------ /person_tracking/target
                                                               |
                                                               v
                                                     person_following_node
                                                        |              |
                                                        |              +-> /speed_limit
                                                        v
                                                /navigate_to_pose Action
                                                               |
                                                               v
                                                             Nav2
```

建议拆分为以下 ROS 2 包：

| 包 | 节点/插件 | 职责 |
| --- | --- | --- |
| `person_tracking_msgs` | 无 | 定义雷达轨迹、融合人物目标和状态消息 |
| `iwr6843_lrp_ros` | `iwr6843_lrp_node` | 串口、配置、TLV 解析、坐标约定、诊断 |
| `rknn_yolov8_ros` | `rknn_yolov8_node` | 保持现有 RGB-D 同步、YOLO 推理和三维检测 |
| `person_target_fusion` | `person_target_fusion_node` | 时间对齐、TF、关联、锁定、融合和状态机 |
| `person_following` | `person_following_node` | 跟随点、Action 管理、限速和丢失处理 |
| `radar_person_costmap_layer` | `RadarPersonLayer` | 可选的 local costmap 插件，管理动态人物生命周期 |

各包分离的原因是：雷达驱动可独立复用和回放；融合算法不依赖 Nav2；Nav2 故障不会阻塞
传感器接收；代价地图插件可以在跟踪稳定后再引入。

## 5. 消息与话题契约

### 5.1 `RadarTrack.msg`

```text
uint32 track_id

geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration

float32 confidence
uint16 point_count
float32 position_variance
float32 velocity_variance
```

约定：

- `position`、`velocity` 和 `acceleration` 都表达在外层消息 `header.frame_id` 中。
- TI 没有提供的方差字段填 `NaN`，不能用零表示“完全准确”。
- `confidence` 的定义必须在驱动 README 中注明。若 TI 固件字段不是概率，不应伪装成概率。
- `track_id` 只在当前 TI tracker 生命周期内有效。

### 5.2 `RadarTrackArray.msg`

```text
std_msgs/Header header
uint32 frame_number
person_tracking_msgs/RadarTrack[] tracks
```

`header.stamp` 必须表示雷达测量时间，不能简单表示融合节点收到消息的时间。

### 5.3 `FusedPersonTarget.msg`

```text
uint8 SEARCHING=0
uint8 VISION_CONFIRMED=1
uint8 FUSED_LOCKED=2
uint8 TRACKING_VISION=3
uint8 TRACKING_RADAR=4
uint8 SENSOR_GAP=5
uint8 LOST=6

uint8 SOURCE_NONE=0
uint8 SOURCE_VISION=1
uint8 SOURCE_RADAR=2

std_msgs/Header header
uint8 tracking_state
uint8 source_mask

int64 vision_tracking_id
int64 radar_track_id

geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance velocity

float32 confidence
float32 vision_age_sec
float32 radar_age_sec
bool navigation_allowed
```

约定：

- `header.frame_id` 第一版固定为 `odom`。
- `header.stamp` 是滤波状态对应时间，不是最后一条任意传感器消息的时间。
- `navigation_allowed=false` 时，下游必须停止发送新目标，并取消现有跟随目标。
- 当前 YOLO 未启用持久 tracker 时，`vision_tracking_id=-1`。
- `source_mask` 表示本次状态更新使用的数据来源，不代表历史上是否见过该目标。

### 5.4 话题与 QoS

| 话题 | 类型 | 发布者 | QoS 建议 |
| --- | --- | --- | --- |
| `/radar/tracks` | `RadarTrackArray` | 雷达驱动 | Sensor Data，KeepLast 5 |
| `/radar/points` | `PointCloud2` | 雷达驱动，可选 | Sensor Data，KeepLast 2 |
| `/yolo/detections` | 现有 `DetectionArray` | YOLO | 保持当前可靠 QoS |
| `/person_tracking/target` | `FusedPersonTarget` | 融合节点 | Reliable，KeepLast 5 |
| `/person_tracking/status` | `DiagnosticArray` | 融合节点 | Reliable，KeepLast 5 |
| `/person_tracking/markers` | `MarkerArray` | 融合节点 | Reliable，KeepLast 5 |
| `/person_tracking/dynamic_tracks` | `RadarTrackArray` | 融合节点 | Reliable，KeepLast 5 |
| `/person_following/target_pose` | `PoseStamped` | 跟随节点 | Reliable，KeepLast 5 |
| `/speed_limit` | `nav2_msgs/SpeedLimit` | 跟随节点 | Reliable，KeepLast 1 |

`/person_tracking/dynamic_tracks` 包含当前所有可用人物轨迹，同时标记或排除被跟随目标的方式需在
costmap 接入前固定。不要让 costmap 插件反向决定跟随身份。

## 6. 坐标系设计

### 6.1 坐标树

```text
map -> odom -> base_link
                  |
                  +-> camera_link -> camera_color_optical_frame
                  |
                  +-> iwr6843_link
```

`iwr6843_link` 使用 ROS REP-103 约定：

```text
x: 前
y: 左
z: 上
```

TI 固件原生坐标轴可能与 ROS 不同。驱动必须在一个明确位置完成轴交换和符号转换：

- 推荐在 TLV 解析后立即转换为 `iwr6843_link`；
- 或保留 `iwr6843_native`，再通过固定 TF 转换；
- 两种方式只能选择一种，禁止驱动和静态 TF 重复旋转。

### 6.2 为什么融合在 `odom`

- `camera` 和 `radar` 都随机器人运动，直接在传感器坐标系中跟踪会混入机器人自运动。
- `odom` 连续，适合位置差分、速度估计和短时间滤波。
- `map -> odom` 可能在 AMCL 重定位时跳变，不适合保存卡尔曼状态。
- Nav2 仍然接收 `map` 目标；`person_following` 在发送目标前执行 `odom -> map`。

### 6.3 雷达位置与速度转换

设雷达目标位置和相对速度为 `p_r`、`v_r`，表达在 `iwr6843_link`。令：

- `R_br`：`iwr6843_link` 到 `base_link` 的旋转；
- `t_br`：雷达原点在 `base_link` 下的位置；
- `R_ob`、`t_ob`：`base_link` 到 `odom` 的变换；
- `v_b`、`omega_b`：底盘原点线速度和角速度，表达在 `base_link`。

则：

```text
p_b = t_br + R_br * p_r
p_o = t_ob + R_ob * p_b

v_o = R_ob * (v_b + omega_b x p_b + R_br * v_r)
```

只有当 TI `velocity` 确实表示雷达坐标中目标相对位置的时间导数时，才可使用上述速度公式。
第一版应以按时间戳转换后的雷达位置为主，让融合滤波器估计速度；补偿后的 TI 速度作为对照，
通过移动平台实验验证后再正式加入滤波更新。

### 6.4 移动平台限制

TI Long Range People Tracking/GTRACK 通常以固定雷达为使用前提。输出后补偿只能修正已经正确
生成的轨迹，不能修复雷达前端因机器人运动导致的静态杂波、点云或聚类错误。因此雷达接管前必须
完成以下实验：

1. 人物和机器人都静止；
2. 人物静止、机器人直线运动；
3. 人物静止、机器人原地旋转；
4. 人物匀速行走、机器人静止；
5. 人物和机器人同时运动。

如果场景 2、3 中静止人物频繁丢失、产生明显虚假速度或发生 ID 切换，不允许启用雷达导航接管。

## 7. 时间戳与数据缓存

### 7.1 视觉时间

现有 `/yolo/detections.header` 已复制输入 RGB 图像的时间戳和光学坐标系，这是正确的。推理耗时
不能覆盖原始测量时间。融合节点根据 `processing_time_ms` 记录延迟，但仍以图像时间做 TF 和关联。

### 7.2 雷达时间

TI UART 数据通常包含帧号和设备周期，但未必包含可直接映射到 ROS 时钟的绝对时间。驱动第一版：

1. 记录完整 packet 接收完成时的 ROS 时间；
2. 根据 UART 波特率和 packet 字节数估算传输时间；
3. 将估算后的帧测量时刻写入 `header.stamp`；
4. 保留 `frame_number`，检测丢帧、重复帧和重启；
5. 后续如能读取设备时钟，建立设备周期到 ROS 时间的线性映射。

不得在融合节点重新把雷达消息盖成 `now()`。

### 7.3 odometry 缓存

移动平台速度补偿需要雷达测量时刻的底盘状态。融合节点或独立工具应缓存最近 1 至 2 秒 odometry：

- 平移线性插值；
- 姿态四元数 SLERP；
- 线速度和角速度线性插值；
- 雷达时间早于缓存或晚于最新 odom 太多时拒绝该帧，不使用“最新 odom”代替。

### 7.4 乱序处理

第一版卡尔曼滤波不实现 Out-of-Sequence Measurement 回溯更新：

- 事件队列按 `header.stamp` 排序后处理；
- 比当前滤波时间早超过 `max_out_of_order_sec` 的观测直接丢弃并计数；
- 允许很小的乱序窗口，例如 50 ms，用于 UART 和推理延迟差异；
- 不允许时间戳倒退使滤波器使用负 `dt`。

## 8. 雷达驱动节点实现

### 8.1 启动过程

```text
打开 CLI 串口
    -> 发送 stop/config/start 命令
    -> 检查每条命令应答
打开 Data 串口
    -> 启动读取线程
    -> 搜索 magic word
    -> 校验 packet header 和 totalPacketLen
    -> 解析 TLV
    -> 发布完整 RadarTrackArray
```

固件配置、串口设备、波特率和 TLV 协议版本都必须参数化。解析器必须绑定一个明确的 TI
Industrial Toolbox/People Tracking 版本，不能用“兼容所有版本”的隐式结构体强制转换。

### 8.2 线程模型

```text
serial_read_thread
    只读取字节并写入有界环形缓冲区

packet_parse_thread
    等待条件变量、组包、校验、TLV解析、发布

ROS executor
    参数、服务、诊断定时器和生命周期管理
```

要求：

- 不使用 detached thread；
- 停止时设置 `stop_requested`、通知条件变量并 `join()`；
- 串口读取线程不执行 ROS 发布和复杂日志；
- 缓冲区超限时计数并重新寻找 magic word，不能无限增长；
- 单个 TLV 越界、长度不一致或 packet 过大时丢弃整帧；
- 禁止把串口原始结构体直接 reinterpret_cast 后跨平台使用，逐字段解码并检查字节序和对齐。

### 8.3 驱动诊断

至少发布以下诊断值：

- 固件和协议版本；
- 雷达帧率；
- UART 字节率；
- packet 数、丢帧数、解析错误数、重同步次数；
- 当前 target 数；
- 最近一帧 age；
- 环形缓冲区高水位；
- CLI/Data 串口状态。

## 9. 视觉目标选择与雷达关联

### 9.1 当前视觉限制

现有 YOLO 节点能提供：

- person 类别和置信度；
- RGB bbox；
- 与 RGB 对齐的深度；
- 相机光学坐标下三维位置；
- 原始 RGB 时间戳。

当前 `tracking_id=-1` 时，没有视觉持久身份。在单人测试场景可以选择置信度最高且深度有效的人物；
多人场景必须增加人工选择、视觉 tracker 或 ReID，不能继续依靠“每帧置信度最高”。

### 9.2 初始关联

只有 `VISION_CONFIRMED` 状态允许创建雷达绑定。处理步骤：

1. 根据视觉时间戳把人物三维位置转换到 `odom`；
2. 把每个雷达轨迹预测或插值到同一时间，只补偿消息时间差；
3. 执行二维位置门控；
4. 可选执行高度、距离、速度方向和点数门控；
5. 选取代价最小且与第二候选有足够差距的轨迹；
6. 同一 `track_id` 连续匹配 `association_confirm_frames` 后锁定。

推荐使用马氏距离而不是固定欧氏距离：

```text
innovation = z_vision - z_radar
S = P_vision + P_radar
d2 = innovation^T * inverse(S) * innovation
```

二维 99% 卡方门限可从 `d2 < 9.21` 开始。同时设置绝对位置上限，防止协方差异常大时错误匹配：

```yaml
association_max_distance_m: 0.80
association_mahalanobis_gate: 9.21
association_confirm_frames: 3
association_max_gap_sec: 0.30
association_min_margin: 0.25
```

`association_min_margin` 用于要求最佳候选明显优于第二候选。存在两个近似候选时保持
`VISION_CONFIRMED`，不进行雷达锁定。

### 9.3 锁定后关联

锁定后不能只检查 ID，还要检查轨迹连续性：

- `track_id` 与锁定 ID 相同；
- 消息 age 小于门限；
- 位置创新小于门限；
- 速度和加速度无不合理突跳；
- 质量、点数和协方差满足要求；
- 附近没有产生关联歧义的新轨迹。

若 TI tracker 重启或 frame number 回退，立即解除绑定。旧 ID 即使再次出现也视为新轨迹。

## 10. 融合滤波器

### 10.1 状态模型

第一版采用二维恒速度线性卡尔曼滤波：

```text
state X = [x, y, vx, vy]^T

X(k+1) = F(dt) * X(k) + process_noise

F(dt) =
[1 0 dt 0 ]
[0 1 0  dt]
[0 0 1  0 ]
[0 0 0  1 ]
```

过程噪声由行人未知加速度标准差 `sigma_accel` 生成。不要第一版直接把 `ax, ay` 放入状态；
行人加速度和 TI 加速度输出都可能快速波动，容易造成过度预测。

### 10.2 观测模型

视觉只更新位置：

```text
z_vision = [x, y]^T
```

雷达第一阶段只更新位置：

```text
z_radar_position = [x, y]^T
```

完成移动平台速度验证后，雷达再更新位置和速度：

```text
z_radar = [x, y, vx, vy]^T
```

TI GTRACK 已经滤波，融合层的雷达观测噪声不能设得过小，否则会完全复制 GTRACK 输出并产生
二次滤波延迟。所有噪声必须参数化并通过 rosbag 统计调节。

### 10.3 滤波器输出

每次真实观测到达时：

1. 将滤波器预测到观测时间；
2. 进行创新门控；
3. 更新状态和协方差；
4. 将状态短距离预测到当前发布时刻；
5. 发布 `FusedPersonTarget`。

滤波器可以在消息间隔内提供当前位置和速度估计，但不等于允许传感器失效后长期导航。

运动方向：

```text
heading = atan2(vy, vx)
```

仅当：

```text
hypot(vx, vy) >= minimum_heading_speed
```

且速度协方差低于门限时，方向有效。人物接近静止时使用机器人到人物的连线计算跟随方向。

### 10.4 异常观测

满足任一条件则拒绝该次更新：

- 时间戳非法、倒退或 age 超限；
- TF 在测量时刻不可用；
- 坐标包含 NaN/Inf；
- 位置超出系统允许区域；
- innovation 超过门限；
- 速度或加速度超过合理人体上限；
- 雷达 tracker 刚重启；
- `odom` 在对应时刻不可用。

拒绝单帧不等于立刻丢失目标；由状态机根据连续失败次数和消息 age 决定。

## 11. 融合状态机

### 11.1 状态定义

| 状态 | 含义 | `navigation_allowed` |
| --- | --- | --- |
| `SEARCHING` | 没有视觉确认人物 | false |
| `VISION_CONFIRMED` | 视觉人物有效，尚未稳定绑定雷达 | 可配置；默认 vision-only 可用 |
| `FUSED_LOCKED` | 视觉与锁定雷达轨迹同时有效 | true |
| `TRACKING_VISION` | 已确认目标，雷达暂时不可用，视觉仍有效 | true |
| `TRACKING_RADAR` | 视觉离开视场，锁定雷达轨迹仍持续实测 | true，使用较低速度上限 |
| `SENSOR_GAP` | 两者都短暂没有新帧，只容忍通信间隙 | false，不发送新目标 |
| `LOST` | 绑定失效或超时 | false，取消 Nav2 目标 |

### 11.2 转移规则

```text
SEARCHING
  -- 连续视觉确认 --> VISION_CONFIRMED

VISION_CONFIRMED
  -- 雷达连续匹配 --> FUSED_LOCKED
  -- 视觉继续、雷达无匹配 --> TRACKING_VISION
  -- 视觉失效 --> LOST

FUSED_LOCKED
  -- 雷达失效、视觉有效 --> TRACKING_VISION
  -- 视觉失效、锁定雷达持续有效 --> TRACKING_RADAR
  -- 两者短时无新帧 --> SENSOR_GAP
  -- ID跳变/关联歧义/严重创新异常 --> LOST

TRACKING_VISION
  -- 原目标与雷达重新连续匹配 --> FUSED_LOCKED
  -- 视觉失效 --> LOST

TRACKING_RADAR
  -- 视觉重新出现且与锁定雷达匹配 --> FUSED_LOCKED
  -- 雷达短时漏帧 --> SENSOR_GAP
  -- 雷达ID失效、歧义或超时 --> LOST

SENSOR_GAP
  -- 锁定雷达恢复且连续性成立 --> TRACKING_RADAR
  -- 视觉恢复并确认 --> 对应视觉/融合状态
  -- gap超时 --> LOST

LOST
  -- 完成Action取消并等待重新选择 --> SEARCHING
```

### 11.3 雷达接管有效条件

进入并保持 `TRACKING_RADAR` 必须同时满足：

```text
radar_track_id == locked_radar_track_id
radar_age <= radar_timeout_sec
radar_confidence >= radar_min_confidence
position_innovation <= radar_max_innovation
track_jump <= radar_max_track_jump
association_is_unambiguous == true
radar_driver_healthy == true
```

不再使用固定 `0.8 s` 限制有真实雷达观测的接管。可保留一个硬性
`radar_only_identity_lease_sec`，用于多人环境防止长期错误身份；单人受控场景可放宽，最终由测试确定。

`SENSOR_GAP` 的时间从 0.3 至 0.8 秒开始调节。该状态不发送新的外推导航目标，只保持或取消
现有目标，避免把短时通信故障扩展成无观测追踪。

## 12. `person_following` 重构

### 12.1 当前行为

当前节点直接订阅 `/yolo/detections`，选择每帧置信度最高人物，将相机坐标转换到 `map`，执行
一阶低通滤波并发送 `NavigateToPose`。引入融合后，这些职责需要调整。

### 12.2 重构后职责

- 只订阅 `/person_tracking/target`；
- 不再依赖 `rknn_yolov8_ros` 消息；
- 不再选择人物，也不维护雷达 ID；
- 不再对人物位置做第二次一阶低通；
- 检查 `navigation_allowed`、状态、age 和协方差；
- 将 `odom` 人物状态转换到 `map`；
- 计算安全跟随位姿；
- 对 Nav2 Action 的发送、替换、取消进行串行管理；
- 发布调试目标和速度限制。

### 12.3 跟随位姿

人物运动且速度方向可靠时：

```text
direction = normalize([vx, vy])
follow_position = person_position - follow_distance * direction
```

人物低速或速度方向不可靠时：

```text
direction = normalize(person_position - robot_position)
follow_position = person_position - follow_distance * direction
```

目标朝向默认面向人物当前位置。跟随点必须满足：

- 与人物距离不小于 `follow_distance`；
- 与机器人距离不小于最小目标移动门限；
- 位于有效 TF 和 Nav2 工作坐标系；
- 不使用数秒后的预测人物位置；
- 必要的延迟补偿限制在测量 age 范围，例如 0.1 至 0.3 秒。

### 12.4 Nav2 Action 管理

Nav2 `NavigateToPose` 支持新目标替换当前目标，不需要 waypoint。必须避免并发 goal race：

```text
IDLE
  -> SEND_PENDING
  -> ACTIVE
  -> REPLACE_PENDING
  -> ACTIVE

任意有效状态
  -> CANCEL_PENDING
  -> IDLE
```

规则：

- 同一时刻只允许一个 send 或 cancel 请求在途；
- Action 回调不在持有状态 mutex 时调用新的异步操作；
- 目标无效优先级高于待发送的新目标；
- Nav2 server 不可用时保持停止状态并节流日志；
- 新目标移动和时间门限同时满足后才替换；
- 设置最大刷新间隔，避免目标缓慢移动却长期不更新。

建议初始值：

```yaml
follow_distance: 1.20
goal_update_min_distance: 0.30
goal_update_min_interval_sec: 0.50
goal_update_max_interval_sec: 1.50
target_max_age_sec: 0.30
minimum_heading_speed: 0.20
```

最终值必须结合机器人最大速度、制动距离和室内空间调节。

## 13. 动态目标、动态障碍物与速度控制

### 13.1 三者不能混用

| 数据 | 作用 | Nav2 接口 |
| --- | --- | --- |
| 融合人物目标 | 决定机器人要跟到哪里 | `NavigateToPose` Action |
| 动态人物障碍物 | 防止机器人碰撞人物、绕开其他人 | local costmap layer |
| 跟随速度限制 | 根据距离和可信度限制最高速度 | `nav2_msgs/SpeedLimit` |

Lidar `/scan` 加入 costmap 是障碍物观测，不是动态导航目标。把被跟随人物加入 costmap 不能替代
更新 `NavigateToPose`。

### 13.2 标准 Nav2 costmap 的限制

标准 `ObstacleLayer` 或 `VoxelLayer` 只标记当前占据位置，不消费轨迹的 `vx`、`vy`、`ax`、`ay`。
因此：

- 雷达位置可以转换成 `PointCloud2` 后被标记；
- 雷达速度不会“自然”影响规划器和控制器；
- 把未来整条轨迹画进二维 costmap 会形成没有时间维度的永久障碍走廊，通常过度保守；
- 仅发布稀疏人物点且没有可靠 clearing，会留下旧障碍单元。

### 13.3 推荐 costmap 接入方式

第一阶段只在 RViz 发布雷达轨迹和人体 marker，不进入 costmap，先验证跟踪和接管。

第二阶段实现 `RadarPersonLayer`，仅加入 `local_costmap`：

- 订阅当前有效动态人物轨迹；
- 每个 update 周期只绘制未过期轨迹；
- 保存上一周期 bounds，确保旧人物位置被清除；
- 根据人体半径和协方差绘制圆或椭圆；
- 区分被跟随人物和其他人物；
- 被跟随人物使用较小但非零安全半径；
- 其他人物使用正常人体半径和 inflation；
- 轨迹过期后立即从该 layer 清除，不依赖无穷 observation persistence。

不建议第一版把人物写入 `global_costmap`。临时人员会导致全局路径频繁变化和残留。只有在局部控制器
长期无法绕行、确实需要全局重新规划时，再给 global costmap 增加短生命周期数据。

### 13.4 短期速度投影

后续可在 `RadarPersonLayer` 中为其他人物绘制很短的运动胶囊：

```text
segment_start = current_position
segment_end = current_position + velocity * obstacle_projection_time
```

`obstacle_projection_time` 建议不超过 0.3 至 0.5 秒，并随协方差扩大横向范围。这仍不是严格的
时间参数化动态规划，只是局部保守处理。被跟随人物不应使用过长投影，否则会阻塞其后的跟随点。

### 13.5 速度限制

标准 costmap 代价只会间接影响控制器，不保证按人物速度自动限速。`person_following` 应显式发布
`nav2_msgs/msg/SpeedLimit`，并确认 Controller Server 订阅的话题和单位配置。

建议状态策略：

| 状态 | 速度策略 |
| --- | --- |
| `FUSED_LOCKED` | 正常跟随速度上限 |
| `TRACKING_VISION` | 正常或略低上限 |
| `TRACKING_RADAR` | 降低上限，初始可取正常值的 50% 至 70% |
| `SENSOR_GAP` | 不发布新目标，准备停止 |
| `LOST` | 取消目标；由 Nav2/底盘完成受控停车 |

还应根据人物距离设置分段限速和迟滞，避免速度在阈值附近抖动。不要在 Nav2 工作时由
`person_following` 直接发布另一套 `/cmd_vel`。

## 14. 多线程与共享状态

### 14.1 YOLO 节点

保持当前有界队列方案：

```text
RGB/Depth同步回调 -> 容量3的队列 -> 推理线程
```

队列满时丢弃最旧帧，保证延迟而不是完整帧率。同步回调不得执行推理和后处理。

### 14.2 融合节点

```text
vision callback --+
                  +-> bounded event queue -> fusion worker -> publishers
radar callback ---+
odom callback ----> odom time buffer
```

- 回调只校验基本字段并入队；
- 单一 fusion worker 独占卡尔曼状态、锁定 ID 和状态机；
- 队列按时间排序，容量和最大 age 有限；
- odom 缓存用独立 mutex，查询时复制局部数据后释放锁；
- TF 查询、矩阵运算和发布都不在队列 mutex 下执行；
- 条件变量用于等待，禁止空转；
- 退出时通知并 join worker。

建议事件队列容量从 32 开始，并增加以下统计：

- vision/radar 接收数；
- 队列丢弃数；
- 乱序丢弃数；
- TF 失败数；
- 关联成功/失败/歧义数；
- 状态转移次数。

### 14.3 跟随节点

建议使用 `MultiThreadedExecutor` 和两个 callback group：

- 目标订阅和跟随目标计算：Mutually Exclusive；
- Nav2 Action goal response、feedback、result：Mutually Exclusive。

Action 状态使用一个短生命周期 mutex。任何 `async_send_goal()`、`async_cancel_goal()`、TF 查询和
日志都不要在持锁期间执行。

### 14.4 Costmap 插件

订阅回调更新一个带时间戳的 track map；costmap 的 `updateBounds()`/`updateCosts()` 在 costmap
线程中复制快照后绘制。不要让串口、融合或 Action 线程直接修改 costmap master grid。

## 15. 参数组织

建议分为三个配置文件：

```text
iwr6843_lrp_ros/config/iwr6843_lrp.yaml
person_target_fusion/config/person_target_fusion.yaml
person_following/config/person_following.yaml
```

融合参数初始模板：

```yaml
person_target_fusion_node:
  ros__parameters:
    tracking_frame: odom
    base_frame: base_link
    radar_frame: iwr6843_link

    vision_topic: /yolo/detections
    radar_topic: /radar/tracks
    odom_topic: /odom
    target_topic: /person_tracking/target

    person_class_id: 0
    vision_min_confidence: 0.70
    vision_timeout_sec: 0.35
    radar_timeout_sec: 0.25
    sensor_gap_timeout_sec: 0.50

    association_confirm_frames: 3
    association_max_gap_sec: 0.30
    association_max_distance_m: 0.80
    association_mahalanobis_gate: 9.21
    association_min_margin: 0.25

    radar_min_confidence: 0.0
    radar_max_track_jump_m: 1.0
    radar_only_identity_lease_sec: 3.0

    filter_sigma_accel: 1.5
    vision_position_stddev_m: 0.20
    radar_position_stddev_m: 0.25
    radar_velocity_stddev_mps: 0.50
    use_radar_velocity: false

    minimum_heading_speed_mps: 0.20
    max_out_of_order_sec: 0.05
    max_measurement_age_sec: 0.50
```

这些值只是安全起点，不是最终标定值。尤其是雷达方差、速度使用和 identity lease 必须基于实测。

## 16. Phase1 launch 接入

新增参数：

```text
start_radar:=false
start_person_fusion:=false
start_person_following:=false
enable_person_following_navigation:=false
start_radar_costmap_layer:=false
```

依赖关系：

```text
start_person_fusion
  需要 start_yolo
  radar 可选，允许 vision-only 调试

start_person_following
  需要 start_person_fusion

enable_person_following_navigation
  需要 start_person_following
  默认 false
```

雷达没有连接时，phase1 的普通 Nav2 和 YOLO 必须仍能正常启动。任何雷达串口错误不能导致整个
launch 退出。自动导航保持显式开关，避免系统启动后立即向人物移动。

## 17. 分阶段实施计划

### 阶段 A：接口包

实现 `person_tracking_msgs`：

- `RadarTrack.msg`；
- `RadarTrackArray.msg`；
- `FusedPersonTarget.msg`；
- 消息构建和 `ros2 interface show` 验证。

验收：三个消息可以被独立 C++ 测试节点发布和订阅，字段语义写入 README。

### 阶段 B：雷达输入验收（已完成）

- 固定 TI 固件和协议版本；
- 完成 CLI/Data 串口；
- 完成 packet/TLV 解析；
- 发布 `/radar/tracks` 和诊断；
- RViz 显示目标位置、ID 和速度箭头；
- 录制静止和移动机器人 rosbag。

验收：连续运行至少 30 分钟，无缓冲区增长、解析崩溃和明显内存增长；帧率与 TI GUI 基本一致。

### 阶段 C：坐标和移动平台验证

- 标定 `base_link -> iwr6843_link`；
- 验证轴方向、距离和高度；
- 按测量时间转换雷达位置到 `odom`；
- 完成机器人直行和旋转实验；
- 比较位置差分速度、TI 原始速度和自运动补偿速度。

验收：静止人物在机器人运动时的 `odom` 位置和速度误差满足后续跟随要求。未满足时不得进入阶段 E。

### 阶段 D：离线关联与融合

- 使用 rosbag 同步回放 `/yolo/detections`、`/radar/tracks`、`/odom`、`/tf`；
- 实现时间排序、TF、门控、连续确认和锁定；
- 实现恒速度卡尔曼滤波；
- 暂时只发布 marker 和 `FusedPersonTarget`，不连接 Nav2。

验收：单人场景能稳定锁定；视觉离开 FOV 后保持原雷达 ID；ID 跳变时进入 `LOST`。

### 阶段 E：跟随节点重构

- 改为订阅 `FusedPersonTarget`；
- 删除直接选择 YOLO person 和重复低通滤波；
- 在发送时将 `odom` 转换到 `map`；
- 实现 Action 串行状态机；
- 实现状态相关速度限制；
- 先以 `enable_person_following_navigation=false` 验证目标话题。

验收：RViz 中目标连续、丢失后 marker 正确清除；启用导航后目标替换频率受控且取消可靠。

### 阶段 F：雷达动态障碍物

- 首先仅发布调试人体 marker；
- 实现 local costmap `RadarPersonLayer`；
- 验证旧轨迹清除、人体半径和目标人物特殊处理；
- 再评估 0.3 至 0.5 秒速度胶囊和 global costmap 的必要性。

验收：人员离开后无残留代价；目标人物不会使跟随位姿永久不可达；其他人物能触发局部绕行或减速。

### 阶段 G：phase1 集成

- 增加 launch 参数和配置路径；
- 保持所有新功能默认关闭；
- 按雷达、融合、跟随、导航顺序启用；
- 增加诊断和 rosbag 录制 launch 参数。

## 18. 测试矩阵

| 场景 | 预期结果 |
| --- | --- |
| 单人静止，机器人静止 | 视觉雷达稳定匹配，速度接近零 |
| 单人静止，机器人直行 | `odom` 中人物基本静止，不出现机器人速度残留 |
| 单人静止，机器人旋转 | 旋转补偿后位置连续，雷达不频繁换 ID |
| 人物正常行走 | 速度方向与实际一致，目标更新率受控 |
| 人物离开相机 FOV、雷达持续 | 进入 `TRACKING_RADAR`，降低速度继续跟随 |
| 相机恢复 | 与锁定雷达匹配后回到 `FUSED_LOCKED` |
| 雷达短时漏 1 至 2 帧 | 进入/维持短暂 gap，不产生远期预测目标 |
| 雷达 ID 消失后出现新 ID | 不自动接管，进入 `LOST` 或等待视觉重绑 |
| 两人交叉 | 关联歧义时停止雷达接管，不猜测 |
| 人物绕墙角、两传感器都丢失 | 取消跟随，不预测穿墙 |
| TF 缺失或超时 | 拒绝观测，不使用最新 TF 替代历史 TF |
| Nav2 server 重启 | Action 状态恢复到 IDLE，节点不崩溃 |
| 串口损坏 packet | 丢帧并重新同步 magic word，线程继续运行 |
| CPU 高负载 | 有界队列丢旧数据，延迟不持续累积 |

## 19. 需要记录的指标

- 视觉到融合、雷达到融合、融合到 Nav2 的 P50/P95 延迟；
- 视觉和雷达帧率、队列丢弃率；
- 视觉雷达匹配成功率和歧义率；
- 单次雷达接管持续时间；
- 每分钟雷达 ID switch 次数；
- `odom` 目标位置和速度抖动；
- 每秒 Nav2 goal 替换次数；
- 从 `LOST` 到 Action cancel 完成的时间；
- 人物距离小于安全距离的次数；
- costmap 轨迹过期后的清除时间。

这些指标应写入日志或 diagnostics，不能只依赖 RViz 主观判断。

## 20. 安全规则

- 自动人物导航默认关闭，由明确 launch 参数开启。
- `navigation_allowed=false` 必须触发目标取消，不能只停止发布新目标。
- 视觉未确认的雷达 ID 永远不能启动人物跟随。
- 雷达 ID 跳变、多人关联歧义、TF 错误或时间戳异常时采取停止策略。
- `TRACKING_RADAR` 使用较低速度上限。
- Lidar、深度相机和底盘急停仍是主要防碰撞链路，人物跟踪不是安全传感器。
- 任何 costmap 雷达层失效都不能阻塞 lidar costmap 或 Nav2 主流程。
- 调试 marker 必须设置 lifetime 或发送 DELETE，避免 RViz 保留过期目标造成误判。

## 21. 实施前待确认项

开始编码前需要固定以下事实：

1. TI Long Range People Tracking 固件和 Industrial Toolbox 的准确版本；
2. UART Target List、Target Index 和 Point Cloud TLV 的准确二进制定义；
3. TI `velocity`、`acceleration` 的坐标系和移动平台语义；
4. 雷达实际安装位置、俯仰角、轴方向和视场；
5. 雷达串口设备名、波特率和 udev 唯一属性；
6. 底盘 `/odom.twist` 是否按 ROS 约定表达在 `child_frame_id=base_link`；
7. phase1 实际 Controller Server 的 `/speed_limit` 话题和单位配置；
8. 第一阶段目标场景是严格单人还是允许多人；
9. 允许的最大跟随速度、最小距离和制动距离；
10. 是否需要人工选择初始人物，还是在单人场景自动选择。

这些信息中，1、2、3 和 6 会直接影响数学和解析正确性，不能用默认假设替代。
