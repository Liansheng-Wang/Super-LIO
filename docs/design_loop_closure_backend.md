# Super-LIO 后端回环闭合 & 全局优化 —— 技术方案

> 文档状态: **草案 / 待实现**。本方案已与项目维护者对齐方向,尚未开工。
> 目的: 给后续实现者(可能是你)提供架构、技术选型依据和分阶段验收标准,减少"边写边猜"的返工。
> 参考来源: [`hku-mars/Voxel-SLAM`](https://github.com/hku-mars/Voxel-SLAM)(BTC 回环检测 + GTSAM 位姿图 + BALM2 风格体素全局 BA)。

---

## 1. 目标与非目标

### 目标(Building)

给 Super-LIO(当前只有 LIO 前端,**无关键帧、无位姿图、无回环检测**)补上一套后端:

1. **BTC/STD 三角描述子回环检测** —— 从关键帧点云中识别"回到过的地方"。
2. **单会话 GTSAM 位姿图优化** —— 用里程计边 + 回环边修正关键帧轨迹漂移。
3. **BALM2 风格分层体素全局 BA** —— 在位姿图收敛之后,对全部关键帧点云做一次平面级精修,进一步提升建图一致性(对应 Voxel-SLAM 里 `finish` 触发的"最终全局优化")。

### 非目标(Not building,明确排除)

- **不移植 Voxel-SLAM 自己的前端**(`preintegration.hpp` / `ekf_imu.hpp` / `feature_point.hpp` 里的 IMU 预积分 + EKF + 特征提取)。Super-LIO 已有可用的 18 维 ESKF(见 `src/super_lio/include/lio/ESKF.h`),不维护第二套状态估计器,也因此不需要 GTSAM 的 `ImuFactor` / `CombinedImuFactor`。
- **不移植多会话地图拼接**(`loop_refine.hpp` 里的 `PGO_Edge` / `PGO_Edges` / `is_adapt` / `connect`)。这是 Voxel-SLAM 支持"多次建图自动融合"的机制,和"单次建图内部消除漂移"是两个不同需求,先不做。
- **不改造 Super-LIO 现有的 `OctVoxMap` 增量体素索引**。全局 BA 用的是 Voxel-SLAM 自己的自适应八叉树平面结构(`OctoTree` / `LidarFactor`),和前端用于 KNN 最近邻匹配的 `OctVoxMap` 是两套独立表示,不做统一,避免影响前端已验证的实时性能。
- **不把优化结果回写进实时 ESKF / TF**。所有回环修正只作用于"保存地图"这条离线路径,不在运行时闭环,避免引入运行时抖动/震荡的新故障模式。

---

## 2. 总体架构

### 2.1 为什么要拆成三个 ROS2 包(许可证边界是硬约束)

Voxel-SLAM 仓库根目录 `LICENSE` 是纯 **GPLv2**(不带 "or later" 字样),`VoxelSLAM/package.xml` 的 license 字段甚至是 `TODO`(上游没填,视为未声明)。Super-LIO 自身是 **GPLv3**。GPLv2(无 or-later)与 GPLv3 在 FSF 的解释下是**互不兼容**的两个协议 —— 不能把两者的代码编译进同一个二进制 / 同一个静态库。

处理方式:**拆成三个独立 ROS2 包,新增的两个包与现有 `super_lio` 之间只通过进程间通信(ROS2 topic / service)交互,不做代码级链接**:

| 包名 | 许可证 | 内容 | 是否新增 |
|---|---|---|---|
| `super_lio` | GPLv3(不变) | 现有 LIO 前端 + 新增的关键帧发布器 | 已存在,小改动 |
| `super_lio_loop_msgs` | Apache-2.0 / BSD-3-Clause(纯接口包,无逻辑代码) | `Keyframe.msg` / `LoopCandidate.msg` / `GetCorrectedPoses.srv` 等 IDL 定义 | **新增** |
| `super_lio_loop_backend` | GPLv2(承接 Voxel-SLAM 原始许可证) | BTC 回环检测 + GTSAM 位姿图 + OctreeGBA 全局 BA,移植自 Voxel-SLAM | **新增** |

`super_lio_loop_msgs` 单独成包是为了让 `super_lio`(GPLv3)和 `super_lio_loop_backend`(GPLv2)都只依赖一个"许可证宽松、无实质代码"的接口包,而不是互相依赖对方的生成头文件 —— 这是 ROS 生态里处理跨许可证节点通信的标准做法(类似很多 GPL 驱动节点和 BSD/Apache 许可证消息包共存的模式)。

**这不是免责声明,是需要显式执行的动作**:

- `super_lio_loop_backend` 包内必须带 `THIRD_PARTY_NOTICE.md`,注明代码来源 `hku-mars/Voxel-SLAM`、保留原始 GPLv2 协议全文、列出被移植的文件清单。
- 若未来联系到 Voxel-SLAM 作者拿到 "GPLv3 或 GPLv3-or-later" 的书面授权,可以简化为直接并入 `super_lio` 主库 —— 这是长期可选项,不阻塞当前工作(见 §8 风险清单)。

### 2.2 数据流

```
┌─────────────────────────────┐
│  super_lio (GPLv3, 现有)     │
│  ESKF 前端里程计              │
│                              │
│  [新增] KeyframeManager:     │
│   位移/旋转超阈值 → 触发关键帧  │
└──────────────┬───────────────┘
               │ topic: /super_lio/keyframe
               │ (Keyframe.msg: id, world_pose, lidar系降采样点云)
               ▼
┌─────────────────────────────────────────────┐
│  super_lio_loop_backend (GPLv2, 新增, 独立进程) │
│                                               │
│  ① BTC/STD 回环检测                            │
│     体素化→平面提取→三角描述子→哈希检索→平面ICP验证  │
│     → 候选回环边 (id_i, id_j, 相对位姿, 置信度)     │
│                                               │
│  ② GTSAM 单会话位姿图 (ISAM2)                    │
│     里程计边(相邻关键帧 ESKF 相对位姿)              │
│     + 回环边(①的验证结果)                         │
│     → 修正后的关键帧位姿                          │
│                                               │
│  ③ OctreeGBA 全局体素 BA (仅在触发时运行,非实时)    │
│     ②的位姿 + 各关键帧点云+协方差 → 自适应八叉树      │
│     → 平面级 BA 精修 → 最终关键帧位姿                │
└──────────────────┬────────────────────────────┘
                    │ service: /super_lio_loop_backend/get_corrected_poses
                    ▼
┌─────────────────────────────┐
│  super_lio 保存地图流程         │
│  用修正后位姿重新拼接关键帧点云   │
│  → 落盘 global.pcd / 分块地图    │
└─────────────────────────────┘
```

关键设计点:数据流是**单向 + 请求式**,没有运行时反馈环路。`super_lio` 只在"保存地图"这个动作触发时才调用 `get_corrected_poses` 服务拉取一次修正结果,平时两个进程之间只有关键帧的单向发布,不存在运行时闭环导致的稳定性风险。

### 2.3 接口草案(供实现者细化,非最终定稿)

`super_lio_loop_msgs`:

```text
# Keyframe.msg
std_msgs/Header header
uint32 id
geometry_msgs/Pose world_pose      # 触发时刻的 ESKF 世界系位姿(初值,供位姿图使用)
sensor_msgs/PointCloud2 cloud      # LiDAR/body 系降采样点云(未做世界系变换!)

# LoopCandidate.msg   (backend 内部调试/可视化用,不要求 super_lio 订阅)
uint32 id_i
uint32 id_j
geometry_msgs/Pose relative_pose
float32 score

# GetCorrectedPoses.srv
---
uint32[] ids
geometry_msgs/Pose[] corrected_poses
bool success
```

`Keyframe.msg` 里的点云**必须是 body/LiDAR 系**,不能提前转到世界系 —— 位姿图和全局 BA 优化的是每个关键帧的位姿本身,如果点云已经用初始位姿转好,优化后就没法重新应用修正后的位姿。

**每点协方差不通过网络传输**:Voxel-SLAM 的 `pointVar`(点+3x3协方差)是由固定的传感器噪声模型(测距误差、角分辨率)在 `calcBodyVar` 里从点的 range 现算出来的,不依赖 ESKF 协方差。因此 `super_lio_loop_backend` 收到原始点云后本地重新计算协方差即可,不需要在 `Keyframe.msg` 里塞一个协方差数组,省掉大量带宽。

---

## 3. 技术选型

| 决策点 | 选定方案 | 理由 | 被否决的备选 |
|---|---|---|---|
| 回环检测器 | **BTC/STD**(三角描述子,`BTC.h/.cpp` 移植) | 与整体移植目标(照搬 Voxel-SLAM 已验证实现)一致;不依赖学习模型,纯 PCL/Eigen,依赖少;论文在 HILTI/ICCV 竞赛中验证过室内外场景 | Scan Context(更轻量但需重新验证效果,若后续想收窄依赖可作为二期替换项);基于学习的描述子(引入深度学习框架,过重,直接排除) |
| 位姿图优化库 | **GTSAM ISAM2** | 用户明确要求"连 GTSAM 全局 BA 一起搬";Voxel-SLAM 原生实现基于 GTSAM,复用可验证代码,增量优化性能成熟 | g2o(workspace 里另一个项目 `lightning-lm` 用的是 g2o 抽象层,但改用 g2o 需要重写 `loop_refine.hpp` 里全部因子/求解器接口,偏离已验证实现,工作量更大);Ceres(同理未选) |
| 全局精修优化器 | **Voxel-SLAM 自带 `OctreeGBA` + `LidarFactor`**(BALM2 风格) | 与位姿图共用同一套平面表征和点协方差模型,数据结构衔接成本低,是经过论文验证的数值代码 | 基于 Super-LIO 自己的 `OctVoxMap` 改造出全局 BA —— `OctVoxMap` 是为增量 KNN 查询设计的,不是为平面级 BA 设计的自适应八叉树,改造成本高于直接复用,除非未来发现移植适配有严重问题 |
| 包/进程边界 | **三包拆分 + ROS2 IPC 通信**(见 §2.1) | 规避 GPLv2/GPLv3 许可证冲突;GTSAM 依赖只影响装了 `super_lio_loop_backend` 的用户,现有用户零影响 | 直接把 Voxel-SLAM 代码链接进 `super_lio` 现有 `lio` 库(许可证风险,不选,除非拿到上游 relicense 确认) |

---

## 4. 关键数据结构(移植时的类型映射)

| Voxel-SLAM 类型(`tools.hpp` / `voxel_map.hpp`) | Super-LIO 对应类型(`basic/alias.h`) | 说明 |
|---|---|---|
| `IMUST`(旋转+位置+速度+bias 状态) | `BASIC::SE3` + 额外字段 | 只有位姿部分(R, p)需要跨包传输;bias/v 是 Voxel-SLAM 自己前端才用,新代码里可以精简掉不需要的字段 |
| `pointVar`(点 + 3x3 协方差) | 无对应,新增 | 本地在 `super_lio_loop_backend` 内计算,见 §2.3 |
| `PVec` / `PVecPtr` | `std::vector<pointVar>` | 原样保留 |
| `ScanPose` | 新写一个薄封装,持有 `BASIC::SE3` 初值 + 优化后位姿 + 对应 keyframe id | 不直接照搬,因为原结构耦合了 Voxel-SLAM 自己的 `IMUST` |

`voxel_map.hpp`(约 1700 行,`OctoTree` / `LidarFactor` / `Lidar_BA_Optimizer` / `LI_BA_Optimizer`)和 `tools.hpp` 建议**原样移植,只在包边界做一次类型转换**(`BASIC::SE3` ↔ `IMUST`),不要用 `BASIC::SE3`/`SO3` 模板重写这部分数学 —— 这是经过论文验证的数值代码,重写收益低、风险高。

---

## 5. `super_lio` 侧需要新增的最小改动

现状(已确认):`src/super_lio/src/lio/super_lio.cpp` 里的 `caceData()` / `ProcessCaceMap()` / `saveMap()` 只是按固定间隔把整帧点云存成 `scans_N.pcd`,没有关键帧筛选、没有位姿图节点概念。

需要新增:

1. `include/lio/keyframe_manager.h` + `src/lio/keyframe_manager.cpp`:
   - 输入:每帧的 `NavState`(来自 `ESKF::GetNavState()`)+ 降采样点云。
   - 触发条件:相对上一关键帧平移 > `lio.loop.kf_trans_thresh` **或** 旋转 > `lio.loop.kf_rot_thresh`。
   - 输出:递增 id + `Keyframe.msg`,通过 `ROSWrapper` 新增的 publisher 发出。
2. `ROSWrapper` 新增 `pub_keyframe_`(`super_lio_loop_msgs::msg::Keyframe`)和对应的 `pub_keyframe(...)` 方法,仿照现有 `pub_cloud_world` 的写法。
3. `saveMap()` 里新增一段:若 `lio.loop.enable_backend` 为 true,调用 `GetCorrectedPoses` service client,拿到修正位姿后按 id 替换关键帧位姿,再执行现有的点云拼接落盘逻辑;service 不可用或超时则退回现有行为(不阻塞现有用户)。
4. 全部改动挂在 yaml 开关后面,关闭时(默认)对现有性能 **零影响** —— 这是 Phase 1 验收的硬指标。

---

## 6. 分阶段实施与验收条件

> 原则:每个阶段独立可合并、独立提供价值,不要求后面阶段完成。数值门槛是建议初始值,实测后应在本文档同步更新,不要在代码里"悄悄"改而不回填文档。

### Phase 1 —— 基础设施 + 关键帧发布 + BTC 回环检测(仅检测,不做修正)

**范围**:`super_lio_loop_msgs` 包、`super_lio` 的 `KeyframeManager`、`super_lio_loop_backend` 包骨架 + BTC 移植。

**验收条件**:

- [ ] `colcon build --packages-select super_lio_loop_msgs super_lio_loop_backend super_lio` 全部成功;`super_lio_loop_backend` 的 `CMakeLists.txt` 用标准 `find_package(GTSAM REQUIRED)`(非 ament 宏)。GTSAM 在目标机器上通常需要 `ppa:borglab/gtsam-release-4.1`(Ubuntu)安装 `libgtsam-dev`,或从源码按 tag `4.2` 编译 —— 不是标准 ROS apt 源的一部分,需在实现前于目标机器实测确认。
- [ ] `lio.loop.enable_keyframe_pub` 关闭时,同一段 bag 跑两次(改动前/改动后),处理耗时和现有输出话题内容不变(用 `Timer::PrintAll()` 或等效手段对比)。
- [ ] 打开开关后,按 `kf_trans_thresh`(建议初值 1.0 m)/ `kf_rot_thresh`(建议初值 15°)正确触发关键帧,id 严格递增,`Keyframe.msg` 里的点云确认是 body 系(未做世界系变换)。
- [ ] 在一段"走一圈回到起点"的测试 bag 上,`super_lio_loop_backend` 输出的 `/super_lio_loop_backend/loop_candidates` 能正确匹配上起点/终点关键帧(人工核对至少 1 个已知回环场景)。
- [ ] 在一段直线走廊 / 无重复路径的测试 bag 上,不产生任何候选(人工复核确认无误报)。

### Phase 2 —— GTSAM 单会话位姿图

**范围**:里程计边 + 回环边接入 GTSAM ISAM2,`GetCorrectedPoses` service,`super_lio` 保存地图时接入修正结果。

**验收条件**:

- [ ] 里程计边(相邻关键帧 ESKF 相对位姿 + 噪声模型)和回环边(Phase1 验证通过的候选,经 plane-to-plane ICP 精修相对位姿)都能正确插入因子图,ISAM2 增量更新不 crash。
- [ ] 单次关键帧到达时的增量优化耗时在实测硬件上给出量化数字(建议目标 <50ms/keyframe,具体以实测为准并回填本文档)。
- [ ] `GetCorrectedPoses` service 返回的位姿数组能被 `super_lio` 正确消费,重新拼接出的地图文件可以正常加载(用现有 `run_loc_online`/查看工具验证)。
- [ ] **量化对比**:选一段有明显回环的测试 bag,对比"仅里程计拼图" vs "回环修正后拼图"在回环闭合点的地图错位(如同一面墙的双重成像)是否消失,以及首尾位姿误差下降幅度,把对比结果(数值或截图)记录进 PR 描述。
- [ ] 关闭该功能(`enable_backend=false` 或不启动 backend 节点)时,现有建图/保存流程回归测试通过,行为与改动前完全一致。

### Phase 3 —— `OctreeGBA` 全局体素 BA 精修

**范围**:Phase2 输出的位姿 + 关键帧点云喂给 `OctreeGBA`,做一次离线的分层体素平面 BA。

**验收条件**:

- [ ] `OctreeGBA` 能正确接收全部关键帧的(修正位姿, 点云)对,构建自适应八叉树并完成平面级 BA,不 crash、不发散。
- [ ] **量化对比**:同一测试 bag 下,对比 Phase2 输出 vs Phase3 输出的局部平面一致性(如墙面拟合平面的法向量离散度或"墙厚度"指标)。若无明显改善,需在 PR 里记录原因分析(如原始点云噪声已是瓶颈),而不是强行判定通过。
- [ ] 给出实测耗时量级(如"N 个关键帧下 < X 分钟"),明确这是非实时离线步骤。
- [ ] 全局 BA 结果只影响落盘地图文件,**不回写实时里程计 / TF**——回归测试确认开启/关闭 Phase3 时运行时话题和 TF 输出完全一致。

---

## 7. yaml 配置项草案

`super_lio` 侧(追加到现有 `config/*.yaml`,沿用 `lio.<section>.<key>` 风格):

```yaml
lio.loop.enable_keyframe_pub: false   # 总开关,默认关闭,零成本
lio.loop.kf_trans_thresh: 1.0         # 米
lio.loop.kf_rot_thresh: 15.0          # 度
lio.loop.keyframe_topic: "/super_lio/keyframe"
lio.loop.enable_backend: false        # 保存地图时是否调用 get_corrected_poses
lio.loop.backend_service_timeout: 5.0 # 秒,超时则退回现有保存逻辑
```

`super_lio_loop_backend` 侧(新建 `config/loop_backend.yaml`,独立命名空间):

```yaml
loop.btc.voxel_size: 1.0
loop.btc.score_threshold: 0.75        # 具体含义/初值需参考 Voxel-SLAM 原始 ConfigSetting 后填充
loop.pgo.odom_noise: [...]            # 里程计边噪声模型
loop.pgo.loop_noise: [...]            # 回环边噪声模型
loop.gba.voxel_size: 1.0
loop.gba.min_eigen_value: ...
```

> 具体数值需要在 Phase1/2/3 实现过程中参考 Voxel-SLAM 原始 `ConfigSetting` / `gba_*` 全局变量的默认值,并结合实测调整,这里只列字段结构。

---

## 8. 风险与待确认事项

| 项 | 说明 | owner / 解决路径 |
|---|---|---|
| GPLv2 vs GPLv3 许可证 | 已通过 §2.1 的三包拆分架构规避链接层面的冲突;`VoxelSLAM/package.xml` license 字段是 `TODO`,建议同时给 hku-mars 提 issue 询问是否可以补充为 "or later" | 长期项,不阻塞当前实现;若拿到确认可简化架构 |
| GTSAM 安装方式 | ROS2 官方 apt 源通常不含 GTSAM ament 包,需 PPA 或源码编译,具体在目标部署机器(Ubuntu 22.04/24.04, Humble/Jazzy)上未验证 | 实现 Phase1 前需要在目标机器上跑一次确认,回填本文档 §6 Phase1 验收项 |
| 关键帧阈值 / BTC 参数初值 | `kf_trans_thresh` / `kf_rot_thresh` / BTC `ConfigSetting` 里的参数强依赖场景(室内走廊 vs 室外开阔地),Voxel-SLAM 自己也是配置项而非固定值 | 留到 Phase1 用实际测试 bag 调,调完回填 §7 |
| 里程计边噪声模型在退化场景下的可信度 | 当前假设"ESKF 在两个关键帧之间轨迹足够准,可以用简单 between-factor";如果某段轨迹漂移很大(长走廊/几何退化),里程计边噪声需要相应放大,否则位姿图会错误地压制正确的回环修正 | Phase2 实现时先记录为已知限制;如实测中出现问题,再考虑引入更精细的噪声估计(不引入 IMU 预积分因子,保持前端单一职责) |

---

## 9. 参考资料

- [hku-mars/Voxel-SLAM](https://github.com/hku-mars/Voxel-SLAM) —— BTC 回环检测 + GTSAM 位姿图 + BALM2 风格全局 BA 的原始实现,本方案的直接移植来源。
- BALM2: `Liu, Zheng et al.` —— 体素级平面 BA 的数学基础,对应 `voxel_map.hpp` 里的 `LidarFactor`。
- Super-LIO 现有前端:`src/super_lio/include/lio/ESKF.h`、`src/super_lio/include/lio/super_lio.h`、`src/super_lio/include/ros/ROSWrapper.h`、`src/super_lio/include/common/ds.h`。
