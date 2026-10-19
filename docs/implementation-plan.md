# MuJoCo 多相机实验平台设计与实现计划

日期：2026-10-19。状态：渲染基线已实现，并在当前 RTX 4060 Laptop 上完成桌面与后台性能测试；RTX 5060 验收及任务 3–8 仍未完成，任务 2 已实现。

**目标：** 在 Ubuntu 桌面上编辑 MuJoCo 场景，同时观察最多三路相机输出，为视觉算法和机器人策略提供明确的数据与控制接口。

**架构：** Python 应用使用 PySide6 构建桌面界面，MuJoCo 负责模型、物理和渲染。界面、仿真、渲染通过命令队列和不可变快照连接；算法不直接修改仿真状态。

**技术栈：** Python、MuJoCo 官方 Python 包、PySide6、NumPy、pytest。首次性能验证后锁定实际验证通过的版本；不把文档最新版本直接当作部署版本。

**规格：** 本文件“需求与边界”和“数据契约”是本计划的实现依据。

**执行方式：** 使用 executing-plans 在当前项目内按任务执行。任务未完成前保持复选框未选中。每个交付物通过相应检查后单独提交；当前工作区没有 Git 仓库，渲染基线以源码目录交付。

## 需求与边界

### 已确认需求

- Ubuntu 本地桌面应用，目标 GPU 为 RTX 5060。
- 打开完整 MJCF/XML 或 URDF 场景，也可向现有场景追加模型。
- 支持模型引用的资源，调整模型位置和朝向，保存与恢复项目。
- 一个独立全局视图，最多同时启用三个算法相机视图。
- 每路默认 1280×720，目标每路 30 FPS；允许独立设置分辨率。
- 设置相机位姿、成像参数，输出 RGB、深度、相机参数和真实位姿。
- 支持静态编辑、动态仿真；机器人运动时持续观察相机画面。
- 第一阶段准备算法和控制接口；真实位姿估计、视觉定位、机器人策略的具体集成属于后续阶段。

### 建议的第一版行为

- 编辑界面提供场景树、属性面板、全局视图、三路可停靠相机面板和运行状态栏。
- 相机支持鼠标移动/旋转、数值输入、对准选中物体；全局视图支持环绕、平移、缩放，不修改算法相机。
- 支持世界固定相机和挂载到指定 body 的相机。挂载相机存储相对位姿，输出实时世界位姿，适用于后续腕部相机。
- 相机使用理想针孔模型，支持 fx、fy、cx、cy、分辨率、裁剪距离；视场角与内参作为两种编辑方式，不能保存互相冲突的值。畸变、曝光、噪声和滚动快门不在第一版物理成像范围内。
- 静态编辑只更新运动学并渲染，不推进物理时间。动态运行支持开始、暂停、单步、重置。
- 进入运行时保存编辑初态；暂停保留当前状态；重置恢复编辑初态。返回编辑默认恢复初态，另设“将当前状态设为初态”，防止仿真结果无意覆盖布置。
- 结构修改只在编辑态应用。运行时允许移动独立相机；改变内参或分辨率在采样边界原子生效，并更新配置版本。
- 只导入已经展开的 URDF，不处理 Xacro、Gazebo 插件、ROS launch 或自动生成机器人控制器。
- 内置 MuJoCo 能直接加载的网格资源随模型导入；不兼容的网格给出文件名和转换提示，不承诺任意 URDF 一键兼容。

## 技术方案比较

| 方案 | 适用性 | 决策 |
| --- | --- | --- |
| Python + PySide6 + MuJoCo | Python 算法接入直接，适合属性面板和多窗口；需验证图像传输开销 | 采用 |
| C++ + Qt + MuJoCo | 对渲染和内存控制更细，但 Python 算法桥接与开发成本更高 | 性能测量证实必要时再局部引入 |
| 扩展 MuJoCo 自带 viewer | 快速验证模型，但完整项目编辑和多面板工作流仍需大量应用层工作 | 仅用于调试模型 |

PySide6 的停靠组件支持面板停靠与浮动，适用于相机面板布局。[Qt 官方文档](https://doc.qt.io/qtforpython-6/PySide6/QtWidgets/QDockWidget.html)

## 模块与线程边界

1. **项目与模型服务**：加载、追加、命名空间、资源路径、稳定对象 ID、保存；不依赖界面。
2. **仿真服务**：独占可写 mjData，执行命令与物理步进，发布包含模型版本的状态快照。
3. **渲染服务**：独占渲染用 mjData 和 OpenGL 上下文。复制一个状态并更新运动学后，依次生成全局画面和三个相机的 RGB/深度，同批渲染期间不换状态。
4. **界面**：主线程只处理输入和显示；通过有界队列接收最新一批图像。选择/拖拽事件交给使用同一场景快照的交互服务计算。
5. **数据与扩展接口**：发布相机观测；真实位姿另行发布给评估器；接收带帧标识的估计结果和带有效期的控制命令。

MuJoCo 渲染要求调用线程具有有效 OpenGL 上下文，因此不能让界面和工作线程任意共享并操作同一上下文。[MuJoCo Python 文档](https://mujoco.readthedocs.io/en/stable/python.html)

物理步长独立于 30 Hz 采样周期。样本记录实际仿真时间和步号；不要求每帧恰好跨越相同步数。全局画面不是第四路算法相机。

模型重新编译采用“候选编译成功后切换”：先暂停生产，构建新模型与渲染资源，再切换整个版本；失败保留原模型。旧版本快照、估计结果和命令不得混用。

## 模型导入与保存

- 完整打开时采用源场景的物理参数；追加时保留当前场景的全局参数，对冲突给出报告。
- 使用 MjSpec 模型编辑能力组合模型；每个实例分配唯一前缀并维护内部引用，禁止简单拼接 XML 字符串。[MuJoCo 模型编辑文档](https://mujoco.readthedocs.io/en/stable/programming/modeledit.html)
- 资源解析支持相对路径、绝对路径及用户配置的 package:// 包目录映射；不要求安装 ROS。
- URDF 导入显式检查 visual/collision 保留策略、静态 link 合并和惯性。保留对象追踪所需的 link/body 身份，不让默认优化静默破坏位姿接口。
- 仅有 URDF 的关节定义不等于已经具有可执行控制策略。没有 actuator 的模型仍可查看和被动物理仿真；控制接口明确返回不支持的通道。
- MuJoCo 对 MJCF 与 URDF 有不同的编译默认值，URDF 也只能表达其模型能力的一部分。[URDF 官方说明](https://mujoco.readthedocs.io/en/stable/modeling.html#urdf-extensions)
- 项目目录采用 project.json、scene.xml、assets/。保存规范化场景、资源副本、对象映射、相机配置、初态、物理配置和界面布局，使用临时目录完成后原子替换。
- 不覆盖原始导入文件；导出后的 MJCF 不保证保持原 XML 注释和 include 排版。项目重新打开默认进入编辑初态，不承诺完整求解器运行时检查点恢复。

## 数据契约

| 类型 | 必需字段与约定 |
| --- | --- |
| CameraConfig | camera_id、revision、parent_body_id、相对位姿、width/height、fx/fy/cx/cy、near/far、输出通道 |
| ObservationBatch | run_id、model_revision、batch_id、step_index、sim_time_s、host_monotonic_ns、同一状态的各相机帧 |
| CameraFrame | camera_id、camera_revision、RGB uint8[H,W,3]、depth float32[H,W]、valid_mask、K、T_world_camera |
| GroundTruthBatch | 与 ObservationBatch 相同标识，按 object_id 提供 T_world_object；仅评估消费者接收 |
| PoseEstimate | run_id、model_revision、batch_id、camera_id、object_id、T_camera_object、confidence、算法耗时 |
| ControlCommand | run_id、model_revision、目标步、有效截止步、actuator 名称、控制值、控制语义 |

所有长度为米、时间为秒；旋转以 4×4 刚体变换为接口权威表达。T_A_B 表示将 B 坐标映射到 A。相机接口遵循视觉习惯：x 向右、y 向下、z 向前；适配层转换 MuJoCo 相机坐标。像素以左上角像素中心为 (0,0)，必须用投影测试验证半像素偏移和主点符号。

深度定义为光学轴 Z 深度，不是射线距离；无有效命中或超出范围的像素置 NaN 并输出 valid_mask。首版 RGB 与深度使用同一虚拟相机投影，不模拟真实 RGB-D 相机的双传感器基线。

主点接口为图像坐标，MuJoCo principalpixel 则表示相对图像中心的偏移；适配时显式换算，不能直接复制。相机参数能力依据[相机 XML 文档](https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-camera)。

未来评估按 T_camera_object = inverse(T_world_camera) × T_world_object 得到真值，计算平移欧氏距离与 SO(3) 旋转角误差，不直接相减欧拉角。缓存按帧标识查找；结果过期时标注无法匹配，不使用当前位姿替代。

第一版提供 Python 订阅接口及样例消费者。观测队列容量为两批，满时丢弃最旧整批并统计丢帧；慢算法不能阻塞仿真。真实算法进程隔离、共享内存传输、ROS 2 适配放在后续阶段。

## 建议代码结构

```text
pyproject.toml                     依赖与命令入口
src/camera_lab/app.py              应用生命周期
src/camera_lab/contracts.py        配置、观测、真值、控制类型
src/camera_lab/coordinates.py      坐标与投影转换
src/camera_lab/project/importer.py MJCF/URDF 加载和组合
src/camera_lab/project/assets.py   资源解析与归档
src/camera_lab/project/store.py    项目保存与恢复
src/camera_lab/sim/engine.py       仿真状态机与命令执行
src/camera_lab/sim/snapshot.py     完整状态快照
src/camera_lab/render/worker.py    上下文生命周期与批量渲染
src/camera_lab/render/camera.py    内参与挂载相机适配
src/camera_lab/ui/window.py        主窗口与布局
src/camera_lab/ui/viewport.py      图像显示与鼠标交互
src/camera_lab/ui/properties.py    对象与相机属性
src/camera_lab/api/streams.py      有界订阅接口
src/camera_lab/api/control.py      控制校验与队列
examples/consume_frames.py        订阅示例
tests/                            数据、模型、投影、状态机与集成测试
benchmarks/three_cameras.py       性能基准与报告
```

## 实施顺序与验收

### 任务 1 验证渲染链路与性能基线

文件：pyproject.toml、render/worker.py、benchmarks/three_cameras.py、tests/test_render_smoke.py。

- [x] 建立最小应用和内置测试场景：地面、桌面、三个彩色刚体、一个带 actuator 的摆杆、三台相机。
- [ ] 在目标 Ubuntu/5060 上验证 PySide6 主线程与渲染线程的上下文创建、关闭和重复启动。
- [x] 输出三路 720p RGB 和 float32 深度，并显示全局视图；检查 dtype、尺寸、非空图像和窗口响应。
- [ ] 用 `python benchmarks/three_cameras.py --cameras 3 --width 1280 --height 720 --seconds 60` 测量预热后的帧率、P95 批次耗时、实际物理速度、CPU/GPU 利用率和内存。
- [x] 保存依赖锁文件及机器/驱动信息。若不达标，先定位物理、渲染、读回或显示瓶颈，再决定优化；不得静默降低验收分辨率。

验收：三路与全局视图完整运行；性能报告可复现。这个任务先于完整界面开发。

### 任务 2 固定坐标与帧数据契约

文件：contracts.py、coordinates.py、render/camera.py、tests/test_projection.py、tests/test_frames.py。

- [x] 用已知相机前方两米的平面构建深度测试，中心区域误差小于 1 毫米。
- [x] 用偏心主点和不同 fx/fy 构建标记投影测试，验证理论像素和实际渲染位置误差不超过 1 像素。
- [x] 实现上表类型、单位、不可变发布和坐标适配；限制 fx/fy>0、0<near<far、分辨率为正整数。
- [x] 验证修改相机参数后新图像和新 K 同时生效，三个相机的 batch_id、step_index 和 sim_time 一致。
- [x] 运行 `pytest tests/test_projection.py tests/test_frames.py -v`，通过后提交该交付物。

### 任务 3 场景导入与追加

文件：project/importer.py、project/assets.py、tests/test_importer.py、tests/fixtures/。

- [ ] 编写独立 MJCF、带 visual/collision 的 URDF、嵌套 include、重名资源、缺失网格和 package:// 测试模型。
- [ ] 实现完整打开及追加，处理根部位姿、freejoint 初态、对象命名空间和资源目录。
- [ ] 对不能保真的全局配置、外部插件依赖或执行器引用冲突返回具体诊断；原场景保持有效。
- [ ] 验证重复追加同一机器人两次仍能分别取得 body 位姿；失败导入不改变当前模型版本。
- [ ] 运行 `pytest tests/test_importer.py -v`，通过后提交该交付物。

### 任务 4 静态编辑与多相机桌面界面

文件：ui/window.py、ui/viewport.py、ui/properties.py、tests/test_camera_editing.py。

- [ ] 构建场景树、属性面板、全局视图和三路可浮动面板；隐藏面板不默认停止其数据输出。
- [ ] 实现对象与相机选择、平移/旋转、数值输入、相机对准目标、视场显示、RGB/深度显示切换。
- [ ] 实现世界固定/挂载相机，鼠标调整相机不修改全局观察视角，反向也不修改。
- [ ] 运行 `pytest tests/test_camera_editing.py -v`，并人工完成导入、摆放、设相机、同时观察三路画面的完整流程。

验收：编辑状态物体不因重力移动；挂载相机能保持相对父 body 的位姿。

### 任务 5 动态仿真与状态切换

文件：sim/engine.py、sim/snapshot.py、tests/test_simulation.py、tests/test_snapshot.py。

- [ ] 编写开始、暂停、单步、重置、返回编辑和保存当前为初态的状态迁移测试。
- [ ] 实现仿真独占状态、按需完整快照、重编译版本切换，以及渲染前的运动学更新。
- [ ] 用运动摆杆验证连续图像随仿真变化，三路真值仍对应相同采样状态。
- [ ] 验证暂停后时间不变但独立相机仍可移动；重置更新 run_id 并拒绝旧命令。
- [ ] 运行 `pytest tests/test_simulation.py tests/test_snapshot.py -v`，通过后提交该交付物。

### 任务 6 项目保存与恢复

文件：project/store.py、tests/test_project_roundtrip.py。

- [ ] 实现规范化 MJCF、资源归档、项目 schema_version、稳定 ID、相机配置、初态和布局保存。
- [ ] 将保存的项目复制到新目录并移除测试用原资源目录，重新打开并比较场景、相机参数及对象身份。
- [ ] 模拟写入失败，验证旧项目仍能打开；未知 schema_version 给出明确错误。
- [ ] 运行 `pytest tests/test_project_roundtrip.py -v`，通过后提交该交付物。

### 任务 7 数据接口和机器人控制接口

文件：api/streams.py、api/control.py、examples/consume_frames.py、tests/test_streams.py、tests/test_control.py。

- [ ] 实现图像订阅与独立真值订阅，提供按帧标识关联的样例消费者。
- [ ] 使用故意延迟的消费者验证丢弃最旧批次、队列有界、没有混帧，仿真继续运行。
- [ ] 定义估计结果入口与帧匹配；用合成估计结果校验坐标和时间契约，不接入真实算法。
- [ ] 为内置摆杆发送有效控制命令；校验 actuator 名称、有限数值、控制范围、模型版本和有效期。过期命令拒绝，超时恢复明确配置的默认控制。
- [ ] 运行 `pytest tests/test_streams.py tests/test_control.py -v`，通过后提交该交付物。

验收：外部样例能消费数据；运行中的控制示例可使模型运动并更新画面。此示例只验证接口，不包含机器人策略产品功能。

### 任务 8 综合验收与交付

文件：benchmarks/three_cameras.py、tests/test_workflow.py、README.md、docs/user-guide.md。

- [ ] 完成“打开场景—追加 URDF—摆放—设置三相机—保存重开—运行—订阅数据—暂停重置”的集成测试。
- [ ] 执行 `pytest -v`；需 GPU 的测试明确标记并在真实图形环境执行，不能把跳过计为通过。
- [ ] 在 RTX 5060 上预热 10 秒、测量 60 秒：三路各 1280×720 RGB+深度、全局视图可见、物理实时运行；记录每路实际输出帧率和完整批次 P95。
- [ ] 性能目标：每路约 30 FPS、批次处理 P95 不超过 33.3 毫秒、无持续队列堆积；对内置场景和用户代表性模型分别报告，失败不降低指标冒充通过。
- [ ] 执行 30 分钟运行及重复打开/关闭场景测试，检查内存稳定、上下文释放与错误恢复。
- [ ] 交付安装启动说明、模型支持范围、坐标与数据契约、示例项目、性能报告和故障诊断说明。

## 后续阶段

第二阶段接入一个真实物体位姿估计算法，提供预测叠加、平移/旋转误差、延迟与丢帧统计。对称物体另定义对称感知评价，不能仅用姿态角判断算法正确性。

第三阶段接入机器人策略：控制周期、动作语义、关节/执行器映射、复位、超时和回放；随后加入视觉定位。只有外部集成需要时才添加 ROS 2 或网络桥接。

## 计划覆盖检查

| 需求 | 任务 |
| --- | --- |
| MJCF/URDF 打开与追加 | 3 |
| 模型与相机编辑、多视图 | 4 |
| 相机参数与 RGB/深度/真值 | 2、7 |
| 静态/动态切换 | 5 |
| 保存恢复 | 6 |
| 算法和机器人接口 | 7 |
| 三路 720p、30 FPS、Ubuntu/5060 | 1、8 |

首先执行任务 1。完整相机编辑功能的开发以渲染链路及坐标测试通过为前提；性能风险应在 UI 大量开发前暴露。

## 当前实现记录

2026-10-19：已交付 outputs/camera-lab 渲染基线。当前实际硬件为 RTX 4060 Laptop，桌面 60 秒测试每路 29.85 FPS，渲染 P95 22.90 ms。任务 1 中针对 RTX 5060 的验收仍未完成；其余功能任务尚未执行。详细记录见 camera-lab/reports/verification.md。

2026-10-19：任务 2 已实现。增加相机内参/独立分辨率与裁剪接口、光学坐标转换、同步采样标识及独立真值批次；包含实际渲染投影测试和样本导出脚本。相机挂载沿用导入模型已有的父 body，编辑挂载关系属于任务 4。PoseEstimate 与 ControlCommand 的运行时接入仍按任务 7 实现。
