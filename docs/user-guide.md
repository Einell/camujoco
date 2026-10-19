# 使用指南

## 启动与建立场景

在仓库目录执行 `./run.sh`。首次安装或迁移到新机器，请先按 [README](../README.md) 建立 Python 环境。启动后主窗口显示独立 GLOBAL 视图与三路相机；默认各路 1280×720、目标 30 FPS，同时输出 RGB 和深度。

快速体验：点击 Open project，选择 `examples/projects/table-robot.camproj`，再点击 Running。这个简化 URDF 机器人用于测试导入与相机接口，没有机器人策略或自动生成的执行器。

从模型开始：

1. Open model 选择 MJCF `.xml` 或 `.urdf`，替换当前场景。
2. Append model 添加模型，填写位置（米）和 XYZ 欧拉角（度）。例如先打开 `examples/models/table.xml`，再追加 `examples/models/simple_robot.urdf`，Z 填 0.75。
3. 有 `package://` 引用时，先通过 Map package 指定包名和包目录。Xacro 需提前展开。
4. 在 Scene properties 选择物体，设置 XYZ 和 wxyz 四元数并 Apply body。修改物体会暂停并重置整个仿真到新初态。

模型支持范围和追加行为见 [模型导入](model-import.md)。

## 调整与观察相机

在场景树 Cameras 中选中相机。Camera parent 为 world 时相机固定在世界中，选择物体则随该物体运动。位姿数值按所选父坐标系解释，改变父节点后需要重新检查相机位置。

相机局部坐标为 X 向右、Y 向下、Z 向前。可设置分辨率、fx/fy/cx/cy 内参和 near/far 裁剪距离；点击 Apply camera 生效。选择目标物体后点击 Aim camera at body，可对准其当前原点。

- GLOBAL：左键拖动旋转、滚轮调整观察距离，独立于输出相机。
- 相机画面：左键拖动沿局部 XY 平移，滚轮沿光轴前后移动。这里的“缩放”是移动相机；调整焦距请修改 fx/fy。
- 相机操作保持当前运行或暂停状态，不会因滚轮操作停止物理仿真。
- Depth 仅切换显示，后台仍输出 RGB 与深度。深度预览范围固定为 0–5 米，黑色表示无效深度；算法应读取实际浮点深度与 valid_mask。
- Camera frustums 只在 GLOBAL 中显示相机视场提示，不污染相机输出。
- Float view 将预览移到独立窗口，关闭浮动窗口后返回主窗口。

详细说明见 [编辑操作](scene-editor.md) 和 [坐标、图像契约](camera-data-api.md)。

## 编辑、运行与保存

Running / Pause 保持当前位置并切换物理时间推进；Single step 推进一步后暂停。Reset initial 与 Return to edit 恢复初态并清零时间。暂停期间仍可移动相机和获取新图像。

要保留运行中某一姿态，先点击 Use pose as initial，再 Save project。直接保存项目保存的是既定初态，不是动态求解器检查点。Save project 会暂停仿真。

`.camproj` 包含基础场景、资源、初态、相机和布局，可移动到另一个目录后重新打开。Open project 替换当前场景，进入 edit 模式；点击 Running 开始。重置或重开会生成新的 run_id，算法端应按新的版本接收数据。详见 [项目文件](project-storage.md) 和 [仿真状态](simulation.md)。

## 接入算法和控制

当前提供 Python 进程内接口。算法只消费观测订阅；评估模块另外订阅真值，按完整 FrameStamp 对齐。队列有界，慢消费者丢弃旧批次，不阻塞渲染。

`examples/consume_frames.py` 演示订阅；`examples/evaluate_pose.py` 使用合成误差验证位姿评估；`examples/control_pendulum.py` 演示执行器命令与超时。真实位姿估计算法、视觉定位和机器人策略仍待后续接入。URDF 导入不会自动创建 actuator，不能将任意关节名直接当执行器名。

字段、线程规则、控制语义及超时机制见 [数据与控制接口](data-control-api.md)。

## 故障排查

| 现象 | 检查方法 |
| --- | --- |
| Qt 提示 xcb 或 xcb-cursor 缺失 | 使用 `./run.sh`；新机器按报错安装对应 Ubuntu Qt/xcb 运行库。本机 `.runtime` 不随 Git 分发。 |
| EGL 初始化失败 | 检查 `nvidia-smi` 是否正常，以及会话是否能访问 GPU；不要把 Qt 依赖缺失当成显卡驱动问题。 |
| 帧率很低 | 检查状态栏渲染器是否是 NVIDIA。llvmpipe 为 CPU 软件渲染；关闭其他 GPU 密集程序，并查看报告的真实 FPS、P95 和物理实时率。 |
| 物体不动但相机可移动 | 检查 Mode 和 Running。打开模型、编辑物体、保存项目、重置会暂停；相机移动保持当前状态。 |
| 导入失败 | 查看界面中的路径与错误；检查网格/纹理、包映射、惯性、原生插件。失败时当前场景仍可用。 |
| 保存后姿态不是预期状态 | 保存前先 Use pose as initial。项目不保存速度、接触求解器或控制调度队列。 |
| 算法图像与真值错位 | 按完整 FrameStamp 关联，不按队列读取顺序匹配；检查 dropped 和 run_id。 |
| 控制没有生效 | 检查执行器能力列表、run_id/model_revision、目标步和单调时钟截止时间；查看控制结果订阅。 |

复杂模型可能达不到默认性能目标。请保留原分辨率测量并记录实际结果，不能把目标 FPS 设置值当成测量值。
