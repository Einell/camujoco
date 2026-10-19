# Python SDK 0.2.0 验证

2026-10-19，分支 feat/python-sdk，基于已验收版本 b3e7639。用户要求保持原桌面功能，增加可独立调用的模块化 Python SDK，并完成测试。

## 变更

- 将 app.py 中原 Worker 提取到无 Qt 的 runtime.py；桌面仍使用同一 Worker，保留原 UI 命令、事件和模型导入失败回退。
- CameraLab 提供上下文管理、精确步进、实时运行、捕获、项目保存恢复；scene/cameras 提供分组方法，data 复用原 DataAPI。
- SDK 同步请求通过有界队列交给唯一 MuJoCo 所有者线程执行；失败传回调用者，关闭时拒绝尚未执行的命令。
- Qt 改为 desktop/test 可选依赖。只导入数据契约不会加载 MuJoCo/Qt。
- 原 RenderSession、DataAPI 导入及 app.Worker 导入继续可用。桌面相机滚轮保持运行状态、物体编辑暂停重置等行为不变。

## 测试结果

| 环境 | 结果 |
| --- | --- |
| 软件 EGL、Qt offscreen，全套 | 87 passed，43.94 秒，无跳过 |
| NVIDIA GPU、Qt offscreen，全套 | 87 passed，12.33 秒，无跳过 |
| Python 路径确实未安装 PySide6，SDK 测试 | 11 passed，8.93 秒 |
| 独立安装 wheel，不加载源码且无 PySide6 | 三相机实际渲染，精确推进 10 步，观测与真值 stamp 匹配 |
| 示例 examples/sdk_capture.py | 无 Qt 输出三路 RGB/深度/掩码和标定文件 |
| 独立代码审查 | 两个边界问题复现并修复后复核通过 |

原来的 76 项测试全部保留并通过。新增测试覆盖精确手动步进、实时订阅与暂停、场景/相机/项目往返、错误后恢复、控制与位姿误差评估、并发关闭、启动失败清理、运行时错误传播、无 Qt 以及契约独立导入。

审查发现的两个问题均先用测试复现失败，再修复：仅改标定保留源相机的跟踪模式；close 后未执行的排队保存请求被拒绝且不会写文件。

## 真实桌面性能回归

预热 10 秒后测量 60 秒，独立运行三路 1280×720 RGB/深度和 GLOBAL 视图，测试窗口禁用用户输入以保持固定条件。

- 每路实际显示 29.87 FPS。
- 渲染批次 P95：21.97 ms。
- 物理实时率：0.999。
- UI 丢批：0；工作线程无错误。

[原始报告](sdk-desktop-60s.json)、[恢复正常交互后的桌面截图](sdk-desktop.png)。达到原本约 30 FPS 与渲染 P95 ≤ 33.3 ms 的本机目标；P95 是渲染批次耗时，不是算法端到端延迟。

## 打包

`mujoco_camera_lab-0.2.0-py3-none-any.whl` 构建成功。核心依赖仅 MuJoCo 3.6.0、NumPy 2.2.6；desktop extra 包含 PySide6 Essentials 6.8.3。安装包包含 SDK、共享 runtime 和 demo.xml。

SHA256：`061300d302654f4628b403945431a1371a230abc6f9d2a8b05704603506d53fb`。

## 范围

本轮不添加网络服务、ROS 2、真实视觉算法或机器人策略。无界面仍需可工作的 OpenGL 后端。实际硬件为 RTX 4060 Laptop；未据此宣称 RTX 5060 或任意复杂模型达标。此前 30 分钟记录属于旧版验收，本轮不把它当成新代码的长测结果。
