# 仿真状态验证

2026-10-19。

- 软件 EGL + Qt offscreen 全套：53 项通过（最终重试）。
- 新增测试覆盖单步与同步真值、保存初态清零动态状态、初态复制/重置、状态转换、旧运行命令拒绝、完整积分快照恢复及跨会话/重复恢复拒绝、桌面按钮完整切换。
- 使用 320×180 软件渲染执行 offscreen 界面检查；见 simulation-offscreen.json 与 simulation-offscreen.png。这不是 GPU 性能验收。
- git diff --check 通过。
- 重试后真实 NVIDIA EGL 全套：53 passed in 4.42s。
- 真实桌面：三路 1280×720 RGB+深度，预热 1 秒、测量 5 秒，每路 29.39 FPS，渲染 P95 22.21 ms，UI 丢帧 0。GPU 为 RTX 4060 Laptop；这是短时检查，不替代目标 RTX 5060 的完整验收。
- 桌面报告和截图见 simulation-desktop.json、simulation-desktop.png。


- 独立复核仿真状态与快照：7 passed in 2.13s，无遗留阻断问题。
- mocap 物体编辑同步初态，覆盖编辑、保存、重置与复制后真值一致；Qt 退出测试同时接受窗口已隐藏或已销毁。
