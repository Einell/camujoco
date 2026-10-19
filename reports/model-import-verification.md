# 模型导入验证

日期：2026-10-19。

- 软件 EGL 全套：38 passed in 10.15s。
- NVIDIA EGL 全套：38 passed in 2.41s。
- 独立代码复核：导入/UI 14 passed，无剩余阻断问题。
- 真实桌面打开 simple_robot.urdf，三路 1280×720 RGB+深度和独立全局视图；预热 1 秒、测量 5 秒，每路 29.73 FPS，批次渲染 P95 22.52 ms，UI 丢帧 0。
- GPU：RTX 4060 Laptop。导入后暂停，仿真时间为零；本次为短时导入渲染检查，不代表 RTX 5060、复杂机器人或动态策略性能验收。

桌面报告见 model-import-desktop.json，截图见 model-import.png。覆盖打开、重复追加、位置/旋转、相对资源/include、package 映射、执行器引用、错误后保留场景和启动错误后重试。
