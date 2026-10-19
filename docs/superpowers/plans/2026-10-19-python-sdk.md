# Python SDK 实现计划

> 使用 executing-plans 在当前会话逐任务执行；用户已授权修改和测试，不增加审批往返。

**目标：** 保持桌面行为，增加不依赖 Qt 的进程内 Python SDK。
**架构：** 提取原 Worker，新增同步请求机制和 CameraLab 公开入口，共用原有命令路径。
**技术栈：** Python 3.10、MuJoCo 3.6、NumPy、可选 PySide6。
**规格：** ../specs/2026-10-19-python-sdk-design.md

## 任务 1：公开 SDK 行为测试

- [x] 新建 tests/test_sdk.py，测试无 Qt 导入、精确步进/捕获、三相机配置与保存恢复、实时订阅与暂停、加载错误后恢复、关闭后拒绝命令及线程退出。
- [x] 执行 `MUJOCO_GL=egl PYTHONPATH=src:.runtime/python-deps python3 -m pytest tests/test_sdk.py -q`，先观察缺少 CameraLab 的失败。

## 任务 2：共享运行核心与 SDK

- [x] 将 app.py 的 Worker 移到 runtime.py，桌面保持 `from .runtime import Worker`，旧导入兼容。
- [x] 在 Worker 引入 Future 请求，保持原命令行为；增加 SDK capture/describe/configure 与多步支持；所有错误/关闭路径完成或拒绝请求。
- [x] 新建 sdk.py，实现 CameraLab、Scene、Cameras；根包导出 CameraLab 和既有数据契约。
- [x] 执行 SDK 和原桌面测试，再运行完整测试。

## 任务 3：安装与交付

- [x] pyproject 将 Qt 改为 desktop/test extra；README 更新桌面安装方式，新增 docs/python-sdk.md 和无 Qt 示例。
- [x] 子进程阻止 PySide6 导入验证核心，构建 wheel 并检查依赖与模块。
- [x] 在真实 GPU 执行完整测试与 60 秒桌面回归，独立代码审查；修复发现的问题，记录报告，提交并推送。
