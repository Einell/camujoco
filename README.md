# MuJoCo Camera Lab

面向 Ubuntu 的 MuJoCo 场景与多相机桌面工具。支持 MJCF/URDF 打开与追加、物体与相机编辑、三路 RGB/深度预览、静态/动态切换，以及自包含项目保存。默认三路各 1280×720，目标 30 FPS。

通过 `from camera_lab import CameraLab` 可不启动界面直接导入场景、配置相机、步进和采样；详见 [Python SDK](docs/python-sdk.md)。核心安装不需要 Qt，桌面为可选组件。本轮重构的回归结果见 [SDK 验证报告](reports/python-sdk-verification.md)。

Python 进程内数据接口提供同步观测、独立真值、按帧位姿误差评估及带有效期的执行器控制。真实视觉算法、视觉定位与机器人策略在后续阶段接入。

从 [使用指南](docs/user-guide.md) 开始；已有场景可直接打开 `examples/projects/table-robot.camproj`。测量结果与未验证范围见 [第一阶段验收](reports/acceptance.md)。

## 当前工作区直接启动

所需依赖已下载到项目内的 .runtime/ 目录，可在本机终端执行：

```bash
./run.sh
```

此启动器会自动使用项目内的 Qt 依赖和 xcb-cursor 库；将项目复制到其他机器后按下面方式安装。

## 安装与运行

在 Ubuntu 22.04 或更新版本、Python 3.10+ 环境中：

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e '.[desktop,test]'
MUJOCO_GL=egl camera-lab
```

若创建虚拟环境提示缺少 ensurepip，安装 Ubuntu 的 python3-venv 包。若 Qt 提示 xcb 依赖缺失，按错误信息安装对应系统库；不要通过安装另一套 NVIDIA 驱动解决 Qt 库问题。

默认 1280×720、30 FPS，所有相机同时生成 RGB 和深度；勾选深度只改变预览，不改变输出。深度预览固定显示 0–5 米，黑色表示无有效深度。全局视图左键拖动旋转，滚轮缩放；其变化不影响三台相机。

## 性能验证

检查 `nvidia-smi`，并确认最终报告的 gl_renderer 为 NVIDIA，而不是 llvmpipe。

后台测试会渲染全局画面，但不显示窗口：

```bash
MUJOCO_GL=egl camera-lab-benchmark --warmup 10 --seconds 60 --report reports/headless.json
```

包含实际桌面显示的测试：

```bash
MUJOCO_GL=egl camera-lab --warmup 10 --seconds 60 --report reports/desktop.json --screenshot reports/desktop.png
```

结果按每路相机计帧率，不将三路帧数相加。桌面报告以提交给 UI 的完整批次数计数，不是显示器扫描帧率；P95 是每批四个视角加三路深度的渲染耗时，不是端到端算法延迟。

报告记录环境、真实渲染器、时间、帧率、P95、CPU 时间和进程内存；后台报告额外采样 NVIDIA GPU 利用率和显存。GPU 数据是设备整体数据，可能包含其他桌面程序。

## 测试

```bash
MUJOCO_GL=egl QT_QPA_PLATFORM=offscreen pytest -q
```

测试实际调用 MuJoCo 渲染，不用假图像替代。没有 GPU 的机器可能通过 Mesa 软件渲染完成正确性检查；这不代表 GPU 性能合格。offscreen Qt 测试不代表真实桌面显示验收。

## 实现边界

本版本的仿真与渲染都在同一个工作线程，保证一批图像来自同一状态。UI 只接收不可变批次，使用容量为一的最新帧邮箱，慢 UI 不造成无限内存积压。当前已有内存快照和版本化接口；是否拆分物理服务或增加跨进程通信，留待后续算法与策略集成需求明确后决定。

基准场景包含地面、桌面、三个彩色刚体和受控摆杆；其性能不能直接外推到任意复杂机器人场景。

## 相机数据接口

详细字段、坐标约定与配置方法见 [相机观测与真实位姿接口](docs/camera-data-api.md)。运行 `python examples/capture_calibrated.py` 可导出三路同步样本。

## 复现综合验收

在仓库根目录、已安装依赖的环境中执行：

```bash
MUJOCO_GL=egl python benchmarks/desktop_acceptance.py --benchmark --seconds 60 --report reports/robot-desktop.json
MUJOCO_GL=egl python benchmarks/desktop_acceptance.py --seconds 1800 --reload-every 60 --report reports/soak.json
```

第一条对随仓库提供的简单机器人项目测量；第二条显示三路 720p，连续运行 30 分钟并周期重新打开项目，记录 RSS、GPU 显存和运行进度。专用测试窗口禁用编辑输入；提前关闭会判为未完成。吞吐测试应与其他渲染测试分开执行。提前关闭窗口、加载失败或进度停止不算验收通过。

本机使用 `.runtime` 时，先设置 `PYTHONPATH=src:.runtime/python-deps`、`LD_LIBRARY_PATH=.runtime/system-libs/usr/lib/x86_64-linux-gnu`；新机器推荐使用上面的虚拟环境安装方式。
