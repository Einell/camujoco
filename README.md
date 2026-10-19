# MuJoCo Camera Lab

Ubuntu 本地桌面多相机平台的第一个交付物：可运行的渲染验证程序。

当前提供内置动态场景、独立全局视图、三路 1280×720 RGB/深度输出、暂停/继续、重置、全局视角鼠标旋转缩放，以及后台和桌面基准。现在也提供 Python 相机标定、独立分辨率/裁剪设置及带采样标识的 RGB/深度/真值接口。模型导入、完整编辑器、桌面相机参数面板、项目保存和外部算法订阅服务尚未实现。

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
python -m pip install -e '.[test]'
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

本版本的仿真与渲染都在同一个工作线程，保证一批图像来自同一状态。UI 只接收不可变批次，使用容量为一的最新帧邮箱，慢 UI 不造成无限内存积压。后续正式平台会将物理仿真拆成独立服务，并引入完整快照和版本化接口。

基准场景包含地面、桌面、三个彩色刚体和受控摆杆；其性能不能直接外推到任意复杂机器人场景。

## 相机数据接口

详细字段、坐标约定与配置方法见 [相机观测与真实位姿接口](docs/camera-data-api.md)。运行 `python examples/capture_calibrated.py` 可导出三路同步样本。
