# Python SDK 设计

用户已批准 Python 进程内 SDK，并要求保持原功能不变、修改后测试。

公开入口 `from camera_lab import CameraLab`。构造时启动唯一 MuJoCo 所有者线程，默认暂停且不持续渲染；支持上下文管理器。桌面默认行为不变，仍持续渲染、演示场景启动即运行，导入失败可回退示例。SDK 指定无效启动模型时直接抛出异常并清理线程。

从 app.py 提取 Worker 到无 Qt 的 runtime.py，桌面重新导入同一个 Worker。SDK 同步命令使用有界队列与 Future，在该线程内执行，与桌面共享原有导入、编辑、保存和状态切换路径。命令成功后返回；失败抛出原异常。调用无隐式超时，避免超时后命令仍修改场景；close 阻止新请求，等待当前操作与资源清理，未执行的请求明确失败。公开方法不能在所有者线程回调中阻塞调用。

CameraLab 提供 run/pause/step/reset/save_initial/capture/capture_with_truth/save_project/load_project/close；scene 提供 load/append/set_pose/describe；cameras 提供 configure/aim/move/describe。相机 configure 支持 CameraConfig 字段的部分更新及 parent/position/rotation。所有对象描述均为副本，不暴露可变 MuJoCo model/data。

SDK 默认 step 精确推进正整数个模型 timestep 后保持暂停；capture 不推进物理。run 启动墙钟实时推进与有界订阅，pause 停止推进。capture 返回 ObservationBatch；显式 capture_with_truth 返回同一 FrameStamp 的观测与真值。data 属性复用 DataAPI 的订阅、估计评估及控制接口。算法代码由调用方执行，不进入渲染线程。

核心安装依赖仅 MuJoCo 和 NumPy；Qt 放入 desktop extra，测试 extra 含 Qt 以继续执行原桌面测试。既有低层导入和 API 保持可用。无界面仍需 EGL/GLFW/OSMesa 渲染环境。本次不实现网络、ROS 或真实算法。
