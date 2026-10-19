# 数据、评估与控制接口

当前提供同一 Python 进程内的线程接口，不包含网络服务、ROS 2、真实视觉算法或机器人策略。

## 观测和真值订阅

```python
from camera_lab.api import DataAPI
from camera_lab.app import Window

api = DataAPI()
images = api.subscribe_observations()
truth = api.subscribe_truth()  # 只交给评估消费者
window = Window(api=api)
# 在算法线程中调用 images.get_nowait()；无数据时抛出 queue.Empty。
```

图像和真值使用独立订阅，每个订阅最多两批。队满时丢弃最旧的完整批次，dropped 累计丢批数；慢消费者不会在渲染线程执行算法。每种流最多 64 个订阅，调用 subscription.close() 可关闭；窗口退出时 api.close() 统一关闭订阅。算法不能在 Qt 主线程长时间阻塞。

每批三路相机的 stamp 相同，单独真值流按完整 stamp 关联。不要依赖两个队列取出的顺序始终一致：消费者可能分别丢帧。重置或换模型会清除旧版本的队列与评估缓存。只保留图像消费所需权限时，不向算法传入真值订阅。

直接使用 RenderSession 时由生产线程调用 `api.bind(renderer)`、`api.drain_controls(renderer)` 和 `api.publish(*renderer.capture_sample())`。桌面 Worker 自动完成这些操作。

## 位姿评估

PoseEstimate 包含原始 FrameStamp、camera_id、object_id、光学坐标下的 T_camera_object、confidence 和 latency_ms。变换必须为有限的刚体变换，单位为米。

`api.evaluate(estimate)` 返回 PoseError：

- matched：同一帧匹配成功，translation_m 为平移欧氏距离，rotation_rad 为 SO(3) 旋转角误差。
- unavailable：帧已过期、运行已改变或未发布；不会用当前物体位姿替代旧真值。
- unknown_target：匹配帧中没有所指定的相机或物体。

默认缓存 120 批的相机/物体变换，不缓存历史图像；可用 DataAPI(history_size=...) 调整。对称物体的特殊评价尚未实现。

## 控制接口

`api.actuators` 提供不可变的执行器名称、ctrlrange、默认值和语义。只有具名执行器可通过此接口控制；纯 URDF 没有 actuator 时，命令会明确拒绝。

ControlCommand 必须指定：

- run_id / model_revision：与当前场景一致。
- target_step：在该 step_index 对应的下一次积分前生效；允许未来步，不接受已经过去的目标步。
- valid_until_step：最后可生效的积分前步号，包含此边界。
- expires_at_ns：`time.monotonic_ns()` 时间域的绝对截止时刻，不能使用 Unix 时间戳。
- values：执行器名称到有限数值的映射。
- semantics：目前只支持 native_ctrl，直接对应 MuJoCo actuator 的 ctrl；并不统一解释为关节角、速度或力矩。

`api.submit_control(command)` 将请求放入容量 64 的线程安全队列；满时抛出 queue.Full。执行器、范围、版本和期限在工作线程校验。`api.subscribe_control_results()` 接收 ControlResult，accepted 表示已进入调度，不保证在截止前实际执行；结果流同样容量两批，消费者应检查 dropped。

目标步相同且作用于同一执行器时，后提交命令覆盖先提交命令。未来调度队列上限 64；每物理步更新，不以图像帧率充当控制频率。请为图像读回及线程调度留足未来步余量，例如当前观测 step_index+30。

第一次有效命令接管内置示例控制后，自动摆动停止。未指定或已超时的执行器恢复默认值：0；若受限 ctrlrange 不含 0，则使用最接近 0 的合法边界。可通过 api.actuators 查询。墙钟期限在暂停时也检查；重置、模型改变后旧调度清空。恢复快照后旧命令不会继续使用。关闭窗口停止仿真。

控制命令、订阅和历史缓存不保存到项目文件。新打开会话需要重新发送控制命令。外部算法若需要跨进程接口，后续通过适配器扩展。

## 可运行示例

先按 README 安装依赖。当前本机本地依赖模式可设置：

```bash
export PYTHONPATH=src:.runtime/python-deps
export LD_LIBRARY_PATH=.runtime/system-libs/usr/lib/x86_64-linux-gnu
export MUJOCO_GL=egl
python3 examples/consume_frames.py
python3 examples/control_pendulum.py
python3 examples/evaluate_pose.py
```

前两个示例打开桌面约 6 秒后退出，分别输出消费/丢批统计与命令接收统计。位姿示例以真值合成 1 厘米误差，仅验证接口与坐标计算，不代表真实算法精度。
