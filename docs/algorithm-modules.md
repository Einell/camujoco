# Python 算法模块

Qt 只负责桌面界面。算法使用 `camera_lab.modules`，无需导入 Qt，也不直接操作 MuJoCo。

```python
from camera_lab import CameraLab
from camera_lab.modules import FixedPoseEstimator
from queue import Empty
import time

with CameraLab() as lab:
    lab.modules.register('demo', lambda: FixedPoseEstimator(object_id='red_box'))
    results = lab.modules.subscribe_results()
    lab.modules.start('demo')
    lab.run()
    end = time.monotonic() + 5
    while time.monotonic() < end:
        try:
            result = results.get_nowait()
            print(result.stamp, result.estimates, result.errors)
        except Empty:
            time.sleep(.01)
    lab.modules.stop('demo', timeout=2)
```

真实算法实现三个方法，再注册工厂。配置通过工厂闭包传入，重启时创建全新实例：

```python
class MyEstimator:
    def start(self, cancel):
        self.cancel = cancel
        # 在算法线程加载模型。

    def estimate(self, observation):
        # observation.frames 提供各相机 RGB、米制深度、有效掩码、K 和标定。
        # 调用你的模型，返回 PoseEstimate 的有限序列。
        # 输出必须沿用 observation.stamp 和输入 camera_id。
        # T_camera_object 使用光学坐标：x 右、y 下、z 前，平移单位米。
        return []

    def close(self):
        # 释放算法模型、外部连接等资源。
        pass
```

`lab.modules.register('my-model', lambda: MyEstimator())` 即可接入。桌面可使用 `window.modules.register(...)` 注册同样的工厂，选择框会自动刷新，无需改核心或面板代码。

- 算法只接收观测批次，不接收物体真实位姿。这里的隔离是 API 边界，不是恶意插件安全沙箱；观测本身包含相机标定/外参。
- 每个模块独立线程，输入最多积压两批，过载丢弃最旧帧。结果订阅也最多两批，每批最多 256 个估计。
- `status(name)` 返回状态、已处理批次、输入丢帧、耗时和异常。异常只停止对应模块。
- 评估依据完整帧标识匹配真值。历史默认 120 批；重置、换模型或历史淘汰后返回 `unavailable`，目标未知为 `unknown_target`，不使用当前帧替代。
- `stop(name, timeout=0)` 请求取消；返回值表示线程是否已退出，超时后仍显示 `stopping`。适配器必须响应取消，且 `start/estimate/close` 都应有限时完成。Python 线程无法强杀；需要硬隔离时使用子进程包装算法。纯 Python CPU 密集任务仍可能因 GIL 影响响应。
- SDK `close()` 最多等待模块两秒，返回是否全部停止；上下文管理器超时会发出警告。桌面异步等待最多两秒，超时发出警告后关闭。未响应的后台线程不会阻止进程退出，但资源仅在适配器返回后清理。
- `FixedPoseEstimator` 只返回配置的固定姿态，用于验证接口，不是位姿估计算法。默认场景桌面演示目标为 `red_box`。
- 桌面结果面板显示原始批次、run/model 标识及平移/旋转误差；结果是历史输入帧的结果，不叠加到正在变化的实时画面。

机器人动作仍使用现有 `lab.data.submit_control(...)`；本次模块协议针对视觉估计，不强行统一控制策略与估计输出。
