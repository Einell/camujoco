# 仿真状态与初态

界面显示当前 Mode：edit（编辑初态）、paused（暂停当前动态状态）、running（运行）。所有物理与渲染操作仍由同一个工作线程处理，暂停时三路相机继续输出。

| 操作 | 结果 |
| --- | --- |
| Running / Pause | 开始或暂停；继续运行保持当前状态和时间 |
| Single step | 推进一个模型 timestep，随后保持 paused |
| Reset initial | 恢复最近设置的初态，清零时间，进入 edit |
| Return to edit | 与 Reset initial 相同，放弃本次动态演化回到初态 |
| Use pose as initial | 将当前关节配置（包括自由物体）及 mocap 姿态设为初态，清零时间、速度、激活与控制输入，进入 edit |

保存初态不改变模型关节的物理参考配置。物体静态编辑仍修改对应物体的初态；追加模型会保留已有对象的初态。打开另一个场景则使用新场景初态。相机参数、挂载和全局观察视角在重置时保留。

暂停和编辑状态下不会推进重力或运动；可以继续调整相机。相机操作保持当前运行/暂停状态，运行中调整相机不会中断物理仿真。物体位姿编辑会重置整个仿真到编辑后的初态。

Reset、Return to edit、Use pose as initial 及快照恢复都会生成新 run_id；保存初态还更新 model_revision。单步增加 step_index，图像与真值仍使用同一采样标识。

## Python 状态接口

```python
from camera_lab.render.worker import RenderSession
from camera_lab.sim.engine import Simulation

with RenderSession() as renderer:
    simulation = Simulation(renderer)
    simulation.transition('step')
    observation, truth = renderer.capture_sample()
    simulation.transition('save_initial')
    simulation.transition('run')
    simulation.advance(0.1)
    simulation.transition('pause')
```

可为 transition 传入 expected_run_id；运行标识改变后带旧标识的命令会被拒绝。桌面本地按钮使用同一线程 FIFO，外部执行器控制入口已使用强制版本校验，见 [控制接口](data-control-api.md)。

## 内存快照

renderer.snapshot() 按需捕获 MuJoCo 的 mjSTATE_INTEGRATION（包含积分所需状态、控制、外力、mocap 等），不会每帧复制。renderer.restore_snapshot(snapshot) 验证运行和模型版本后恢复并生成新 run_id；同一旧快照不能在重置、换模型或恢复后再次使用。只支持创建它的会话及模型版本。

快照与“保存当前姿态为初态”用途不同：快照保留动态积分状态，初态用于从静止重新开始。编辑初态可通过项目保存持久化；完整动态快照仍仅在内存中，不保证跨 MuJoCo 版本的二进制快照兼容。
