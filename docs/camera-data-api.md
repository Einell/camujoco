# 相机观测与真实位姿接口

相机参数、RGB、深度和位姿在同一采样状态中发布。此接口属于计划任务 2；桌面参数面板与外部算法订阅服务分别在后面的界面、数据流任务实现。

## 使用方法

```python
from dataclasses import replace
from camera_lab.render.worker import RenderSession

with RenderSession() as session:
    config = replace(session.camera_config('front'), width=640, height=480,
                     fx=520, fy=510, cx=315, cy=237, near=.05, far=10)
    session.configure_camera('front', config)
    session.advance_to(.2)
    observations, truth = session.capture_sample()
```

`RenderSession` 必须在同一线程创建、调用和关闭。`configure_camera` 返回后，下一批图像使用新配置；尺寸、K、裁剪范围与 camera_revision 同时生效。配置改变不推进物理时间。无效参数在创建 CameraConfig 时拒绝；未知相机在配置入口拒绝。

`capture()` 仅返回观测批次；`capture_sample()` 返回观测和单独的真值批次，便于未来把真值只交给评估器。

## 数据字段

- `CameraConfig`：width、height、fx、fy、cx、cy、near、far。焦距和主点使用像素单位，裁剪距离使用米。
- `FrameStamp`：run_id、model_revision、batch_id、step_index、sim_time_s、host_monotonic_ns。整批三路相机和真值共享同一个标识。
- `CameraFrame`：stamp、camera_id、camera_revision、config、RGB、depth、valid_mask、K、T_world_camera。实际字段名为小写 `rgb`。
- `ObservationBatch`：stamp、overview、frames、render_ms。兼容基线的 sequence 和 sim_time 属性。
- `GroundTruthBatch`：stamp、object_poses；使用 body 名称作为对象标识，匿名 body 使用自动生成的标识。对象坐标是 body 坐标，不是 geom 或质心坐标。完整模型导入后的持久对象 ID 由下一阶段管理。

图像和矩阵是独立快照，以 NumPy 只读标志发布；后续采样不复用其内存。真值映射不可直接修改。不要修改 NumPy 写保护标志。

每次 reset 生成新 run_id，batch_id 和 step_index 归零。相机配置版本保留；model_revision 在本阶段为 0，因为还不支持运行中的模型替换。不要单独用 batch_id 关联跨运行的数据。

## 坐标与深度

`T_A_B` 将 B 中的点映射到 A。世界坐标继承模型；相机光学坐标为 x 向右、y 向下、z 向前。T_world_camera 由实际采样时的相机世界位姿计算，包含相机父 body 的运动。

像素中心从左上角 `(0,0)` 起算；默认中心主点是 `((width-1)/2,(height-1)/2)`。投影关系为 `u=fx*X/Z+cx`、`v=fy*Y/Z+cy`。MuJoCo 主点字段与此定义不同，适配层完成方向和半像素换算，并用实际偏心标记点渲染测试验证。[MuJoCo 3.6 相机定义](https://mujoco.readthedocs.io/en/3.6.0/XMLreference.html#body-camera)

RGB 为 uint8[H,W,3]；depth 为 float32[H,W] 的光学轴 Z 深度，单位米。未命中背景和裁剪外像素为 NaN，valid_mask 为 bool[H,W]。受深度缓冲数值精度影响，远裁剪面前最后约百万分之一的距离也按无效处理。

第一版仅支持理想透视针孔模型。读取正交相机将明确报错，不输出误导性的针孔 K。RGB 和深度来自同一个虚拟相机，不模拟真实 RGB-D 相机的双传感器基线。

## 导出可检查的样本

安装项目后运行：

```bash
MUJOCO_GL=egl python examples/capture_calibrated.py --output captures/sample.npz
```

结果包含三路 RGB、深度、掩码、K、外参，以及不依赖 pickle 的 JSON 元数据和物体真值。可通过 `numpy.load('captures/sample.npz', allow_pickle=False)` 读取。
