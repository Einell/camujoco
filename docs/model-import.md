# 模型打开与追加

桌面顶部的 Open model 打开 MJCF/XML 或 URDF，替换整个场景；Append model 将模型追加到当前场景，并弹出位置（米）及固定轴 XYZ 欧拉角（度）输入。旋转顺序对应 Rz(yaw) Ry(pitch) Rx(roll)。

导入成功后暂停仿真并回到模型初态，同时更新模型版本和运行标识。追加也会重置已有物体的动态状态，不保留导入前的仿真进度。失败会显示文件路径和原因，当前场景保持可用。启动参数中的模型无法解析时回退到暂停的示例场景，可以重新打开。

```bash
./run.sh --model examples/models/table.xml
```

随后追加 examples/models/simple_robot.urdf，设置 Z=0.75，即可将机器人放在桌面上。运行复选框控制暂停/继续；Reset 恢复初态。对象位姿和相机参数可通过左侧属性面板编辑，见 [编辑操作](scene-editor.md)；项目保存将在后续步骤实现。

## 资源与模型语义

- 支持本地相对路径、绝对路径及 package:// 包映射。Map package 选择包目录并输入包名；命令行也可传入 `--package robot_description=/absolute/package/path`。无需安装 ROS。
- MJCF 的 include、资源目录和内部引用由 MuJoCo MjSpec 解析。重复追加时为物体、关节、资源和引用分配唯一的 model_0001/ 等前缀。
- 打开场景采用源场景全局设置；追加保留当前场景全局物理和渲染设置，对不同的物理 option 字段给出提示。
- URDF 保留 visual、collision 和固定 link 身份。transmission/Gazebo 扩展不会自动转换为控制器，也不会自动创建 actuator。Xacro 需先展开；网格类型及其他扩展受 MuJoCo 支持范围限制。
- 原始模型文件不被修改。缺失资源、无效惯性、插件依赖及编译错误会作为导入错误显示。
- 优先预览源模型前三台有名称的透视相机；不足三台时自动添加预览相机。追加保持原有预览相机，其他导入相机仍留在模型中。

## Python 接口

```python
from camera_lab.project.importer import SceneProject
from camera_lab.render.worker import RenderSession

project = SceneProject.open('examples/models/table.xml')
project = project.append('examples/models/simple_robot.urdf', position=(0, 0, .75))
with RenderSession(project=project) as renderer:
    observation, truth = renderer.capture_sample()
```

追加返回新项目，失败不修改原项目。四元数参数采用 wxyz 顺序。图像及真值接口见 [数据契约](camera-data-api.md)。
