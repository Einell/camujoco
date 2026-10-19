# 项目保存与恢复

顶部 Save project 保存为 `.camproj`，Open project 重新打开。保存会暂停仿真；打开后进入 edit，仿真时间与速度清零。

保存的是编辑初态。如果要把运行中的当前姿态保留下来，请先点击 Use pose as initial，再保存项目。该格式不保存动态求解器检查点。

项目包包含：

- `project.json`：schema_version、物体编辑、关节/mocap 初态、三台预览相机及标定/挂载、全局观察视角、RGB/深度选择和窗口布局。
- `scene.xml`：规范化基础 MJCF，已展开 include；编辑覆盖保存在 project.json，单独打开该 XML 不等于恢复完整项目。
- `assets/`：网格、纹理等文件资源的副本，按内容哈希命名以避免不同来源的同名资源冲突。

`.camproj` 是 ZIP 容器，可以直接复制到另一个目录或机器。重新打开不依赖原模型目录或 package:// 映射。目标机器仍需兼容的 MuJoCo 和模型所需原生插件；插件二进制不打包。保存时源资源必须仍可读取，失败会显示具体原因。

保存先在目标目录写临时文件，重新加载校验后使用原子替换，写入/替换失败保留已有项目。源 MJCF/URDF 不被覆盖。未知 schema 或缺失资源、无效配置会被拒绝；桌面打开失败保留当前有效场景。

浮动视图、属性面板停靠状态及窗口大小可恢复。新增项目的运行标识与先前会话不同；模型/对象名称保持稳定。

## 示例

打开 `examples/projects/table-robot.camproj`，可看到桌面上的简单机器人和三台相机。修改参数后另存为自己的项目。

```python
from camera_lab.project.store import save_project, load_project
from camera_lab.project.importer import SceneProject
from camera_lab.render.worker import RenderSession

with RenderSession(project=SceneProject.demo()) as renderer:
    save_project(renderer, 'experiment.camproj')

project, layout = load_project('experiment.camproj')
with RenderSession(project=project) as renderer:
    observation, ground_truth = renderer.capture_sample()
```

当前 schema_version 为 1。源 XML 的注释、include 排版不保留；MuJoCo XML 序列化数值存在文本精度舍入，跨项目比对应使用合理浮点容差。实验运行时完整内存快照见 [仿真状态文档](simulation.md)。
