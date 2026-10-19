# 项目保存与恢复验证

2026-10-19。

- 软件 EGL 全套：61 passed in 25.05s。
- NVIDIA EGL 全套：61 passed in 6.64s。
- 独立复核存储/导入：18 passed in 3.88s，无剩余阻断问题。
- 资源迁移覆盖：删除原文件、换目录、再次保存、纹理、不同来源同名网格、缺失/非法相机引用、未知 schema、失败替换保留旧项目；修复 strippath 导入兼容性。
- 桌面完整打开 examples/projects/table-robot.camproj，恢复机器人、三路相机及 edit 初态，生成截图 project-storage.png。
- 该示例短测三路720p RGB+深度：呈现 28.17 FPS、生产29.15 FPS、渲染 P95 35.42ms、UI丢帧5。硬件RTX4060 Laptop，预热1秒测量5秒，暂停仿真。这次结果未达到30FPS/P95≤33.3ms目标，性能优化与长时目标硬件验收仍留综合验收阶段；不降低分辨率冒充达标。
- 重置图像测试保留物理qpos/xpos完全相等断言；针对NVIDIA偶发单像素两级量化差异，RGB最大容差2且平均绝对差<0.01。

使用说明见 docs/project-storage.md。保存文件为初态项目包，不是运行中的积分检查点。
