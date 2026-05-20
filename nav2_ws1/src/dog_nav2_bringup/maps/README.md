把你的 2D 栅格地图文件放在这里，例如：

- `map.yaml`
- `map.pgm`（或 `map.png`）

然后启动：

```bash
ros2 launch dog_nav2_bringup nav2_fastlio_static_map.launch.py map:=/abs/path/to/map.yaml
```

**RViz 中想看 Global/Local Costmap：** 当前配置未加载 Nav2 的 Costmap 插件（避免“No map received”报错）。若要显示 costmap，请安装插件后手动添加：
```bash
sudo apt install ros-humble-nav2-rviz-plugins
```
然后在 RViz 里 Add -> By display type -> nav2_rviz_plugins/Costmap，Topic 选 `/global_costmap/costmap` 或 `/local_costmap/costmap`。

