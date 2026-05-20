PCD -> 栅格地图工具

要求：
- 系统已安装 `libpcl-dev`（或相应来源编译的库）。
- CMake 和编译器（g++）

构建：

```bash
sudo apt-get install -y build-essential cmake libpcl-dev
mkdir -p build && cd build
cmake ..
make -j
```

使用：

```bash
./pcd_to_gridmap /path/to/scans.pcd output.pgm 0.05 50
# 参数：<输入 pcd> <输出 pgm> <分辨率（米）> <地图边长（米）>
```

输出为 PGM 灰度图（黑色为占据，白色为空），可用图像工具直接查看。