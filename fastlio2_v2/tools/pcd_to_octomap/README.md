PCD -> OctoMap 工具

要求：
- 系统已安装 `libpcl-dev` 和 `liboctomap-dev`（或相应来源编译的库）。
- CMake 和编译器（g++）

构建：

```bash
sudo apt-get install -y build-essential cmake libpcl-dev liboctomap-dev
mkdir -p build && cd build
cmake ..
make -j
```

使用：

```bash
./pcd_to_octomap /path/to/scans.pcd output.bt 0.05
# 参数：<输入 pcd> <输出 .bt> <分辨率（米）>
```

输出为 OctoMap 二进制 `.bt` 文件，可用 OctoMap 工具或程序加载查看。
