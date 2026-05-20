#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <limits>
#include <cmath>

// 简单二维栅格地图生成：投影到 xy 平面
int main(int argc, char** argv) {
    if (argc < 5) {
        std::cerr << "Usage: pcd_to_gridmap <input.pcd> <output.pgm> <resolution> <size>\n";
        std::cerr << "  <resolution>: 每个格子的边长（米）\n";
        std::cerr << "  <size>: 地图边长（米），地图为 size x size\n";
        return 1;
    }
    std::string infile = argv[1];
    std::string outfile = argv[2];
    double resolution = atof(argv[3]);
    double size = atof(argv[4]);
    if (resolution <= 0 || size <= 0) {
        std::cerr << "参数必须为正数" << std::endl;
        return 1;
    }
    int grid_size = static_cast<int>(std::ceil(size / resolution));

    // 读取点云
    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(infile, cloud) == -1) {
        std::cerr << "无法读取PCD文件: " << infile << std::endl;
        return 1;
    }

    // 计算点云中心
    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double max_y = std::numeric_limits<double>::lowest();
    for (const auto& p : cloud.points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y)) continue;
        if (p.x < min_x) min_x = p.x;
        if (p.x > max_x) max_x = p.x;
        if (p.y < min_y) min_y = p.y;
        if (p.y > max_y) max_y = p.y;
    }
    double cx = (min_x + max_x) / 2.0;
    double cy = (min_y + max_y) / 2.0;

    // 初始化栅格地图
    std::vector<std::vector<int>> grid(grid_size, std::vector<int>(grid_size, 0));

    // 投影点云到 xy 平面并计数
    for (const auto& p : cloud.points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        // 只保留高度在 0~1.0 米之间的点
        if (p.z < 0 || p.z > 1.0) continue;
        int ix = static_cast<int>(std::floor((p.x - cx + size/2) / resolution));
        int iy = static_cast<int>(std::floor((p.y - cy + size/2) / resolution));
        if (ix >= 0 && ix < grid_size && iy >= 0 && iy < grid_size) {
            grid[iy][ix]++;
        }
    }

    // 输出为 PGM 灰度图（0=空，255=占据）
    std::ofstream ofs(outfile, std::ios::out);
    ofs << "P2\n" << grid_size << " " << grid_size << "\n255\n";
    for (int y = 0; y < grid_size; ++y) {
        for (int x = 0; x < grid_size; ++x) {
            int val = grid[y][x] > 0 ? 0 : 255; // 占据为黑，空白为白
            ofs << val << " ";
        }
        ofs << "\n";
    }
    ofs.close();
    std::cout << "栅格地图已保存为: " << outfile << std::endl;
    return 0;
}