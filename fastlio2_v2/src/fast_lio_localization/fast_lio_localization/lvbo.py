import open3d as o3d
import numpy as np

# 1. 读取点云文件
# 如果PCD文件是二进制格式，使用 read_point_cloud 会自动识别
pcd = o3d.io.read_point_cloud("scans.pcd")  # 替换为你的文件名，可用 fastlio2_v2/PCD/scans.pcd
print(f"原始点云数量: {len(pcd.points)}")

# 2. 执行统计滤波
# nb_neighbors: 分析邻居点数量 (类似PCL的MeanK)
# std_ratio: 标准差倍数 (类似PCL的StddevMulThresh)
# 返回值: (滤波后的点云, 被滤除的点的索引列表)
cl, ind = pcd.remove_statistical_outlier(nb_neighbors=50, std_ratio=1)

# 3. 提取滤波后的点云
filtered_pcd = pcd.select_by_index(ind)  # 保留有效点
outlier_pcd = pcd.select_by_index(ind, invert=True)  # 提取被滤除的噪点(用于可视化)
print(f"滤波后点云数量: {len(filtered_pcd.points)}")
print(f"滤除噪点数量: {len(outlier_pcd.points)}")

# 4. 可视化对比
# 原始点云设为灰色
pcd.paint_uniform_color([0.5, 0.5, 0.5])
# 滤波后点云设为绿色
filtered_pcd.paint_uniform_color([0, 1, 0])
# 噪点设为红色
outlier_pcd.paint_uniform_color([1, 0, 0])

# 同时显示三部分
o3d.visualization.draw_geometries([pcd, filtered_pcd, outlier_pcd],
                                   window_name="统计滤波结果: 灰=原始, 绿=有效, 红=噪点")

# 5. 保存结果
o3d.io.write_point_cloud("output_filtered.pcd", filtered_pcd)
print("结果已保存到: output_filtered.pcd")