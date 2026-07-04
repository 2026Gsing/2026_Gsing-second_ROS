#!/usr/bin/env python3
"""
PCD 全局坐标系 X 方向过滤
读取 PCD 文件，只保留 x > 0 的点（地图全局坐标系），覆盖保存原文件。

用法：
  python3 filter_pcd_x.py <pcd_path> [x_min]

示例：
  python3 filter_pcd_x.py scans.pcd          # x > 0
  python3 filter_pcd_x.py scans.pcd 0.5      # x > 0.5
"""

import numpy as np
import sys

try:
    import open3d as o3d
except ImportError:
    o3d = None


def filter_pcd_open3d(pcd_path: str, x_min: float = 0.0):
    """使用 open3d 读取/写入 PCD"""
    pcd = o3d.io.read_point_cloud(pcd_path)
    points = np.asarray(pcd.points)
    if len(points) == 0:
        print(f"  点云为空，跳过")
        return

    mask = points[:, 0] > x_min
    kept = mask.sum()
    if kept == 0:
        print(f"  过滤后点云为空！({len(points)} → 0)")
        return

    pcd.points = o3d.utility.Vector3dVector(points[mask])
    if pcd.colors:
        pcd.colors = o3d.utility.Vector3dVector(np.asarray(pcd.colors)[mask])
    o3d.io.write_point_cloud(pcd_path, pcd)
    print(f"  过滤: {len(points)} → {kept} 点 (x > {x_min})")


def filter_pcd_raw(pcd_path: str, x_min: float = 0.0):
    """不依赖 open3d，直接解析 ASCII PCD 文件"""
    with open(pcd_path, 'r') as f:
        lines = f.readlines()

    # 找到数据起始行
    header_end = 0
    data_mode = 'ascii'
    for i, line in enumerate(lines):
        if line.startswith('DATA '):
            data_mode = line.strip().split()[1].lower()
            header_end = i + 1
            break

    if data_mode != 'ascii':
        print(f"  不支持的 PCD 数据格式: {data_mode}，尝试用 open3d")
        if o3d:
            filter_pcd_open3d(pcd_path, x_min)
        else:
            print("  open3d 未安装，无法处理二进制 PCD")
        return

    header = lines[:header_end]
    data_lines = lines[header_end:]

    # 找出 x 字段的索引
    fields_line = next((l for l in header if l.startswith('FIELDS')), None)
    if not fields_line:
        print("  无法解析 PCD FIELDS")
        return

    fields = fields_line.strip().split()[1:]  # ['x', 'y', 'z'] or ['x', 'y', 'z', ...]
    try:
        x_idx = fields.index('x')
    except ValueError:
        print("  PCD 中没有 x 字段")
        return

    # 过滤
    kept_lines = []
    for line in data_lines:
        line = line.strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) > x_idx and float(parts[x_idx]) > x_min:
            kept_lines.append(line)

    # 更新 POINTS 计数
    new_header = []
    for line in header:
        if line.startswith('POINTS '):
            new_header.append(f'POINTS {len(kept_lines)}\n')
        else:
            new_header.append(line)

    with open(pcd_path, 'w') as f:
        f.writelines(new_header)
        for line in kept_lines:
            f.write(line + '\n')

    print(f"  过滤: {len(data_lines)} → {len(kept_lines)} 点 (x > {x_min})")


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    pcd_path = sys.argv[1]
    x_min = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0

    print(f"过滤 PCD: {pcd_path}")
    if o3d:
        filter_pcd_open3d(pcd_path, x_min)
    else:
        filter_pcd_raw(pcd_path, x_min)


if __name__ == '__main__':
    main()
