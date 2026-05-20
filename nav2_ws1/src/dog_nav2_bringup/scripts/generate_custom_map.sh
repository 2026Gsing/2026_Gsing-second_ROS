#!/bin/bash
# 任务赛场地地图生成脚本（无固定像素限制，超出区域设为不可通行）
# 包含：围墙（不可通行）、存放区（不可通行）、归位区（可通行）、减速带（可通行）、启动区（可通行）

NAV2_DIR=~/program/ROS/nav2_ws1/src/dog_nav2_bringup

# 1. 进入地图目录，清理旧文件
cd $NAV2_DIR/maps/
rm -f task_field_map.pgm task_field_map.yaml

# 2. 生成更大尺寸的基础图（800×1000像素，覆盖足够区域），默认全黑（不可通行）
convert -size 800x1000 xc:black task_field_map.pgm

# 3. 绘制核心赛事场地（6000mm×4000mm → 600×400像素）为白色可通行区
# 核心区域位置：X=100-500像素，Y=200-800像素（居中放置）
convert task_field_map.pgm -fill white \
-draw "rectangle 100,200 499,799" \
task_field_map.pgm

# 4. 绘制外围围墙（黑色，20像素宽，在核心场地边缘）
convert task_field_map.pgm -fill black \
-draw "rectangle 100,200 499,219" \    # 下围墙
-draw "rectangle 100,780 499,799" \    # 上围墙
-draw "rectangle 100,200 119,799" \    # 左围墙
-draw "rectangle 480,200 499,799" \    # 右围墙
task_field_map.pgm

# 5. 绘制物资箱存放区（8个，25×25像素，黑色不可通行）
# 第一行（Y=300像素）：X=150/210/270/330像素
# 第二行（Y=360像素）：X=150/210/270/330像素
convert task_field_map.pgm -fill black \
-draw "rectangle 137,287 162,312" \
-draw "rectangle 197,287 222,312" \
-draw "rectangle 257,287 282,312" \
-draw "rectangle 317,287 342,312" \
-draw "rectangle 137,347 162,372" \
-draw "rectangle 197,347 222,372" \
-draw "rectangle 257,347 282,372" \
-draw "rectangle 317,347 342,372" \
task_field_map.pgm

# 6. 绘制物资箱归位区（4个，40×40像素，彩色可通行）
# 位置：Y=700像素（核心场地顶部），X=160/240/320/400像素
convert task_field_map.pgm \
-fill "#00FF00" -draw "rectangle 160,700 199,739" \ # 0号：绿色（食品箱）
-fill "#888888" -draw "rectangle 240,700 279,739" \ # 1号：灰色（工具箱）
-fill "#0000FF" -draw "rectangle 320,700 359,739" \ # 2号：蓝色（仪器箱）
-fill "#FF0000" -draw "rectangle 400,700 439,739" \ # 3号：红色（药品箱）
task_field_map.pgm

# 7. 绘制减速带（黄色，可通行，中心Y=500像素）
convert task_field_map.pgm -fill "#FFFF00" \
-draw "rectangle 120,490 479,510" \
task_field_map.pgm

# 8. 绘制启动区（浅蓝色，80×80像素，核心场地底部中央）
convert task_field_map.pgm -fill "#ADD8E6" \
-draw "rectangle 260,220 339,299" \
task_field_map.pgm

# 9. 生成YAML配置文件（适配新尺寸）
cat > task_field_map.yaml << 'END_OF_YAML'
# 任务赛定制地图配置（超出区域不可通行）
image: task_field_map.pgm
resolution: 0.01          # 1像素=0.01m（10mm）
origin: [-1.0, 8.0, 0.0]  # 适配新地图的坐标原点
negate: 0                 # 黑色=不可通行，白色=可通行
occupied_thresh: 0.65
free_thresh: 0.196
END_OF_YAML

echo "✅ 定制化地图生成完成！"
echo "地图说明："
echo "  - 核心赛事区（600×400像素）：白色可通行"
echo "  - 超出区域：黑色不可通行"
echo "  - 启动区、减速带、归位区：可通行"
echo "  - 围墙、存放区：不可通行"
