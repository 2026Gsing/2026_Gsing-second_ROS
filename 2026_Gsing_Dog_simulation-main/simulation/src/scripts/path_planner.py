#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from visualization_msgs.msg import Marker, MarkerArray
import math
import heapq
import numpy as np

class AStar:
    """A*路径规划算法"""
    def __init__(self, width=6, height=8, resolution=0.05):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.grid_width = int(width / resolution)
        self.grid_height = int(height / resolution)
        self.grid = np.zeros((self.grid_height, self.grid_width), dtype=np.int8)
        self.origin_x = -width/2
        self.origin_y = 0
    
    def world_to_grid(self, x, y):
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        gx = max(0, min(gx, self.grid_width - 1))
        gy = max(0, min(gy, self.grid_height - 1))
        return gx, gy
    
    def grid_to_world(self, gx, gy):
        x = gx * self.resolution + self.origin_x + self.resolution/2
        y = gy * self.resolution + self.origin_y + self.resolution/2
        return x, y
    
    def update_obstacles(self, obstacles, robot_radius=0.3):
        self.grid.fill(0)
        for ox, oy in obstacles:
            gx, gy = self.world_to_grid(ox, oy)
            radius_cells = int(robot_radius / self.resolution) + 2
            for dy in range(-radius_cells, radius_cells + 1):
                for dx in range(-radius_cells, radius_cells + 1):
                    ngx = gx + dx
                    ngy = gy + dy
                    if 0 <= ngx < self.grid_width and 0 <= ngy < self.grid_height:
                        dist = math.hypot(dx * self.resolution, dy * self.resolution)
                        if dist <= robot_radius + 0.1:
                            self.grid[ngy, ngx] = 1
    
    def heuristic(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])
    
    def get_neighbors(self, node):
        x, y = node
        neighbors = []
        directions = [
            (1, 0), (-1, 0), (0, 1), (0, -1),
            (1, 1), (1, -1), (-1, 1), (-1, -1)
        ]
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.grid_width and 0 <= ny < self.grid_height:
                if self.grid[ny, nx] == 0:
                    if abs(dx) == 1 and abs(dy) == 1:
                        if self.grid[y + dy, x] == 0 and self.grid[y, x + dx] == 0:
                            neighbors.append((nx, ny))
                    else:
                        neighbors.append((nx, ny))
        return neighbors
    
    def plan(self, start_world, goal_world):
        start_gx, start_gy = self.world_to_grid(start_world[0], start_world[1])
        goal_gx, goal_gy = self.world_to_grid(goal_world[0], goal_world[1])
        start = (start_gx, start_gy)
        goal = (goal_gx, goal_gy)
        
        # 如果起点或终点在障碍物上，返回直线路径
        if self.grid[start_gy, start_gx] == 1 or self.grid[goal_gy, goal_gx] == 1:
            return [start_world, goal_world]
        
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        
        while open_set:
            current = heapq.heappop(open_set)[1]
            if current == goal:
                path = []
                while current in came_from:
                    world_x, world_y = self.grid_to_world(current[0], current[1])
                    path.append((world_x, world_y))
                    current = came_from[current]
                path.append(start_world)
                path.reverse()
                return path
            
            for neighbor in self.get_neighbors(current):
                dx = neighbor[0] - current[0]
                dy = neighbor[1] - current[1]
                move_cost = 1.414 if (abs(dx) == 1 and abs(dy) == 1) else 1.0
                tentative_g = g_score[current] + move_cost
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        return [start_world, goal_world]


class FastTaskCompetition(Node):
    def __init__(self):
        super().__init__('fast_task_competition')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
        self.get_logger().info("🏆 高速任务赛机器人启动中...")

        # 发布速度指令
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 发布路径可视化
        self.path_pub = self.create_publisher(MarkerArray, '/path_visualization', 10)
        
        # 订阅里程计
        self.odom_sub = self.create_subscription(
            Odometry, 
            '/odom',
            self.odom_callback, 
            10
        )
        
        # 订阅高分区信息
        self.high_score_sub = self.create_subscription(
            Int32,
            '/high_score_zone',
            self.high_score_callback,
            10
        )
        
        # 机器人当前位置
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.odom_received = False
        
        # ==================== 物资点坐标（彩色地板区域）====================
        self.supply_positions = [
            # 近排 (y=1.45)
            {"id": 0, "x": -1.2, "y": 1.45, "type": "食品区", "color": "绿色", "zone_id": 0, "visited": False},
            {"id": 1, "x": -0.4, "y": 1.45, "type": "工具区", "color": "灰色", "zone_id": 1, "visited": False},
            {"id": 2, "x": 0.4, "y": 1.45, "type": "仪器区", "color": "蓝色", "zone_id": 2, "visited": False},
            {"id": 3, "x": 1.2, "y": 1.45, "type": "药品区", "color": "红色", "zone_id": 3, "visited": False},
            # 远排 (y=2.05)
            {"id": 4, "x": -1.2, "y": 2.05, "type": "食品区", "color": "绿色", "zone_id": 0, "visited": False},
            {"id": 5, "x": -0.4, "y": 2.05, "type": "工具区", "color": "灰色", "zone_id": 1, "visited": False},
            {"id": 6, "x": 0.4, "y": 2.05, "type": "仪器区", "color": "蓝色", "zone_id": 2, "visited": False},
            {"id": 7, "x": 1.2, "y": 2.05, "type": "药品区", "color": "红色", "zone_id": 3, "visited": False},
        ]
        
        # ==================== 归位区坐标 ====================
        self.return_zones = [
            {"id": 0, "type": "食品区", "color": "绿色", "x": 1.2, "y": 5.4},
            {"id": 1, "type": "工具区", "color": "灰色", "x": 0.4, "y": 5.4},
            {"id": 2, "type": "仪器区", "color": "蓝色", "x": -0.4, "y": 5.4},
            {"id": 3, "type": "药品区", "color": "红色", "x": -1.2, "y": 5.4},
        ]
        
        # ==================== 高分区 ======================
        self.high_score_zone = None
        
        # ==================== A*路径规划 ====================
        self.astar = AStar(width=6.0, height=8.0, resolution=0.05)
        
        # ==================== 路径跟踪 ====================
        self.current_path = []
        self.path_index = 0
        self.lookahead_distance = 0.3
        
        # ==================== 任务序列 ====================
        self.task_sequence = self.calculate_optimal_sequence()
        self.current_task_index = 0
        self.score = 0
        
        # 当前携带的"物资类型"（实际上是记忆要去的归位区）
        self.current_carrying_type = ""
        self.current_carrying_zone = -1
        
        # 控制参数
        self.control_rate = 50
        self.max_linear_speed = 0.3
        self.min_linear_speed = 0.05
        self.max_angular_speed = 2.5
        
        # PID参数
        self.kp_linear = 1.0
        self.kp_angular = 3.0
        self.ki_angular = 0.05
        self.kd_angular = 0.1
        self.angular_error_integral = 0.0
        self.last_angular_error = 0.0
        
        # 到达阈值
        self.supply_threshold = 0.4
        self.return_threshold = 0.3
        self.avoid_distance = 0.5  # 避让距离
        
        # 计时器
        self.timer = self.create_timer(1.0/self.control_rate, self.control_loop)
        
        self.debug_count = 0
        self.last_task_print = 0  # 上次打印任务列表的时间
        
        self.print_task_list()
        self.print_instructions()

    def print_instructions(self):
        """打印使用说明"""
        self.get_logger().info("="*60)
        self.get_logger().info("📋 使用说明：")
        self.get_logger().info("   1. 空载时才能触发彩色区域（记录类型）")
        self.get_logger().info("   2. 触发后立即进入【携带状态】")
        self.get_logger().info("   3. 携带状态下会自动避开【未访问的】彩色区域")
        self.get_logger().info("   4. 【已访问的】彩色区域可以自由通行")
        self.get_logger().info("   5. 到达归位区后解除携带状态")
        self.get_logger().info("   6. 减速带已改为平面标记，无物理障碍")
        self.get_logger().info("="*60)

    def high_score_callback(self, msg):
        zone_id = msg.data
        if 0 <= zone_id <= 3:
            self.high_score_zone = zone_id
            self.get_logger().info(f"🎯 高分区设置为: {self.return_zones[zone_id]['color']}区")
            self.task_sequence = self.calculate_optimal_sequence()
            self.current_task_index = 0
            self.print_task_list()

    def odom_callback(self, msg):
        """里程计回调"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_theta = math.atan2(siny_cosp, cosy_cosp)
        
        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info("="*80)
            self.get_logger().info("📍 里程计初始化:")
            self.get_logger().info(f"   起点坐标: (0, 0)")
            self.get_logger().info(f"   初始角度: {math.degrees(self.current_theta):.1f}°")
            self.get_logger().info("="*80)

    def calculate_optimal_sequence(self):
        """计算最优任务顺序 - 先近排后远排"""
        available = [s for s in self.supply_positions if not s["visited"]]
        if not available:
            return [("start", None, "返回起点", (0, 0))]
        
        # 按行分组
        near_row = [s for s in available if abs(s["y"] - 1.45) < 0.01]
        far_row = [s for s in available if abs(s["y"] - 2.05) < 0.01]
        
        sequence = []
        
        # 按x坐标排序（从左到右）
        near_row.sort(key=lambda s: s["x"])
        far_row.sort(key=lambda s: s["x"])
        
        # 先处理近排
        for supply in near_row:
            # 到达彩色区域
            sequence.append((
                "supply", supply["id"],
                f"到达{supply['type']}(近排)",
                (supply["x"], supply["y"])
            ))
            # 立即去对应颜色的归位区
            zone = self.return_zones[supply["zone_id"]]
            sequence.append((
                "return", supply["zone_id"],
                f"去{zone['color']}归位区",
                (zone["x"], zone["y"])
            ))
        
        # 再处理远排
        for supply in far_row:
            # 到达彩色区域
            sequence.append((
                "supply", supply["id"],
                f"到达{supply['type']}(远排)",
                (supply["x"], supply["y"])
            ))
            # 立即去对应颜色的归位区
            zone = self.return_zones[supply["zone_id"]]
            sequence.append((
                "return", supply["zone_id"],
                f"去{zone['color']}归位区",
                (zone["x"], zone["y"])
            ))
        
        # 最后返回起点
        sequence.append(("start", None, "返回起点", (0, 0)))
        
        return sequence

    def print_task_list(self):
        """打印任务序列"""
        self.get_logger().info("="*100)
        self.get_logger().info(f"📋 任务序列 (当前任务: {self.current_task_index+1}/{len(self.task_sequence)})")
        
        # 统计完成情况
        visited_count = sum(1 for s in self.supply_positions if s["visited"])
        self.get_logger().info(f"📊 进度: 已访问 {visited_count}/{len(self.supply_positions)} 个彩色区域, 当前得分: {self.score}")
        
        for i, task in enumerate(self.task_sequence):
            if task[3]:
                visited = False
                if task[0] == "supply" and task[1] is not None:
                    visited = self.supply_positions[task[1]]["visited"]
                status = "✓" if visited else " "
                
                # 标记当前任务
                current_marker = "→ " if i == self.current_task_index else "  "
                
                # 标记携带状态
                carrying_info = ""
                if self.current_carrying_zone != -1:
                    carrying_info = f" [携带:{self.current_carrying_type}]"
                
                # 如果是当前任务，用星号标记
                if i == self.current_task_index:
                    self.get_logger().info(f"  {current_marker}{i+1:2d}. [{status}] ⭐ {task[2]:<25} ({task[3][0]:5.2f}, {task[3][1]:5.2f}){carrying_info}")
                else:
                    self.get_logger().info(f"  {current_marker}{i+1:2d}. [{status}]   {task[2]:<25} ({task[3][0]:5.2f}, {task[3][1]:5.2f}){carrying_info}")
        self.get_logger().info("="*100)
        
        self.last_task_print = self.debug_count

    def mark_supply_visited(self, supply):
        """标记彩色区域为已访问，并进入携带状态"""
        supply["visited"] = True
        self.current_carrying_type = supply["type"]
        self.current_carrying_zone = supply["zone_id"]
        
        self.current_path = []
        self.path_index = 0
        
        visited_count = sum(1 for s in self.supply_positions if s["visited"])
        self.get_logger().info(f'✅ 已到达 {supply["type"]}，现在进入携带状态！ (进度: {visited_count}/{len(self.supply_positions)})')
        self.get_logger().info(f'⚠️ 携带状态下会自动避开【未访问的】彩色区域')
        self.get_logger().info(f'✅ 【已访问的】彩色区域可以自由通行')
        self.get_logger().info(f'🎯 目标: 去{self.return_zones[supply["zone_id"]]["color"]}归位区')

    def get_obstacle_positions(self):
        """
        获取所有障碍物位置
        - 空载时：没有障碍物
        - 携带时：只将【未访问的】彩色区域视为障碍物
        - 【已访问的】彩色区域不作为障碍物，可以通行
        """
        obstacles = []
        
        # 只有在携带状态下才需要避开未访问的彩色区域
        if self.current_carrying_zone != -1:
            visited_count = 0
            obstacle_count = 0
            
            for supply in self.supply_positions:
                if supply["visited"]:
                    visited_count += 1
                else:
                    # 只将未访问的彩色区域作为障碍物
                    obstacles.append((supply["x"], supply["y"]))
                    obstacle_count += 1
            
            if obstacles and self.debug_count % 50 == 0:
                self.get_logger().info(f"🗺️ 携带状态避障: {obstacle_count}个未访问区域为障碍, {visited_count}个已访问区域可通行")
                    
        return obstacles

    def plan_path(self, start, goal):
        """规划路径（根据状态决定是否避障）"""
        obstacles = self.get_obstacle_positions()
        
        if obstacles:
            # 有障碍物时使用A*避障
            self.astar.update_obstacles(obstacles, robot_radius=0.3)
            path = self.astar.plan(start, goal)
            if len(path) > 2 and self.debug_count % 20 == 0:
                self.get_logger().info(f"🔄 携带状态下避开 {len(obstacles)} 个【未访问】彩色区域")
        else:
            # 无障碍物时直线路径
            path = [start, goal]
        
        self.visualize_path(path)
        return path

    def visualize_path(self, path):
        """可视化路径"""
        marker_array = MarkerArray()
        
        # 路径点
        for i, (x, y) in enumerate(path):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "path"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.1
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.a = 0.8
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)
        
        # 连线
        if len(path) > 1:
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "path_line"
            marker.id = 0
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.02
            marker.color.a = 0.8
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            
            for x, y in path:
                p = Point()
                p.x = x
                p.y = y
                p.z = 0.1
                marker.points.append(p)
            
            marker_array.markers.append(marker)
        
        self.path_pub.publish(marker_array)

    def get_lookahead_point(self):
        """获取前瞻点"""
        if not self.current_path or self.path_index >= len(self.current_path):
            return None
        
        for i in range(self.path_index, len(self.current_path)):
            x, y = self.current_path[i]
            dist = math.hypot(x - self.current_x, y - self.current_y)
            
            if dist >= self.lookahead_distance:
                return (x, y)
        
        return self.current_path[-1]

    def calculate_target_angle(self, target_x, target_y):
        """计算目标角度"""
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        return math.atan2(dy, dx)

    def check_proximity_to_unvisited_zones(self):
        """检查是否太接近【未访问的】彩色区域（携带状态下）"""
        if self.current_carrying_zone == -1:
            return False
        
        for supply in self.supply_positions:
            # 只检查未访问的区域
            if not supply["visited"]:
                dist = math.hypot(self.current_x - supply["x"], self.current_y - supply["y"])
                if dist < self.avoid_distance:
                    self.get_logger().warn(f"⚠️ 太接近【未访问】彩色区域: {supply['type']}, 距离: {dist:.2f}m")
                    return True
        return False

    def pid_control(self, target_x, target_y):
        """PID控制器"""
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        distance = math.hypot(dx, dy)
        
        target_angle = self.calculate_target_angle(target_x, target_y)
        angle_error = target_angle - self.current_theta
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
        
        # 检查是否太接近未访问的彩色区域
        proximity_warning = self.check_proximity_to_unvisited_zones()
        
        self.debug_count += 1
        if self.debug_count % 20 == 0:
            if self.current_carrying_zone != -1:
                carrying_info = f"🎯 携带:{self.current_carrying_type} → 去{self.return_zones[self.current_carrying_zone]['color']}区"
                if proximity_warning:
                    carrying_info += " ⚠️ 避让未访问区"
            else:
                carrying_info = "🚶 空载"
            
            # 显示当前任务进度
            task_info = f"任务: {self.current_task_index+1}/{len(self.task_sequence)}"
            
            self.get_logger().info(
                f"{carrying_info} | {task_info} | 当前位置: ({self.current_x:.2f}, {self.current_y:.2f}) | "
                f"目标: ({target_x:.2f}, {target_y:.2f}) | "
                f"距离: {distance:.2f}m | "
                f"角度误差: {math.degrees(angle_error):.1f}°"
            )
        
        # PID计算角速度
        self.angular_error_integral += angle_error * (1.0/self.control_rate)
        self.angular_error_integral = max(-1.0, min(1.0, self.angular_error_integral))
        
        angular_error_derivative = (angle_error - self.last_angular_error) * self.control_rate
        
        angular_z = (self.kp_angular * angle_error + 
                     self.ki_angular * self.angular_error_integral + 
                     self.kd_angular * angular_error_derivative)
        angular_z = max(-self.max_angular_speed, min(self.max_angular_speed, angular_z))
        
        self.last_angular_error = angle_error
        
        # 速度决策
        if distance < 0.3:
            angle_threshold = 10.0
        elif distance < 0.5:
            angle_threshold = 15.0
        else:
            angle_threshold = 20.0
        
        if abs(math.degrees(angle_error)) > angle_threshold:
            linear_x = 0.0
        else:
            speed_factor = 1.0
            
            # 如果太接近未访问的彩色区域，减速
            if proximity_warning:
                speed_factor *= 0.5
            
            # 接近目标减速
            if distance < 0.3:
                speed_factor *= 0.3
            elif distance < 0.5:
                speed_factor *= 0.5
            
            base_speed = min(self.max_linear_speed, distance * self.kp_linear)
            base_speed = max(self.min_linear_speed, base_speed)
            linear_x = base_speed * speed_factor
        
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        
        return twist, distance

    def check_supply_reached(self):
        """检查是否到达彩色区域（只有在空载时才能触发）"""
        # 如果正在携带，绝对不能触发新区域
        if self.current_carrying_zone != -1:
            return
        
        for supply in self.supply_positions:
            if not supply["visited"]:
                dist = math.hypot(self.current_x - supply["x"], self.current_y - supply["y"])
                
                if dist < self.supply_threshold:
                    self.get_logger().info(f"🎯 到达彩色区域: {supply['type']}")
                    
                    # 立即停止
                    self.cmd_vel_pub.publish(Twist())
                    
                    # 标记为已访问并进入携带状态
                    self.mark_supply_visited(supply)
                    break

    def execute_task(self):
        """执行当前任务"""
        if self.current_task_index >= len(self.task_sequence):
            return True
            
        task = self.task_sequence[self.current_task_index]
        
        # 检查任务类型和当前状态是否匹配
        if task[0] == "supply":
            # 到达彩色区域任务：必须空载
            if self.current_carrying_zone != -1:
                self.get_logger().warn(f"⚠️ 当前正要去归位区（携带{self.current_carrying_type}），不能执行到达任务！")
                # 强制跳转到对应的归位任务
                for i in range(self.current_task_index + 1, len(self.task_sequence)):
                    if self.task_sequence[i][0] == "return" and self.task_sequence[i][1] == self.current_carrying_zone:
                        self.current_task_index = i
                        self.get_logger().info(f"🔄 强制跳转到归位任务: 去{self.return_zones[self.current_carrying_zone]['color']}区")
                        return False
                return False
            
            # 检查区域是否已被访问
            supply_id = task[1]
            if self.supply_positions[supply_id]["visited"]:
                self.get_logger().info(f"⏩ 跳过已访问区域: {task[2]}")
                self.current_task_index += 1
                return False
                
        elif task[0] == "return":
            # 归位任务：必须携带对应类型的物资
            if self.current_carrying_zone == -1:
                self.get_logger().warn(f"⚠️ 当前空载，不能执行归位任务！")
                # 强制跳转到对应的到达任务
                for i in range(self.current_task_index + 1, len(self.task_sequence)):
                    if self.task_sequence[i][0] == "supply" and not self.supply_positions[self.task_sequence[i][1]]["visited"]:
                        self.current_task_index = i
                        self.get_logger().info(f"🔄 强制跳转到到达任务")
                        return False
                return False
            
            # 检查归位的区域类型是否匹配
            zone_id = task[1]
            if self.current_carrying_zone != zone_id:
                self.get_logger().warn(f"⚠️ 当前携带的是{self.current_carrying_type}，应该去{self.return_zones[self.current_carrying_zone]['color']}区，不是当前目标！")
                # 跳转到正确的归位任务
                for i in range(self.current_task_index + 1, len(self.task_sequence)):
                    if self.task_sequence[i][0] == "return" and self.task_sequence[i][1] == self.current_carrying_zone:
                        self.current_task_index = i
                        self.get_logger().info(f"🔄 跳转到正确的归位任务: 去{self.return_zones[self.current_carrying_zone]['color']}区")
                        return False
                return False
        
        target = task[3]
        if target is None:
            self.current_task_index += 1
            return False
        
        target_x, target_y = target
        
        # 检查是否到达目标
        dist_to_target = math.hypot(target_x - self.current_x, target_y - self.current_y)
        
        if task[0] == "supply":
            threshold = self.supply_threshold
        elif task[0] == "return":
            threshold = self.return_threshold
        else:
            threshold = 0.15
        
        if dist_to_target < threshold:
            self.get_logger().info(f"🎯 到达目标: {task[2]}")
            
            if task[0] == "return":
                # 到达归位区 - 得分！
                if self.current_carrying_zone != -1:
                    zone_id = task[1]
                    zone = self.return_zones[zone_id]
                    
                    base_score = 10
                    if self.high_score_zone == zone_id:
                        base_score += 5
                        self.get_logger().info(f"✨ 高分区奖励 +5")
                    
                    self.score += base_score
                    
                    visited_count = sum(1 for s in self.supply_positions if s["visited"])
                    self.get_logger().info(f"✅ 到达 {zone['color']}归位区, 得分: +{base_score}, 总分: {self.score}, 进度: {visited_count}/{len(self.supply_positions)}")
                    self.get_logger().info(f"🔄 携带状态解除，可以触发下一个彩色区域")
                    
                    # 清空携带状态
                    self.current_carrying_type = ""
                    self.current_carrying_zone = -1
                    
                    # 立即重新规划路径
                    self.current_path = []
                    self.path_index = 0
                    
            elif task[0] == "start":
                visited_count = sum(1 for s in self.supply_positions if s["visited"])
                total_count = len(self.supply_positions)
                self.get_logger().info(f"🏆 任务完成! 总分: {self.score}, 访问区域: {visited_count}/{total_count}")
                self.get_logger().info('''
                ╔════════════════════════════════╗
                ║     🏆 恭喜！任务完成！        ║
                ║     ⭐ 最终得分: {}            ║
                ║     📊 完成率: {}/{}           ║
                ╚════════════════════════════════╝
                '''.format(self.score, visited_count, total_count))
            
            self.current_task_index += 1
            self.current_path = []
            self.path_index = 0
            
            # 打印更新后的任务列表
            if self.current_task_index < len(self.task_sequence):
                self.print_task_list()
            return False
        
        # 路径规划 - 每次规划前更新障碍物
        if not self.current_path or self.path_index >= len(self.current_path):
            start = (self.current_x, self.current_y)
            goal = (target_x, target_y)
            self.current_path = self.plan_path(start, goal)
            self.path_index = 0
            if len(self.current_path) > 2:
                self.get_logger().info(f"🛣️ 规划新路径，长度: {len(self.current_path)}点")
        
        # 获取前瞻点
        lookahead = self.get_lookahead_point()
        if lookahead is None:
            self.current_path = []
            return False
        
        # 更新路径索引
        for i in range(self.path_index, len(self.current_path)):
            x, y = self.current_path[i]
            dist = math.hypot(x - self.current_x, y - self.current_y)
            if dist < 0.1:
                self.path_index = i + 1
        
        # PID控制
        twist, _ = self.pid_control(lookahead[0], lookahead[1])
        self.cmd_vel_pub.publish(twist)
        
        return False

    def control_loop(self):
        """主控制循环"""
        if not self.odom_received:
            self.get_logger().warn("⏳ 等待里程计...", throttle_duration_sec=2.0)
            return
        
        # 检查是否到达彩色区域（只有在空载时才会触发）
        self.check_supply_reached()
        
        completed = self.execute_task()
        
        if completed:
            self.cmd_vel_pub.publish(Twist())
            self.timer.cancel()
            self.get_logger().info("🏁 所有任务完成！")


def main(args=None):
    try:
        rclpy.init(args=args)
        print("\n" + "="*60)
        print("🤖 Gsing Dog 高速任务赛机器人")
        print("="*60)
        print("\n✨ 模式：到达彩色区域 → 去对应颜色归位区")
        print("   ⚠️ 重要规则：")
        print("   1. 空载时才能触发彩色区域")
        print("   2. 触发后进入【携带状态】")
        print("   3. 携带状态下【自动避开】未访问的彩色区域")
        print("   4. 【已访问的】彩色区域可以自由通行")
        print("   5. 到达归位区后解除携带状态")
        print("   6. 减速带已改为平面标记，无物理障碍")
        print("\n🎮 启动区: (0, 0)")
        print("📦 近排区域: y=1.45 (优先)")
        print("📦 远排区域: y=2.05")
        print("🎯 归位区: y=5.4")
        print("="*60)
        
        robot = FastTaskCompetition()
        print("✅ 机器人创建成功")
        
        rclpy.spin(robot)
        robot.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        print("\n👋 用户中断")
    except Exception as e:
        print(f"❌ 错误: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()