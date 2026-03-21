#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, PoseArray, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header, String
import math
import heapq

class Node2D:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.g = 0.0
        self.h = 0.0
        self.parent = None
    
    @property
    def f(self):
        return self.g + self.h
    
    def __lt__(self, other):
        return self.f < other.f
    
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
    
    def __hash__(self):
        return hash((self.x, self.y))

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        
        # --- Parameters ---
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('inflation_radius', 0.20)
        self.declare_parameter('robot_radius', 0.15)
        self.declare_parameter('enemy_radius', 0.20)
        self.declare_parameter('gripper_offset', 0.05)
        
        # Рекомендую попробовать 0.10, если локальный контроллер будет останавливаться чуть раньше
        self.declare_parameter('goal_tolerance', 0.10) 

        self.resolution = self.get_parameter('resolution').value
        self.inflation_radius = self.get_parameter('inflation_radius').value
        self.robot_radius = self.get_parameter('robot_radius').value
        self.enemy_radius = self.get_parameter('enemy_radius').value
        self.gripper_offset = self.get_parameter('gripper_offset').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        
        self.width_m = 1.25
        self.height_m = 1.75
        self.grid_w = int(self.width_m / self.resolution) + 1
        self.grid_h = int(self.height_m / self.resolution) + 1
        
        # --- State ---
        self.current_pose = None
        self.goal_pose = None
        self.obstacles = []
        self.enemy_pose = None
        
        # Переменные для фиксации расчётной цели
        self.target_calculated = False
        self.target_x = 0.0
        self.target_y = 0.0
        
        # --- Subs/Pubs ---
        self.sub_pose = self.create_subscription(Pose2D, '/robot/pose', self.pose_cb, 10)
        self.sub_goal = self.create_subscription(Pose2D, '/goal', self.goal_cb, 10)
        self.sub_obstacles = self.create_subscription(PoseArray, '/obstacles', self.obs_cb, 10)
        self.sub_enemy = self.create_subscription(Pose2D, '/enemy/pose', self.enemy_cb, 10)
        
        self.pub_path = self.create_publisher(Path, '/path', 10)
        self.pub_state = self.create_publisher(String, '/state', 10)
        
        # --- Loop ---
        self.timer = self.create_timer(0.2, self.plan_loop) 
        self.get_logger().info("Path Planner Started. Goal synchronization fixed.")

    def pose_cb(self, msg): 
        self.current_pose = msg

    def goal_cb(self, msg): 
        self.goal_pose = msg
        # Сбрасываем флаг при получении новой цели, чтобы пересчитать точку один раз
        self.target_calculated = False

    def obs_cb(self, msg): 
        self.obstacles = msg.poses
        
    def enemy_cb(self, msg): 
        self.enemy_pose = msg

    def world_to_grid(self, x, y):
        gx = int(x / self.resolution)
        gy = int(y / self.resolution)
        return gx, gy

    def grid_to_world(self, gx, gy):
        wx = (gx * self.resolution) + (self.resolution / 2.0)
        wy = (gy * self.resolution) + (self.resolution / 2.0)
        return wx, wy

    def yaw_to_quaternion(self, yaw):
        return {
            'x': 0.0,
            'y': 0.0,
            'z': math.sin(yaw / 2.0),
            'w': math.cos(yaw / 2.0)
        }

    def check_cell(self, gx, gy):
        if gx < 0 or gx >= self.grid_w or gy < 0 or gy >= self.grid_h:
            return False, 0.0
            
        wx, wy = self.grid_to_world(gx, gy)
        
        if wx < self.robot_radius or wx > self.width_m - self.robot_radius or \
           wy < self.robot_radius or wy > self.height_m - self.robot_radius:
            return False, 0.0
            
        penalty = 0.0
        
        if self.enemy_pose:
            dist = math.hypot(wx - self.enemy_pose.x, wy - self.enemy_pose.y)
            min_dist = self.robot_radius + self.enemy_radius
            
            if dist <= min_dist:
                penalty += 1000.0 
            elif dist < min_dist + 0.3:
                penalty += (min_dist + 0.3 - dist) * 50.0
                
        for obs in self.obstacles:
            if self.goal_pose:
                dist_to_goal = math.hypot(obs.position.x - self.goal_pose.x, obs.position.y - self.goal_pose.y)
                if dist_to_goal <= 0.03:
                    continue
                    
            dist = math.hypot(wx - obs.position.x, wy - obs.position.y)
            if dist < self.robot_radius + 0.02: 
                return False, 0.0 
            elif dist < self.robot_radius + self.inflation_radius:
                penalty += (self.robot_radius + self.inflation_radius - dist) * 20.0 
                
        return True, penalty

    def calculate_standoff_pose(self, start_x, start_y, obj_x, obj_y, offset):
        dx = obj_x - start_x
        dy = obj_y - start_y
        dist = math.hypot(dx, dy)
        angle = math.atan2(dy, dx)
        
        if dist <= offset:
            return start_x, start_y, angle
            
        target_x = obj_x - offset * math.cos(angle)
        target_y = obj_y - offset * math.sin(angle)
        
        return target_x, target_y, angle

    def find_nearest_valid_goal(self, goal_gx, goal_gy):
        queue = [(goal_gx, goal_gy)]
        visited = set([(goal_gx, goal_gy)])
        max_search_radius = int(0.5 / self.resolution) 
        
        while queue:
            cx, cy = queue.pop(0)
            is_valid, _ = self.check_cell(cx, cy)
            
            if is_valid:
                return cx, cy
                
            if abs(cx - goal_gx) > max_search_radius or abs(cy - goal_gy) > max_search_radius:
                continue
                
            for dx, dy in [(0,1), (0,-1), (1,0), (-1,0), (1,1), (-1,-1), (1,-1), (-1,1)]:
                nx, ny = cx + dx, cy + dy
                if (nx, ny) not in visited and 0 <= nx < self.grid_w and 0 <= ny < self.grid_h:
                    visited.add((nx, ny))
                    queue.append((nx, ny))
                    
        return None, None

    def heuristic(self, start_node, end_node):
        return math.hypot(end_node.x - start_node.x, end_node.y - start_node.y)

    def get_neighbors(self, node):
        neighbors = []
        for dx, dy in [(0,1), (0,-1), (1,0), (-1,0), (1,1), (1,-1), (-1,1), (-1,-1)]:
            nx, ny = node.x + dx, node.y + dy
            
            is_valid, penalty = self.check_cell(nx, ny)
            if is_valid:
                step_cost = math.hypot(dx, dy)
                cost = step_cost + penalty
                neighbors.append((Node2D(nx, ny), cost))
        return neighbors

    def plan_path(self):
        # Используем УЖЕ ПРОВЕРЕННЫЕ зафиксированные target_x и target_y
        start_gx, start_gy = self.world_to_grid(self.current_pose.x, self.current_pose.y)
        goal_gx, goal_gy = self.world_to_grid(self.target_x, self.target_y)
        
        start_node = Node2D(start_gx, start_gy)
        goal_node = Node2D(goal_gx, goal_gy)
        
        open_list = []
        closed_set = set()
        
        heapq.heappush(open_list, start_node)
        node_map = {(start_gx, start_gy): start_node} 
        
        while open_list:
            current = heapq.heappop(open_list)
            
            if current == goal_node:
                path = []
                # Эта точка теперь всегда валидна
                path.append((self.target_x, self.target_y))
                current = current.parent
                
                while current:
                    if current.parent is None:
                        path.append((self.current_pose.x, self.current_pose.y))
                    else:
                        wx, wy = self.grid_to_world(current.x, current.y)
                        path.append((wx, wy))
                    current = current.parent
                return path[::-1]
            
            closed_set.add((current.x, current.y))
            
            for neighbor, cost in self.get_neighbors(current):
                if (neighbor.x, neighbor.y) in closed_set:
                    continue
                    
                tentative_g = current.g + cost
                existing_neighbor = node_map.get((neighbor.x, neighbor.y))
                
                if not existing_neighbor or tentative_g < existing_neighbor.g:
                    neighbor.g = tentative_g
                    neighbor.h = self.heuristic(neighbor, goal_node)
                    neighbor.parent = current
                    node_map[(neighbor.x, neighbor.y)] = neighbor
                    heapq.heappush(open_list, neighbor)
                    
        return None 

    def plan_loop(self):
        state_msg = String()

        if not self.current_pose or not self.goal_pose:
            state_msg.data = "done"
            self.pub_state.publish(state_msg)
            return

        # Фиксируем точку ОДИН РАЗ и сразу проверяем на препятствия
        if not self.target_calculated:
            tx, ty, _ = self.calculate_standoff_pose(
                self.current_pose.x, self.current_pose.y,
                self.goal_pose.x, self.goal_pose.y,
                self.gripper_offset
            )
            
            # Конвертируем в грид и проверяем
            goal_gx, goal_gy = self.world_to_grid(tx, ty)
            is_valid_goal, _ = self.check_cell(goal_gx, goal_gy)
            
            # Если цель в препятствии, находим ближайшую свободную
            if not is_valid_goal:
                valid_gx, valid_gy = self.find_nearest_valid_goal(goal_gx, goal_gy)
                if valid_gx is not None:
                    tx, ty = self.grid_to_world(valid_gx, valid_gy)
                else:
                    self.get_logger().warn("Could not find valid goal near the target!")
            
            # Запоминаем финальные безопасные координаты
            self.target_x = tx
            self.target_y = ty
            self.target_calculated = True

        # Считаем расстояние до валидной, зафиксированной точки
        dist_to_target = math.hypot(self.target_x - self.current_pose.x, self.target_y - self.current_pose.y)

        # Проверяем прибытие
        if dist_to_target <= self.goal_tolerance:
            state_msg.data = "done"
            self.pub_state.publish(state_msg)
            
            self.get_logger().info("Target reached! Waiting for new task.")
            self.goal_pose = None
            self.target_calculated = False # Готовимся к следующей цели
            
            return 
        
        state_msg.data = "in work"
        self.pub_state.publish(state_msg)

        path_points = self.plan_path()
        
        if path_points:
            path_msg = Path()
            path_msg.header = Header()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = "map"
            
            for i, pt in enumerate(path_points):
                ps = PoseStamped()
                ps.header = path_msg.header
                ps.pose.position.x = pt[0]
                ps.pose.position.y = pt[1]
                
                if i < len(path_points) - 1:
                    next_pt = path_points[i+1]
                    yaw = math.atan2(next_pt[1] - pt[1], next_pt[0] - pt[0])
                else:
                    yaw = math.atan2(self.goal_pose.y - pt[1], self.goal_pose.x - pt[0])
                
                q = self.yaw_to_quaternion(yaw)
                ps.pose.orientation.x = q['x']
                ps.pose.orientation.y = q['y']
                ps.pose.orientation.z = q['z']
                ps.pose.orientation.w = q['w']
                
                path_msg.poses.append(ps)
            
            self.pub_path.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
