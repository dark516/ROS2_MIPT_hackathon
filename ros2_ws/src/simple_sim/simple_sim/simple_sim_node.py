#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D, PoseArray, Pose
from std_msgs.msg import Header, Bool
from nav_msgs.msg import Path
import pygame
import math
import sys

# Constants
FIELD_WIDTH_M = 1.25
FIELD_HEIGHT_M = 1.75
DEFAULT_SCREEN_WIDTH = 500
DEFAULT_SCREEN_HEIGHT = 700

ROBOT_SIDE_M = 0.2
ROBOT_RADIUS_M = (ROBOT_SIDE_M * math.sqrt(2)) / 2.0 

OBSTACLE_RADIUS_M = 0.02
ARM_REACH_M = 0.15
ARM_CATCH_RADIUS = 0.05

DT = 0.05  # 20 Hz

# Colors
WHITE = (255, 255, 255)
BLUE = (0, 0, 255)      # Our Robot
RED = (255, 0, 0)       # Enemy
BLACK = (0, 0, 0)
GREY = (128, 128, 128)  # Obstacles
LIGHT_GREY = (230, 230, 230)
GRID_COLOR = (240, 240, 240)
TEXT_COLOR = (50, 50, 50)
ARM_COLOR = (0, 200, 0)
PATH_COLOR = (0, 255, 0) # Green path
GOAL_COLOR = (255, 0, 255) # Magenta goal
ZONE_COLOR = (255, 165, 0) # Orange for exclusion zone

class Simple2DSim(Node):
    def __init__(self):
        super().__init__('simple_sim_node')

        # Initialize PyGame
        pygame.init()
        pygame.font.init()
        self.font = pygame.font.SysFont('Arial', 12)
        
        self.screen_width = DEFAULT_SCREEN_WIDTH
        self.screen_height = DEFAULT_SCREEN_HEIGHT
        self.screen = pygame.display.set_mode((self.screen_width, self.screen_height), pygame.RESIZABLE)
        pygame.display.set_caption("Simple 2D ROS2 Simulator")
        self.clock = pygame.time.Clock()

        self.update_scale()

        # State
        self.our_robot_pose = [0.625, 0.2, 1.57]  
        self.enemy_robot_pose = [0.625, 1.5, -1.57] 
        self.obstacles = [
            {'x': 0.6, 'y': 0.8, 'r': OBSTACLE_RADIUS_M},
            {'x': 0.4, 'y': 1.0, 'r': OBSTACLE_RADIUS_M},
            {'x': 0.8, 'y': 1.0, 'r': OBSTACLE_RADIUS_M},
            {'x': 0.5, 'y': 1.2, 'r': OBSTACLE_RADIUS_M},
            {'x': 0.7, 'y': 1.2, 'r': OBSTACLE_RADIUS_M}
        ]
        
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.arm_open = True
        
        self.held_object = None 
        
        self.dragging = None 
        self.drag_offset = (0, 0)
        
        # Переменные для зоны исключения (exclusion zone)
        self.exclusion_zone = None # tuple: (min_x, max_x, min_y, max_y)
        self.drawing_zone_start = None
        self.current_drawing_end = None
        
        self.current_path = [] # List of (x,y) tuples
        self.current_goal = None # (x,y) tuple

        # ROS2 Interfaces
        self.pub_our_pose = self.create_publisher(Pose2D, '/robot/pose', 10)
        self.pub_enemy_pose = self.create_publisher(Pose2D, '/enemy/pose', 10)
        self.pub_obstacles = self.create_publisher(PoseArray, '/obstacles', 10)
        self.pub_goal = self.create_publisher(Pose2D, '/goal', 10)
        
        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.sub_arm = self.create_subscription(Bool, '/robot/arm', self.arm_callback, 10)
        self.sub_path = self.create_subscription(Path, '/path', self.path_callback, 10)

        self.timer = self.create_timer(DT, self.game_loop)
        self.get_logger().info(f"Sim Started. Right-click to set goal. Drag left-click on empty space to draw ignore zone.")

    def update_scale(self):
        scale_x = self.screen_width / FIELD_WIDTH_M
        scale_y = self.screen_height / FIELD_HEIGHT_M
        self.pixels_per_meter = min(scale_x, scale_y)
        self.offset_x = (self.screen_width - (FIELD_WIDTH_M * self.pixels_per_meter)) / 2.0
        self.offset_y = (self.screen_height - (FIELD_HEIGHT_M * self.pixels_per_meter)) / 2.0

    def cmd_vel_callback(self, msg):
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z

    def arm_callback(self, msg):
        was_open = self.arm_open
        self.arm_open = msg.data 
        if was_open and not self.arm_open: 
            self.try_grab()
        elif not was_open and self.arm_open: 
            if self.held_object is not None:
                self.obstacles.append(self.held_object)
                self.held_object = None

    def path_callback(self, msg):
        self.current_path = []
        for pose in msg.poses:
            self.current_path.append((pose.pose.position.x, pose.pose.position.y))

    def try_grab(self):
        if self.held_object is not None: return
        rx, ry, rtheta = self.our_robot_pose
        catch_x = rx + (ROBOT_SIDE_M/2.0 + 0.05) * math.cos(rtheta)
        catch_y = ry + (ROBOT_SIDE_M/2.0 + 0.05) * math.sin(rtheta)
        
        best_idx = None
        min_dist = ARM_CATCH_RADIUS
        for i, obs in enumerate(self.obstacles):
            dist = math.hypot(catch_x - obs['x'], catch_y - obs['y'])
            if dist < min_dist:
                min_dist = dist
                best_idx = i
        
        if best_idx is not None: 
            self.held_object = self.obstacles.pop(best_idx)

    def to_pixels(self, x, y):
        px = int(self.offset_x + x * self.pixels_per_meter)
        py = int(self.screen_height - self.offset_y - (y * self.pixels_per_meter))
        return px, py

    def to_meters(self, px, py):
        x = (px - self.offset_x) / self.pixels_per_meter
        y = (self.screen_height - self.offset_y - py) / self.pixels_per_meter
        return x, y

    def clamp_pose(self, pose):
        margin = ROBOT_SIDE_M / 2.0
        pose[0] = max(margin, min(FIELD_WIDTH_M - margin, pose[0]))
        pose[1] = max(margin, min(FIELD_HEIGHT_M - margin, pose[1]))

    def draw_grid_and_axes(self):
        fx_start, fy_start = self.to_pixels(0, FIELD_HEIGHT_M)
        fx_end, fy_end = self.to_pixels(FIELD_WIDTH_M, 0)
        rect = pygame.Rect(fx_start, fy_start, fx_end - fx_start, fy_end - fy_start)
        pygame.draw.rect(self.screen, WHITE, rect)
        pygame.draw.rect(self.screen, BLACK, rect, 2)

        steps_y = int(FIELD_HEIGHT_M * 100)
        for i in range(steps_y + 1):
            y_m = i / 100.0
            px_start, py = self.to_pixels(0, y_m)
            px_end, _ = self.to_pixels(FIELD_WIDTH_M, y_m)
            color = GRID_COLOR
            width = 1
            if i % 10 == 0: color = LIGHT_GREY
            if i % 50 == 0: color = GREY
            pygame.draw.line(self.screen, color, (px_start, py), (px_end, py), width)
            if i % 10 == 0:
                label = self.font.render(f"{y_m:.1f}", True, TEXT_COLOR)
                self.screen.blit(label, (px_start - 25, py - 5))

        steps_x = int(FIELD_WIDTH_M * 100)
        for i in range(steps_x + 1):
            x_m = i / 100.0
            px, py_start = self.to_pixels(x_m, 0)
            _, py_end = self.to_pixels(x_m, FIELD_HEIGHT_M)
            color = GRID_COLOR
            width = 1
            if i % 10 == 0: color = LIGHT_GREY
            if i % 50 == 0: color = GREY
            pygame.draw.line(self.screen, color, (px, py_start), (px, py_end), width)
            if i % 10 == 0:
                label = self.font.render(f"{x_m:.1f}", True, TEXT_COLOR)
                self.screen.blit(label, (px - 10, py_end + 5))

    def draw_path(self):
        if len(self.current_path) < 2: return
        points = []
        for pt in self.current_path:
            px, py = self.to_pixels(pt[0], pt[1])
            points.append((px, py))
        pygame.draw.lines(self.screen, PATH_COLOR, False, points, 2)

    def draw_goal(self):
        if self.current_goal:
            px, py = self.to_pixels(self.current_goal[0], self.current_goal[1])
            size = 5
            pygame.draw.line(self.screen, GOAL_COLOR, (px - size, py - size), (px + size, py + size), 2)
            pygame.draw.line(self.screen, GOAL_COLOR, (px - size, py + size), (px + size, py - size), 2)

    def draw_robot(self, x, y, theta, color, is_us=False):
        px, py = self.to_pixels(x, y)
        half_side = ROBOT_SIDE_M / 2.0
        corners = [
            (half_side, half_side),
            (-half_side, half_side),
            (-half_side, -half_side),
            (half_side, -half_side)
        ]
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        screen_points = []
        for cx, cy in corners:
            rx = cx * cos_t - cy * sin_t
            ry = cx * sin_t + cy * cos_t
            spx, spy = self.to_pixels(x + rx, y + ry)
            screen_points.append((spx, spy))
        pygame.draw.polygon(self.screen, color, screen_points)
        pygame.draw.polygon(self.screen, BLACK, screen_points, 2)
        
        wheel_len = 0.08
        wheel_width = 0.04
        for side in [1, -1]:
            wc_y = side * half_side
            w_corners = [
                (wheel_len/2, wc_y + wheel_width/2),
                (-wheel_len/2, wc_y + wheel_width/2),
                (-wheel_len/2, wc_y - wheel_width/2),
                (wheel_len/2, wc_y - wheel_width/2)
            ]
            w_screen_points = []
            for wx, wy in w_corners:
                rx = wx * cos_t - wy * sin_t
                ry = wx * sin_t + wy * cos_t
                spx, spy = self.to_pixels(x + rx, y + ry)
                w_screen_points.append((spx, spy))
            pygame.draw.polygon(self.screen, BLACK, w_screen_points)

        if is_us:
            arm_len = 0.08
            arm_spread = 0.1 if self.arm_open else 0.03
            base_x = half_side
            for fy in [arm_spread/2.0, -arm_spread/2.0]:
                p1_x, p1_y = base_x, fy
                p2_x, p2_y = base_x + arm_len, fy
                rx1 = p1_x * cos_t - p1_y * sin_t
                ry1 = p1_x * sin_t + p1_y * cos_t
                rx2 = p2_x * cos_t - p2_y * sin_t
                ry2 = p2_x * sin_t + p2_y * cos_t
                spx1, spy1 = self.to_pixels(x + rx1, y + ry1)
                spx2, spy2 = self.to_pixels(x + rx2, y + ry2)
                pygame.draw.line(self.screen, ARM_COLOR, (spx1, spy1), (spx2, spy2), 5)

    def draw_obstacle(self, obs, is_held=False):
        px, py = self.to_pixels(obs['x'], obs['y'])
        radius_px = int(obs['r'] * self.pixels_per_meter)
        
        # Визуально тускнеет, если находится в зоне игнорирования
        is_ignored = False
        if not is_held and self.exclusion_zone:
            min_x, max_x, min_y, max_y = self.exclusion_zone
            if min_x <= obs['x'] <= max_x and min_y <= obs['y'] <= max_y:
                is_ignored = True

        color = (100, 200, 100) if is_held else GREY
        if is_ignored: color = (200, 200, 200) # Светло-серый для невидимых
            
        pygame.draw.circle(self.screen, color, (px, py), radius_px)
        pygame.draw.circle(self.screen, BLACK, (px, py), radius_px, 1)

    def draw_exclusion_zone(self):
        def _draw_rect(min_x, max_x, min_y, max_y, color, alpha=60):
            px_start, py_start = self.to_pixels(min_x, max_y)
            px_end, py_end = self.to_pixels(max_x, min_y)
            w = max(1, px_end - px_start)
            h = max(1, py_end - py_start)
            rect = pygame.Rect(px_start, py_start, w, h)
            
            s = pygame.Surface((w, h), pygame.SRCALPHA)
            s.fill((*color, alpha))
            self.screen.blit(s, (rect.x, rect.y))
            pygame.draw.rect(self.screen, color, rect, 2)

        # Отрисовка зафиксированной зоны
        if self.exclusion_zone:
            _draw_rect(*self.exclusion_zone, ZONE_COLOR)
            
        # Отрисовка зоны во время перетаскивания мыши
        if self.drawing_zone_start and self.current_drawing_end:
            x1, y1 = self.drawing_zone_start
            x2, y2 = self.current_drawing_end
            _draw_rect(min(x1, x2), max(x1, x2), min(y1, y2), max(y1, y2), (200, 200, 0), 40)

    def resolve_robot_collisions(self):
        dx = self.our_robot_pose[0] - self.enemy_robot_pose[0]
        dy = self.our_robot_pose[1] - self.enemy_robot_pose[1]
        dist = math.hypot(dx, dy)
        min_dist = ROBOT_SIDE_M 
        if dist < min_dist and dist > 0.001:
            overlap = min_dist - dist
            push_x = (dx / dist) * (overlap / 2.0)
            push_y = (dy / dist) * (overlap / 2.0)
            self.our_robot_pose[0] += push_x
            self.our_robot_pose[1] += push_y
            self.enemy_robot_pose[0] -= push_x
            self.enemy_robot_pose[1] -= push_y

    def resolve_object_collisions(self, robot_pose):
        rx, ry, _ = robot_pose
        for obs in self.obstacles:
            dist = math.hypot(rx - obs['x'], ry - obs['y'])
            min_dist = (ROBOT_SIDE_M / 2.0) + obs['r']
            if dist < min_dist:
                overlap = min_dist - dist
                if dist < 0.001: dist = 0.001
                nx = (obs['x'] - rx) / dist
                ny = (obs['y'] - ry) / dist
                obs['x'] += nx * overlap
                obs['y'] += ny * overlap

    def update_physics(self):
        if self.dragging != 'our':
            self.our_robot_pose[2] += self.angular_vel * DT
            self.our_robot_pose[0] += self.linear_vel * math.cos(self.our_robot_pose[2]) * DT
            self.our_robot_pose[1] += self.linear_vel * math.sin(self.our_robot_pose[2]) * DT
            self.our_robot_pose[2] = math.atan2(math.sin(self.our_robot_pose[2]), math.cos(self.our_robot_pose[2]))

        if self.held_object is not None:
            rx, ry, rtheta = self.our_robot_pose
            hold_dist = ROBOT_SIDE_M/2.0 + self.held_object['r']
            self.held_object['x'] = rx + hold_dist * math.cos(rtheta)
            self.held_object['y'] = ry + hold_dist * math.sin(rtheta)

        self.resolve_robot_collisions()
        self.resolve_object_collisions(self.our_robot_pose)
        self.resolve_object_collisions(self.enemy_robot_pose)
        self.clamp_pose(self.our_robot_pose)
        self.clamp_pose(self.enemy_robot_pose)

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                rclpy.shutdown()
                sys.exit()
            
            elif event.type == pygame.VIDEORESIZE:
                self.screen_width = event.w
                self.screen_height = event.h
                self.screen = pygame.display.set_mode((self.screen_width, self.screen_height), pygame.RESIZABLE)
                self.update_scale()

            elif event.type == pygame.MOUSEBUTTONDOWN:
                mx, my = pygame.mouse.get_pos()
                m_x, m_y = self.to_meters(mx, my)
                
                if event.button == 3: # Right Click -> Set Goal
                    self.current_goal = (m_x, m_y)
                    goal_msg = Pose2D()
                    goal_msg.x = m_x
                    goal_msg.y = m_y
                    self.pub_goal.publish(goal_msg)
                    return

                # Left Click -> Drag Obj or Draw Zone
                if event.button == 1:
                    if math.hypot(m_x - self.our_robot_pose[0], m_y - self.our_robot_pose[1]) < ROBOT_SIDE_M/1.5:
                        self.dragging = 'our'
                        self.drag_offset = (self.our_robot_pose[0] - m_x, self.our_robot_pose[1] - m_y)
                        return
                    if math.hypot(m_x - self.enemy_robot_pose[0], m_y - self.enemy_robot_pose[1]) < ROBOT_SIDE_M/1.5:
                        self.dragging = 'enemy'
                        self.drag_offset = (self.enemy_robot_pose[0] - m_x, self.enemy_robot_pose[1] - m_y)
                        return
                    for i, obs in enumerate(self.obstacles):
                        if math.hypot(m_x - obs['x'], m_y - obs['y']) < obs['r'] + 0.05:
                            self.dragging = i
                            self.drag_offset = (obs['x'] - m_x, obs['y'] - m_y)
                            return
                            
                    # Если кликнули по пустому месту — начинаем рисовать зону
                    self.drawing_zone_start = (m_x, m_y)
                    self.current_drawing_end = (m_x, m_y)

            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:
                    if self.drawing_zone_start is not None:
                        x1, y1 = self.drawing_zone_start
                        x2, y2 = self.current_drawing_end
                        # Если потянули мышь (создали область)
                        if abs(x2 - x1) > 0.05 and abs(y2 - y1) > 0.05:
                            self.exclusion_zone = (min(x1, x2), max(x1, x2), min(y1, y2), max(y1, y2))
                        else:
                            self.exclusion_zone = None # Простой клик по полю удаляет зону
                        self.drawing_zone_start = None
                        self.current_drawing_end = None
                    self.dragging = None

            elif event.type == pygame.MOUSEMOTION:
                # Рисуем рамку
                if self.drawing_zone_start is not None:
                    mx, my = pygame.mouse.get_pos()
                    self.current_drawing_end = self.to_meters(mx, my)
                # Или перетаскиваем объект
                elif self.dragging is not None:
                    mx, my = pygame.mouse.get_pos()
                    m_x, m_y = self.to_meters(mx, my)
                    new_x = m_x + self.drag_offset[0]
                    new_y = m_y + self.drag_offset[1]
                    if self.dragging == 'our':
                        self.our_robot_pose[0] = new_x
                        self.our_robot_pose[1] = new_y
                    elif self.dragging == 'enemy':
                        self.enemy_robot_pose[0] = new_x
                        self.enemy_robot_pose[1] = new_y
                    elif isinstance(self.dragging, int):
                        self.obstacles[self.dragging]['x'] = new_x
                        self.obstacles[self.dragging]['y'] = new_y

    def publish_state(self):
        pose_msg = Pose2D()
        pose_msg.x = self.our_robot_pose[0]
        pose_msg.y = self.our_robot_pose[1]
        pose_msg.theta = self.our_robot_pose[2]
        self.pub_our_pose.publish(pose_msg)

        enemy_msg = Pose2D()
        enemy_msg.x = self.enemy_robot_pose[0]
        enemy_msg.y = self.enemy_robot_pose[1]
        enemy_msg.theta = self.enemy_robot_pose[2]
        self.pub_enemy_pose.publish(enemy_msg)

        obs_array = PoseArray()
        obs_array.header = Header()
        obs_array.header.stamp = self.get_clock().now().to_msg()
        obs_array.header.frame_id = "map"
        
        for obs in self.obstacles:
            # Пропускаем публикацию, если объект внутри зоны исключения
            if self.exclusion_zone:
                min_x, max_x, min_y, max_y = self.exclusion_zone
                if min_x <= obs['x'] <= max_x and min_y <= obs['y'] <= max_y:
                    continue
                    
            p = Pose()
            p.position.x = obs['x']
            p.position.y = obs['y']
            p.position.z = 0.0
            p.orientation.w = 1.0
            obs_array.poses.append(p)
            
        self.pub_obstacles.publish(obs_array)

    def game_loop(self):
        self.handle_events()
        self.update_physics()
        self.screen.fill((50, 50, 50)) 
        self.draw_grid_and_axes()
        
        # Рисуем квадрат исключения
        self.draw_exclusion_zone()
        
        self.draw_path()
        self.draw_goal()
        
        for obs in self.obstacles:
            self.draw_obstacle(obs, is_held=False)
            
        if self.held_object is not None:
            self.draw_obstacle(self.held_object, is_held=True)
            
        self.draw_robot(*self.our_robot_pose, BLUE, is_us=True)
        self.draw_robot(*self.enemy_robot_pose, RED, is_us=False)
        
        pygame.display.flip()
        self.publish_state()

def main(args=None):
    rclpy.init(args=args)
    node = Simple2DSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        pygame.quit()

if __name__ == '__main__':
    main()
