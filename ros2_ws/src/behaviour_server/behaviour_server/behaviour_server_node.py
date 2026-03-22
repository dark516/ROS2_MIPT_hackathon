#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, PoseArray
from std_msgs.msg import Bool
import math

# Константы
ENEMY_SAFETY_DISTANCE = 0.35

class BehaviourServer(Node):
    def __init__(self):
        super().__init__('behaviour_server_node')
        
        # Publishers
        self.arm_pub = self.create_publisher(Bool, '/robot/arm', 10)
        self.goal_pub = self.create_publisher(Pose2D, '/goal', 10)
        
        # Subscribers
        self.robot_pose_sub = self.create_subscription(Pose2D, '/robot/pose', self.robot_pose_callback, 10)
        self.obstacles_sub = self.create_subscription(PoseArray, '/obstacles', self.obstacles_callback, 10)
        self.enemy_pose_sub = self.create_subscription(Pose2D, '/enemy/pose', self.enemy_pose_callback, 10)
        self.base_pose_sub = self.create_subscription(Pose2D, '/base', self.base_pose_callback, 10)
        
        # Подписчик на статус достижения цели
        self.goal_reached_sub = self.create_subscription(Bool, '/goal_reached', self.goal_reached_callback, 10)
        
        # Переменные состояния
        self.current_robot_pose = None
        self.current_obstacles = []
        self.enemy_pose = None
        self.base_pose = None
        
        # События от фолловера
        self.has_new_goal_event = False
        self.last_goal_event_value = False
        
        # State Machine
        self.sm_state = "INIT"
        self.wait_start_time = None
        
        self.get_logger().info("Behaviour Server Started.")
        self.state_machine_timer = self.create_timer(0.1, self.state_machine_loop)

    def robot_pose_callback(self, msg):
        self.current_robot_pose = msg

    def obstacles_callback(self, msg):
        self.current_obstacles = []
        for p in msg.poses:
            self.current_obstacles.append({'x': p.position.x, 'y': p.position.y})

    def enemy_pose_callback(self, msg):
        self.enemy_pose = msg

    def base_pose_callback(self, msg):
        self.base_pose = msg

    def goal_reached_callback(self, msg):
        # Просто фиксируем последнее пришедшее сообщение
        self.last_goal_event_value = msg.data
        self.has_new_goal_event = True

    def find_nearest_object(self):
        if not self.current_robot_pose or not self.current_obstacles:
            return None

        nearest_obj = None
        min_dist = float('inf')

        for obj in self.current_obstacles:
            dist = math.hypot(self.current_robot_pose.x - obj['x'], self.current_robot_pose.y - obj['y'])
            if dist < min_dist:
                min_dist = dist
                nearest_obj = obj
        
        return nearest_obj

    def is_enemy_too_close(self):
        if not self.enemy_pose or not self.current_robot_pose: 
            return False
        dist = math.hypot(self.current_robot_pose.x - self.enemy_pose.x, self.current_robot_pose.y - self.enemy_pose.y)
        return dist < ENEMY_SAFETY_DISTANCE

    def state_machine_loop(self):
        if not self.current_robot_pose:
            return # Ждем пока появятся координаты робота

        # Базовая защита от столкновения с противником
        if self.is_enemy_too_close():
            self.get_logger().warn("Враг слишком близко! Приостанавливаю логику.", throttle_duration_sec=2.0)
            return

        # --- Конечный автомат (State Machine) ---
        
        if self.sm_state == "INIT":
            self.get_logger().info("Инициализация: Открываю манипулятор.")
            self.arm_pub.publish(Bool(data=True)) # Открыть манипулятор
            self.sm_state = "SEARCHING"

        elif self.sm_state == "SEARCHING":
            nearest_obj = self.find_nearest_object()
            if nearest_obj:
                self.get_logger().info(f"Нашел объект: x={nearest_obj['x']:.2f}, y={nearest_obj['y']:.2f}")
                
                # Очищаем очередь событий перед отправкой команды
                self.has_new_goal_event = False
                
                # Отправляем цель
                goal_msg = Pose2D(x=nearest_obj['x'], y=nearest_obj['y'], theta=0.0)
                self.goal_pub.publish(goal_msg)
                
                # Переходим в режим ОЖИДАНИЯ ПУТИ
                self.sm_state = "WAIT_PATH_OBJ"
            else:
                self.get_logger().info("Объекты не найдены. Жду...", throttle_duration_sec=3.0)

        elif self.sm_state == "WAIT_PATH_OBJ":
            # Ждем ТОЛЬКО False (фолловер получил путь и начал движение)
            if self.has_new_goal_event and not self.last_goal_event_value:
                self.has_new_goal_event = False # Сбрасываем событие
                self.get_logger().info("Путь до объекта построен. Едем!")
                self.sm_state = "MOVING_TO_OBJ"

        elif self.sm_state == "MOVING_TO_OBJ":
            # Ждем ТОЛЬКО True (фолловер приехал на место)
            if self.has_new_goal_event and self.last_goal_event_value:
                self.has_new_goal_event = False # Сбрасываем событие
                self.get_logger().info("Доехал до объекта. Хватаю!")
                self.arm_pub.publish(Bool(data=False)) # Схватить (закрыть манипулятор)
                self.wait_start_time = self.get_clock().now()
                self.sm_state = "GRABBING"

        elif self.sm_state == "GRABBING":
            # Ждем полсекунды
            elapsed = (self.get_clock().now() - self.wait_start_time).nanoseconds / 1e9
            if elapsed >= 0.5:
                if not self.base_pose:
                    self.get_logger().warn("Координаты базы еще не получены! Жду топик /base.", throttle_duration_sec=2.0)
                    return
                
                self.get_logger().info(f"Объект схвачен. Везем на базу ({self.base_pose.x:.2f}, {self.base_pose.y:.2f}).")
                
                # Очищаем очередь событий перед отправкой команды
                self.has_new_goal_event = False
                
                # Отправляем цель на базу
                goal_msg = Pose2D(x=self.base_pose.x, y=self.base_pose.y, theta=0.0)
                self.goal_pub.publish(goal_msg)
                
                # Переходим в режим ОЖИДАНИЯ ПУТИ
                self.sm_state = "WAIT_PATH_BASE"

        elif self.sm_state == "WAIT_PATH_BASE":
            # Ждем ТОЛЬКО False
            if self.has_new_goal_event and not self.last_goal_event_value:
                self.has_new_goal_event = False
                self.get_logger().info("Путь до базы построен. Едем!")
                self.sm_state = "MOVING_TO_BASE"

        elif self.sm_state == "MOVING_TO_BASE":
            # Ждем ТОЛЬКО True
            if self.has_new_goal_event and self.last_goal_event_value:
                self.has_new_goal_event = False
                self.get_logger().info("Прибыл на базу. Выбрасываю объект!")
                self.arm_pub.publish(Bool(data=True)) # Бросить (открыть манипулятор)
                self.wait_start_time = self.get_clock().now()
                self.sm_state = "DROPPING"

        elif self.sm_state == "DROPPING":
            # Ждем полсекунды
            elapsed = (self.get_clock().now() - self.wait_start_time).nanoseconds / 1e9
            if elapsed >= 0.5:
                self.get_logger().info("Готов к следующему объекту.")
                self.sm_state = "SEARCHING" # Возвращаемся в начало цикла

def main(args=None):
    rclpy.init(args=args)
    node = BehaviourServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
