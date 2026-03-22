import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose2D
import math

class GoalExtractorNode(Node):
    def __init__(self):
        super().__init__('goal_extractor_node')
        
        # Подписчик на массив препятствий
        self.subscription = self.create_subscription(
            PoseArray,
            '/obstacles',
            self.listener_callback,
            10
        )
        
        # Издатель для новой цели
        self.publisher_ = self.create_publisher(
            Pose2D, 
            '/goal', 
            10
        )
        self.get_logger().info('Узел Goal Extractor запущен и ожидает данные...')

    def listener_callback(self, msg):
        # Проверяем, что массив не пустой
        if not msg.poses:
            self.get_logger().warn('Получен пустой массив PoseArray')
            return

        # Берем первый элемент
        first_pose = msg.poses[0]

        # Создаем сообщение Pose2D
        goal_msg = Pose2D()
        goal_msg.x = first_pose.position.x
        goal_msg.y = first_pose.position.y

        # Извлекаем yaw (theta) из кватерниона (orientation)
        q = first_pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        goal_msg.theta = math.atan2(siny_cosp, cosy_cosp)

        # Публикуем в /goal
        self.publisher_.publish(goal_msg)
        self.get_logger().info(
            f'Опубликована цель: x={goal_msg.x:.2f}, y={goal_msg.y:.2f}, theta={goal_msg.theta:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = GoalExtractorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
