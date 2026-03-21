import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from gpiozero import AngularServo
from gpiozero.pins.lgpio import LGPIOFactory
import threading

class ArmController(Node):
    """
    ROS2 узел для управления двухосевым манипулятором.
    Использует gpiozero для управления сервоприводами.
    """
    
    def __init__(self):
        super().__init__('arm_controller_node')
        
        # --- Параметры сервоприводов ---
        # Эти значения нужно подобрать под ваши сервоприводы
        self.declare_parameter('servo_arm_pin', 13)
        self.declare_parameter('servo_gripper_pin', 12)
        self.declare_parameter('servo_min_angle', 0)
        self.declare_parameter('servo_max_angle', 180)
        self.declare_parameter('arm_up_angle', 180)    # Угол для поднятой руки
        self.declare_parameter('arm_down_angle', 0)     # Угол для опущенной руки
        self.declare_parameter('gripper_open_angle', 180)  # Угол для открытого захвата
        self.declare_parameter('gripper_closed_angle', 0)  # Угол для закрытого захвата
        
        # Получаем параметры
        arm_pin = self.get_parameter('servo_arm_pin').value
        gripper_pin = self.get_parameter('servo_gripper_pin').value
        self.arm_up = self.get_parameter('arm_up_angle').value
        self.arm_down = self.get_parameter('arm_down_angle').value
        self.gripper_open = self.get_parameter('gripper_open_angle').value
        self.gripper_closed = self.get_parameter('gripper_closed_angle').value
        
        # --- Инициализация gpiozero с LGPIO для Pi 5 ---
        try:
            pin_factory = LGPIOFactory()
            
            # Инициализация сервоприводов
            self.servo_arm = AngularServo(
                arm_pin,
                min_angle=0, max_angle=180,
                min_pulse_width=0.0005,
                max_pulse_width=0.0025,
                pin_factory=pin_factory
            )
            
            self.servo_gripper = AngularServo(
                gripper_pin,
                min_angle=0, max_angle=180,
                min_pulse_width=0.0005,
                max_pulse_width=0.0025,
                pin_factory=pin_factory
            )
            
            # Устанавливаем начальные положения
            self.servo_arm.angle = self.arm_down
            self.servo_gripper.angle = self.gripper_closed
            
            self.get_logger().info(
                f'GPIOzero инициализирован успешно.\n'
                f'  Серво рука: GPIO{arm_pin}\n'
                f'  Серво захват: GPIO{gripper_pin}'
            )
            
        except Exception as e:
            self.get_logger().error(f'Ошибка инициализации GPIO: {e}')
            return
        
        # --- Состояние манипулятора ---
        self.arm_state = False  # False - опущено, True - поднято
        self.gripper_state = False  # False - закрыто, True - открыто
        
        # --- Подписка на топики ---
        self.sub_arm = self.create_subscription(
            Bool,
            '/arm/state1',
            self.arm_callback,
            10)
        
        self.sub_gripper = self.create_subscription(
            Bool,
            '/arm/state2',
            self.gripper_callback,
            10)
        
        # --- Таймер для проверки состояния (опционально) ---
        self.create_timer(1.0, self.status_timer_callback)
        
        self.get_logger().info('Узел управления манипулятором запущен и готов к работе.')
    
    def arm_callback(self, msg):
        """Обработчик команд для руки (поднятие/опускание)"""
        if msg.data != self.arm_state:
            angle = self.arm_up if msg.data else self.arm_down
            self.servo_arm.angle = angle
            
            self.arm_state = msg.data
            state_str = "ПОДНЯТО" if msg.data else "ОПУЩЕНО"
            self.get_logger().info(f'Рука: {state_str} (угол {angle}°)')
        else:
            self.get_logger().debug(f'Рука уже в нужном положении')
    
    def gripper_callback(self, msg):
        """Обработчик команд для захвата (открытие/закрытие)"""
        if msg.data != self.gripper_state:
            angle = self.gripper_open if msg.data else self.gripper_closed
            self.servo_gripper.angle = angle
            
            self.gripper_state = msg.data
            state_str = "ОТКРЫТО" if msg.data else "ЗАКРЫТО"
            self.get_logger().info(f'Захват: {state_str} (угол {angle}°)')
        else:
            self.get_logger().debug(f'Захват уже в нужном положении')
    
    def status_timer_callback(self):
        """Периодическое информирование о состоянии"""
        self.get_logger().info(
            f'Текущее состояние - Рука: {"↑" if self.arm_state else "↓"}, '
            f'Захват: {"↗" if self.gripper_state else "↘"}',
            throttle_duration_sec=5.0  # Логировать не чаще чем раз в 5 секунд
        )
    
    def destroy_node(self):
        """Корректное завершение работы"""
        self.get_logger().info('Завершение работы, отключение сервоприводов...')
        
        # Возвращаем сервоприводы в безопасное положение (опущено/закрыто)
        if hasattr(self, 'servo_arm'):
            self.servo_arm.angle = self.arm_down
        if hasattr(self, 'servo_gripper'):
            self.servo_gripper.angle = self.gripper_closed
            
        # Небольшая задержка, чтобы сервоприводы успели доехать
        import time
        time.sleep(0.5)
        
        # Отключаем сервоприводы (снимаем сигнал)
        if hasattr(self, 'servo_arm'):
            self.servo_arm.detach()
        if hasattr(self, 'servo_gripper'):
            self.servo_gripper.detach()
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    arm_controller = ArmController()
    
    try:
        rclpy.spin(arm_controller)
    except KeyboardInterrupt:
        arm_controller.get_logger().info('Узел остановлен пользователем (Ctrl+C)')
    finally:
        arm_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
