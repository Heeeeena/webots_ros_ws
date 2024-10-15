import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
import random
import time

class RandomWalkNode(Node):
    def __init__(self):
        super().__init__('random_walk')

        # 订阅 TOF 传感器数据
        self.tof_subscriber = self.create_subscription(Range, '/tof', self.tof_callback, 10)
        self.ps0_subscriber = self.create_subscription(Range, '/ps0', self.ps0_message, 10)
        self.ps7_subscriber = self.create_subscription(Range, '/ps7', self.ps7_message, 10)
        self.ps1_subscriber = self.create_subscription(Range, '/ps1', self.ps1_message, 10)
        self.ps6_subscriber = self.create_subscription(Range, '/ps6', self.ps6_message, 10)

        self.ps0_value = 0
        self.ps7_value = 0
        self.ps1_value = 0
        self.ps6_value = 0
        self.obstacle_detected = False

        self.__now = self.get_clock().now().to_msg()
        # 创建发布器，用于发布速度控制命令
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # 初始化随机数生成器
        random.seed()

        # 初始化一个标志位，表示机器人是否遇到了障碍物
        self.walk_or_rotate = False

        # 定时器用于持续调用 move_robot 函数
        self.timer = self.create_timer(2, self.check_obstacle)


        self.ps0_value = 0


    def ps0_message(self, msg):
        self.ps0_value = msg.range
        self.__now = msg.header.stamp

    def ps7_message(self, msg):
        self.ps7_value = msg.range

    def ps1_message(self, msg):
        self.ps1_value = msg.range

    def ps6_message(self, msg):
        self.ps6_value = msg.range


    def tof_callback(self, msg):
        """TOF 传感器的回调函数，用于检测障碍物"""
        # 如果检测到障碍物 (距离小于0.5米)
        if msg.range < 0.1 or self.ps0_value < 0.04 or self.ps7_value <0.04 or self.ps1_value <0.04 or self.ps6_value <0.04:
            self.get_logger().info(f'Obstacle detected at {msg.range} meters')
            self.obstacle_detected = True
            
        else:
            self.obstacle_detected = False
                
        self.move_robot()
        

    def check_obstacle(self):
        if self.obstacle_detected: 
            self.walk_or_rotate = False
        else:
            self.walk_or_rotate = True


        
    def move_robot(self):
        """根据障碍物检测结果移动机器人"""
        twist_msg = Twist()

        if self.walk_or_rotate:
            if not self.obstacle_detected:
                twist_msg.linear.x = 0.08  # 设置直线前进速度
                self.get_logger().info('Moving forward')
        else:
            twist_msg.angular.z = 0.4  # 设置旋转速度
            
            self.get_logger().info(f'Turning')



        self.velocity_publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)

    # 创建并运行节点
    node = RandomWalkNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 关闭节点
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
