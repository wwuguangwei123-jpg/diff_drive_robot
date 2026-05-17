import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from .pid_controller import PID
import time

class DiffDrivePID(Node):
    def __init__(self):
        super().__init__('diff_drive_pid')
        # 创建订阅 /cmd_vel
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        # 发布左右轮速度
        self.pub_right = self.create_publisher(Float32, '/right_wheel_speed', 10)
        self.pub_left = self.create_publisher(Float32, '/left_wheel_speed', 10)
        # PID 控制器参数
        self.pid_right = PID(1.0, 0.0, 0.0)
        self.pid_left = PID(1.0, 0.0, 0.0)
        self.last_time = time.time()
        # 机器人参数
        self.L = 0.3  # 轮距 m

    def cmd_callback(self, msg):
        now = time.time()
        dt = now - self.last_time if self.last_time else 0.1
        self.last_time = now

        # 期望线速度和角速度
        V = msg.linear.x
        omega = msg.angular.z

        # 目标轮速度
        Vr_target = V + omega * self.L / 2
        Vl_target = V - omega * self.L / 2

        # 假设当前轮速为0（模拟）
        Vr_current = 0.0
        Vl_current = 0.0

        # PID 计算控制量
        Vr_output = self.pid_right.update(Vr_target, Vr_current, dt)
        Vl_output = self.pid_left.update(Vl_target, Vl_current, dt)

        # 发布轮速
        self.pub_right.publish(Float32(data=Vr_output))
        self.pub_left.publish(Float32(data=Vl_output))

        self.get_logger().info(f'Vr: {Vr_output:.2f}, Vl: {Vl_output:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = DiffDrivePID()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
