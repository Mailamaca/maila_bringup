import math
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/ackermann_cmd', 10)
        timer_period = 1/100  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
    def timer_callback(self):
        self.i += 1
        msg = AckermannDriveStamped()
        msg.header.stamp = Clock().now().to_msg()
        msg.drive.steering_angle = math.radians(15) # * math.sin(self.i/100)
        msg.drive.speed = 0.5 # * math.cos(self.i/100)
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.drive.steering_angle)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
