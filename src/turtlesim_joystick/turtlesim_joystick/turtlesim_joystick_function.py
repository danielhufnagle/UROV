# test this function
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class TeleopTurtle(Node):

    def __init__(self):
        super().__init__('teleop_turtle')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.subscription  # prevent unused variable warning

    def joy_callback(self, msg):
        twist = Twist()
        twist.linear.x = msg.axes[1] * 2.0  # Scale linear velocity
        twist.angular.z = msg.axes[0] * 2.0  # Scale angular velocity
        self.publisher_.publish(twist)
        self.get_logger().info(f'Publishing: linear.x={twist.linear.x}, angular.z={twist.angular.z}')

def main(args=None):
    rclpy.init(args=args)

    teleop_turtle = TeleopTurtle()

    rclpy.spin(teleop_turtle)

    teleop_turtle.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()