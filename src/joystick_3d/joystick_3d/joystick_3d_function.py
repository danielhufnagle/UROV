import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class Joy3D(Node):

    def __init__(self):
        super().__init__('Joy3D')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.subscription

    def joy_callback(self, msg):
        twist = Twist()
        # need to be able to move the robot in 3 dimensions

        # joystick axes
        # 0 - LEFTX
        # 1 - LEFTY
        # 2 - RIGHTX
        # 3 - RIGHTY
        # 4 - TRIGGERLEFT
        # 5 - TRIGGERRIGHT

        twist.linear.x = msg.axes[0] * 2.0 # move left and right moving left joystick left and right
        #twist.linear.y = msg.axes[] * 2.0 # move up and down
        twist.linear.z = msg.axes[1] * 2.0 # move forward and backwards
        twist.angular.x = msg.axes[3] * 2.0 # tilt up and down
        twist.angular.y = msg.axes[2] * 2.0 # look left and right
        #twist.angular.z = msg.axes[] * 2.0 # roll side to side
        self.publisher_.publish(twist)
        self.get_logger().info(f'Publishing twist [{twist.linear.x}, {twist.linear.y}, {twist.linear.z}, {twist.angular.x}, {twist.angular.y}, {twist.angular.z}]')

def main(args=None):
    rclpy.init(args=args)

    joy3d = Joy3D()

    rclpy.spin(joy3d)
    
    rclpy.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()