#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

class TeleopTurtle : public rclcpp::Node {
public:
    TeleopTurtle(): Node("teleop_turtle") {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);

        // callback function to process joystick input and publish velocity commands when joystick messages are received
        auto joy_callback = [this](sensor_msgs::msg::Joy::UniquePtr msg) -> void {
            RCLCPP_INFO(this->get_logger(), "Received joystick input: axes[0]=%f, axes[1]=%f", msg->axes[0], msg->axes[1]);
            auto twist = geometry_msgs::msg::Twist();
            twist.linear.x = msg->axes[1];  // Forward/backward
            twist.angular.z = msg->axes[0]; // Left/right
            publisher_->publish(twist);
        };
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, joy_callback);
    }
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
}

int main(int argc char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopTurtle>());
    rclcpp::shutdown();
    return 0;
}