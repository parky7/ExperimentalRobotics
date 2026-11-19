#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <chrono>
#include <functional>

class JoystickTeleop : public rclcpp::Node
{
public:
    JoystickTeleop() : Node("joystick_teleop")
    {
        linear_velocity_ = 0.0;
        angular_velocity_ = 0.0;

        joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&JoystickTeleop::joy_callback, this, std::placeholders::_1));

        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&JoystickTeleop::command_loop, this));

        RCLCPP_INFO(this->get_logger(), "Joystick Teleoperation Started");
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        linear_velocity_ = msg->axes[1];
        angular_velocity_ =   msg->axes[0];

        //RCLCPP_INFO(this->get_logger(), "Linear: %.2f Angular: %.2f", linear_velocity_, angular_velocity_);
    }

    void command_loop()
    {
        geometry_msgs::msg::Twist command;

        command.linear.x = linear_velocity_;
        command.angular.z = angular_velocity_;

        cmd_vel_publisher_->publish(command);
    }   

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    double linear_velocity_;
    double angular_velocity_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoystickTeleop>());
    rclcpp::shutdown();
    return 0;
}