#include "rclcpp/rclcpp.hpp"
#include "PACKAGE_NAME/msg/MESSAGE_TYPE.hpp"
#include <chrono>
#include <functional>

class PerceptionNode : public rclcpp::Node
{
public:
    PerceptionNode() : Node("perception_node")
    {
       

        // Subscriber
        subscriber_ = this->create_subscription<PACKAGE_NAME::msg::MESSAGE_TYPE>(
            "input_topic",
            10,
            std::bind(&PerceptionNode::callback, this, std::placeholders::_1));

        // Publisher
        publisher_ = this->create_publisher<PACKAGE_NAME::msg::MESSAGE_TYPE>(
            "output_topic",
            10);

        // Timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PerceptionNode::command_loop, this));

        RCLCPP_INFO(this->get_logger(), "Node Started");
    }

private:
    void callback(const PACKAGE_NAME::msg::MESSAGE_TYPE::SharedPtr msg)
    {
        /es
    }

    void command_loop()
    {
        PACKAGE_NAME::msg::MESSAGE_TYPE command;

        

        publisher_->publish(command);
    }

    rclcpp::Subscription<PACKAGE_NAME::msg::MESSAGE_TYPE>::SharedPtr subscriber_;
    rclcpp::Publisher<PACKAGE_NAME::msg::MESSAGE_TYPE>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NodeName>());
    rclcpp::shutdown();
    return 0;
}