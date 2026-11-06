#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>

#include <functional>

class PerceptionNode : public rclcpp::Node
{
public:
    PerceptionNode() : Node("perception_node")
    {
        image_subscriber= this->create_subscription<sensor_msgs::msg::Image>(
            "image_raw",
            10,
            std::bind(&PerceptionNode::image_callback, this, std::placeholders::_1));

        point_publisher = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "object_point",
            10);

        RCLCPP_INFO(this->get_logger(), "Perception Node Started");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_publisher;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PerceptionNode>());
    rclcpp::shutdown();
    return 0;
}