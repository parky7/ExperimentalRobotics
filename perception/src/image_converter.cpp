#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

class ImageConverter : public rclcpp::Node
{
private:
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  std::string window_name_;

public:
  ImageConverter()
  : Node("image_converter")
  {
    window_name_ = "Image window";
    cv::namedWindow(window_name_);

    // Subscribe to input video feed and publish output video feed using image_transport
    image_sub_ = image_transport::create_subscription(
      this,
      "/camera/camera/color/image_raw",
      std::bind(&ImageConverter::imageCb, this, std::placeholders::_1),
      "raw");

    image_pub_ = image_transport::create_publisher(this, "/image_converter/output_video");
  }

  ~ImageConverter()
  {
    cv::destroyWindow(window_name_);
  }

  void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv_bridge::CvImagePtr cv_ptr_out;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv_ptr_out = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }


    cv::Mat imageHSV;
    cv::Mat output;
    cv::cvtColor(cv_ptr->image, imageHSV, CV_BGR2HSV);
    cv::cvtColor(cv_ptr->image, output, CV_BGR2HSV);
    cv::Scalar lowerColor = (40, 40, 0);
    cv::Scalar upperColor = (150, 150, 30);

    cv::imshow(window_name_, imageHSV);

    cv::inRange(imageHSV, lowerColor, upperColor, output);

    // Update GUI Window
    cv::imshow("HSV", output);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageConverter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}