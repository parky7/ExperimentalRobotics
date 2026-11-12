#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <algorithm>
#include <cmath>


// int thresh = 100; 
// RNG rng(12345);

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


    cv::Mat imageHSV, output, denoised, gray, drawing;
    cv::cvtColor(cv_ptr->image, imageHSV, CV_BGR2HSV);
    cv::cvtColor(cv_ptr->image, output, CV_BGR2HSV);
    cv::cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);
    


    cv::imshow(window_name_, imageHSV); // This shows the image before filtering
    cv::inRange(imageHSV, cv::Scalar(0, 35, 15), cv::Scalar(25, 255, 95), output);
    cv::Mat kernal = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5)); // denoise
    cv::morphologyEx(output, denoised, cv::MORPH_CLOSE, kernal);

    // variables for contour
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    std::vector<cv::Point2i> center;
    std::vector<int> radius;
    
    cv::findContours(denoised.clone(), contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

    size_t count = contours.size();

    size_t max = 0;
    for(size_t i = 0; i < count; i++){
      cv::Point2f c;
      float r;
      cv::minEnclosingCircle(contours[i], c, r);

      if(r >= 30){
        center.push_back(c);
        radius.push_back(r);
        if(radius.at(radius.size() - 1) > radius.at(max)){
          max = radius.size() - 1;
        }
      }
    }
    
    count = center.size();
    // Find the bigest radus circle and only draw one
    if(count > 0){
      cv::circle(cv_ptr->image, center[max], radius[max], cv::Scalar(255,10,10), 3);
      std::string text = "Center point:(" + std::to_string(center[max].x) + ", " + std::to_string(center[max].y) + ")";
      cv::putText(cv_ptr->image, text, cv::Point(5,15), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,255,0), 2);
    }
    
    // Draw all circles
    // for(size_t i = 0; i < count; i++){
    //   cv::circle(cv_ptr->image, center[i], radius[i], cv::Scalar(255,10,10), 3);
    // }


    // Update GUI Window
    cv::imshow("rbg", cv_ptr->image);
    cv::imshow("denoised", denoised);
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