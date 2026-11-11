#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int thresh = 100; 
RNG rng(12345);

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
    
    cv::blur(gray, gray, cv::Size(3,3));
    imshow("gray", gray);


    cv::imshow(window_name_, imageHSV);
    cv::inRange(imageHSV, cv::Scalar(10, 50, 40), cv::Scalar(20, 255, 100), output);
    cv::Mat kernal = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7,7));
    cv::morphologyEx(output, denoised, cv::MORPH_CLOSE, kernal);
    // Update GUI Window
    cv::imshow("HSV", output);
    cv::imshow("denoised", denoised);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
    thresh_callback(0,0,gray);
  }
void thresh_callback(int, void*,Mat gray )
{
    Mat canny_output;
    Canny( gray, canny_output, thresh, thresh*2 );
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours( canny_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE );
    Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
    for( size_t i = 0; i< contours.size(); i++ )
    {
        Scalar color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
        drawContours( drawing, contours, (int)i, color, 2, LINE_8, hierarchy, 0 );
    }
    imshow( "Contours", drawing );
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