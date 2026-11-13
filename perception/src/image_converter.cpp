#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <memory>

#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/exceptions.hpp>

class ImageConverter : public rclcpp::Node
{
private:
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  image_transport::Subscriber depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  std::string window_name_;
  cv::Mat depth;
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<
    geometry_msgs::msg::PointStamped
  >::SharedPtr object_pub_;

  void publishObjectInBase(
    double X,
    double Y,
    double Z,
    const std_msgs::msg::Header &image_header)
  {
    geometry_msgs::msg::PointStamped camera_point;

    camera_point.header = image_header;

    camera_point.point.x= X;
    camera_point.point.y= Y;
    camera_point.point.z = Z;

    try{
      geometry_msgs::msg::PointStamped robot_point;

      tf_buffer_->transform(
        camera_point,
        robot_point,
        "base_link"
      );

      object_pub_->publish(robot_point);

      RCLCPP_INFO(
        this->get_logger(),
        "Object in base_link: X=%.3f Y=%.3f Z=%.3f",
        robot_point.point.x,
        robot_point.point.y,
        robot_point.point.z
      );

    }catch(const tf2::TransformException &ex){
      RCLCPP_WARN(
        this->get_logger(),
        "TF2 failed: %s",
        ex.what()
      );
    }
  }

public:
  ImageConverter()
  : Node("image_converter")
  {
    window_name_ = "Image window";
    cv::namedWindow(window_name_);

    image_sub_ = image_transport::create_subscription(
      this,
      "/camera/camera/color/image_raw",
      std::bind(&ImageConverter::imageCb, this, std::placeholders::_1),
      "raw");

    image_pub_ = image_transport::create_publisher(this, "/image_converter/output_video");
    
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera/camera/color/camera_info", 10, 
      std::bind(&ImageConverter::cameraCB, this, std::placeholders::_1));

    depth_sub_ = image_transport::create_subscription(
      this,
      "/camera/camera/aligned_depth_to_color/image_raw",
      std::bind(&ImageConverter::depthCB, this, std::placeholders::_1), "raw");
    
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(
        this->get_clock()
      );

      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
        *tf_buffer_
      );

      object_pub_ = this->create_publisher<
        geometry_msgs::msg::PointStamped
      >("/object_position", 10);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(window_name_);
  }

  void cameraCB(const sensor_msgs::msg::CameraInfo::SharedPtr msg){
    fx = msg->k[0];
    fy = msg->k[4];
    cx = msg->k[2];
    cy = msg->k[5];
  }

  void depthCB(const sensor_msgs::msg::Image::ConstSharedPtr msg){
    cv_bridge::CvImagePtr cv_ptr;
    try{
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
      depth = cv_ptr->image;
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
  }

  void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat imageHSV, output, denoised;
    cv::cvtColor(cv_ptr->image, imageHSV, cv::COLOR_BGR2HSV);
    cv::inRange(imageHSV, cv::Scalar(0, 35, 15), cv::Scalar(25, 255, 95), output);
    
    cv::Mat kernal = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5));
    cv::morphologyEx(output, denoised, cv::MORPH_CLOSE, kernal);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    std::vector<cv::Point2i> center;
    std::vector<int> radius;
    
    cv::findContours(denoised.clone(), contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

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
    if(count > 0){
      cv::circle(cv_ptr->image, center[max], radius[max], cv::Scalar(255,10,10), 3);
      cv::circle(cv_ptr->image, center[max], 5, cv::Scalar(0,0,255), cv::FILLED);
      
      std::string text = "Center point:(" + std::to_string(center[max].x) + ", " + std::to_string(center[max].y) + ")";
      cv::putText(cv_ptr->image, text, cv::Point(5,15), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,255,0), 2);
      
      if(!depth.empty() && fx > 0 && fy > 0){
        double depthSum = 0;
        int counter = 0;
        
        int u = center[max].x; // column coordinate
        int v = center[max].y; // row coordinate

        // Sample a 9x9 neighborhood around the center point
        for(int i = -4; i <= 4; i++){
          for(int j = -4; j <= 4; j++){
            int ni = v + i; // row index
            int nj = u + j; // column index

            // Ensure we are inside the image bounds
            if(ni >= 0 && ni < depth.rows && nj >= 0 && nj < depth.cols){
              uint16_t d_val = depth.at<uint16_t>(ni, nj);
              if(d_val != 0){
                depthSum += d_val;
                counter++;
              }
            }
          }
        }
        
        if(counter > 0){
          double meanDepth = depthSum / 1000 / counter;
          std::cout << "Depth: " << meanDepth << std::endl;

          double X = (u - cx) * meanDepth / fx;
          double Y = (v - cy) * meanDepth / fy;

          
          std::string depth_text = "X: " + std::to_string(X) + " Y: " + std::to_string(Y);
          cv::putText(cv_ptr->image, depth_text, cv::Point(5, 35), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 0), 2);

          publishObjectInBase(X,Y,meanDepth,msg->header);
        }
      }
    }

    cv::imshow(window_name_, imageHSV);
    cv::imshow("rbg", cv_ptr->image);
    cv::imshow("denoised", denoised);
    cv::waitKey(3);

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