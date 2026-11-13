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
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

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
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr box_pub_;
  double box_thickness_;

  void publishObjectInBase(
    double X,
    double Y,
    double Z,
    double angle,
    double width,
    double height,
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

      geometry_msgs::msg::PoseStamped camera_pose;
      camera_pose.header = image_header;
      camera_pose.pose.position.x = X;
      camera_pose.pose.position.y = Y;
      camera_pose.pose.position.z = Z + box_thickness_ / 2.0;
      tf2::Quaternion rotation;
      rotation.setRPY(0.0, 0.0, angle);
      camera_pose.pose.orientation = tf2::toMsg(rotation);

      geometry_msgs::msg::PoseStamped robot_pose;
      tf_buffer_->transform(camera_pose, robot_pose, "base_link");

      visualization_msgs::msg::Marker box;
      box.header = robot_pose.header;
      box.ns = "detected_box";
      box.id = 0;
      box.type = visualization_msgs::msg::Marker::CUBE;
      box.action = visualization_msgs::msg::Marker::ADD;
      box.pose = robot_pose.pose;
      box.scale.x = width;
      box.scale.y = height;
      box.scale.z = box_thickness_;
      box.color.r = 0.55f;
      box.color.g = 0.27f;
      box.color.b = 0.07f;
      box.color.a = 1.0f;
      box.lifetime.sec = 0;
      box.lifetime.nanosec = 500000000;
      box_pub_->publish(box);

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
    box_thickness_ = this->declare_parameter<double>("box_thickness", 0.08);

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
      box_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/object_box", 10);
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
    
    cv::findContours(denoised.clone(), contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

    int box_contour = -1;
    float max_radius = 30.0f;
    
    for(size_t i = 0; i < contours.size(); i++){
      cv::Point2f c;
      float r;
      cv::minEnclosingCircle(contours[i], c, r);

      if(r > max_radius){
        max_radius = r;
        box_contour = static_cast<int>(i);
      }
    }
    
    if(box_contour >= 0){
      cv::RotatedRect box_rect = cv::minAreaRect(contours[box_contour]);
      cv::Point center(
        static_cast<int>(std::round(box_rect.center.x)),
        static_cast<int>(std::round(box_rect.center.y)));
      cv::Point2f corners[4];
      box_rect.points(corners);
      for(int i = 0; i < 4; i++){
        cv::line(cv_ptr->image, corners[i], corners[(i + 1) % 4], cv::Scalar(255,10,10), 2);
      }
      cv::circle(cv_ptr->image, center, 5, cv::Scalar(0,0,255), cv::FILLED);
      
      std::string text = "Center point:(" + std::to_string(center.x) + ", " + std::to_string(center.y) + ")";
      cv::putText(cv_ptr->image, text, cv::Point(5,15), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,255,0), 2);
      
      if(!depth.empty() && fx > 0 && fy > 0){
        double depthSum = 0;
        int counter = 0;
        
        int u = center.x; // column coordinate
        int v = center.y; // row coordinate

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

          double angle = box_rect.angle * CV_PI / 180.0;
          double width = box_rect.size.width * meanDepth / fx;
          double height = box_rect.size.height * meanDepth / fy;
          publishObjectInBase(X, Y, meanDepth, angle, width, height, msg->header);
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