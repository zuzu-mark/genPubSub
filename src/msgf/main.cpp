#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <opencv2/core/core.hpp>

#include "sensor_msgs/msg/point_cloud2.hpp"

class ImageCombiner : public rclcpp::Node {
public:
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> rgb_sub_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> depth_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr combined_image_pub_;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2> approximate_policy;

  message_filters::Synchronizer<approximate_policy> sync_;

  ImageCombiner()
      : Node("image_combiner_node"),
        sync_(approximate_policy(10), rgb_sub_, depth_sub_) {
    rgb_sub_.subscribe(this, "/velodyne_points");
    depth_sub_.subscribe(this, "/velodyne_points2");


    combined_image_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/output_image", 10);

    sync_.registerCallback(&ImageCombiner::topic_callback, this);
  }

public:
  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr rgb_image, const sensor_msgs::msg::PointCloud2::SharedPtr depth_image) {
    // Convert ROS Image messages to OpenCV Mat

#if 0
    cv::Mat rgb_cv_image = cv_bridge::toCvShare(rgb_image, "bgr8")->image;
    cv::Mat depth_cv_image = cv_bridge::toCvShare(depth_image, "16UC1")->image;

    // Normalize depth image for visualization
    cv::normalize(depth_cv_image, depth_cv_image, 0, 255, cv::NORM_MINMAX, CV_8UC1);

    // Convert normalized 8-bit depth image to 3-channel for visualization
    cv::cvtColor(depth_cv_image, depth_cv_image, cv::COLOR_GRAY2BGR);

    // Combine the images side by side
    cv::Mat combined_cv_image;
    cv::hconcat(rgb_cv_image, depth_cv_image, combined_cv_image);

    // Convert combined OpenCV Mat to ROS Image message
    sensor_msgs::msg::Image::SharedPtr combined_image = cv_bridge::CvImage(rgb_image->header, "bgr8", combined_cv_image).toImageMsg();
    combined_image_pub_->publish(*combined_image);
#endif
    RCLCPP_INFO(this->get_logger(), "Combined image published");
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageCombiner>());
  rclcpp::shutdown();
  return 0;
}

