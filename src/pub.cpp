#include "pub.hpp"
#include "base.hpp"

//////////////////////////////
// generic publisher
//////////////////////////////
void genPub::send() {
  std::string topic_name = "/string_topic";
  std::string type = "sensor_msgs/msg/PointCloud2";

  publisher2_ =
      node_->create_generic_publisher(topic_name, type, rclcpp::QoS(1));

  ////////////////////////////////
  // execute PUB
  ////////////////////////////////
  {
    // reference
    // https://github.com/GitRepJo/pcl_example/blob/main/src/pcl_example.cpp
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
    sensor_msgs::msg::PointCloud2 bbb;

    // make-data
    std::vector<std::uint8_t> data;
    std::uint8_t a = count_ + 1000;
    data.push_back(a);
    cloud_filtered->data = data;
    this->count_++;

    // convert
    pcl_conversions::fromPCL(*cloud_filtered, bbb);

    // publish
    this->publisher2_->publish(
        serialize_message<sensor_msgs::msg::PointCloud2,
                          sensor_msgs::msg::PointCloud2>(bbb));
  }
}
