#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "rclcpp/generic_publisher.hpp"
#include "rclcpp/generic_subscription.hpp"
// #include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////
class PublisherNode;
class genPub {

public:
  using Ptr = std::shared_ptr<genPub>;
  genPub(PublisherNode *node) : node_(node), count_(0) {}
  void send();

private:
  void write_message(const sensor_msgs::msg::PointCloud2 &data,
                     sensor_msgs::msg::PointCloud2 &message) {
    message = data;
  }

  template <typename T1, typename T2>
  rclcpp::SerializedMessage serialize_message(const T1 &data) {
    T2 message;
    write_message(data, message);

    rclcpp::Serialization<T2> ser;
    rclcpp::SerializedMessage result;
    ser.serialize_message(&message, &result);
    return result;
  }

private:
  std::shared_ptr<rclcpp::GenericPublisher> publisher2_;
  PublisherNode *node_;
  size_t count_;
};
