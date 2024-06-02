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
class DualThreadedNode;
class genSub {
public:
  using Ptr = std::shared_ptr<genSub>;
  genSub(DualThreadedNode *node) : node_(node) {}
  void recv(void);

private:
  sensor_msgs::msg::PointCloud2
  get_data_from_msg(const sensor_msgs::msg::PointCloud2 &message) {
    return message;
  }

  template <typename T1, typename T2>
  void subscribe_raw_messages(size_t expected_recv_msg_count,
                              const std::string &topic_name,
                              const std::string &type);

private:
  DualThreadedNode *node_;
  std::shared_ptr<rclcpp::GenericSubscription> sub_;
};
