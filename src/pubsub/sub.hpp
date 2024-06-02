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

class DualThreadedNode;
//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////
using std::placeholders::_1;
template <typename msgT> class genSub2 {
public:
  // using Ptr = std::shared_ptr<genSub2>;
  genSub2(DualThreadedNode *node) : node_(node) {}
  void recv(std::string topic_name, std::string type);

  ~genSub2() {
    if (sub_ != nullptr) {
      std::cout << "gensub-dtor" << std::endl;
      sub_.reset();
      sub_ = nullptr;
    }
  }
  void stop() {

    if (sub_ != nullptr) {
      std::cout << "deleter(sub)" << std::endl;
      sub_.reset();
      sub_ = nullptr;
    }
  }

private:
  sensor_msgs::msg::PointCloud2 get_data_from_msg(const msgT &message) {
    return message;
  }
#if 0
  sensor_msgs::msg::PointCloud2::SharedPtr
  get_data_from_msg(const msgT &message) { 
     sensor_msgs::msg::PointCloud2::SharedPtr bbb(
         new sensor_msgs::msg::PointCloud2());
     bbb=message;
	  return bbb;
  }
#endif

  void topic_callback(std::shared_ptr<rclcpp::SerializedMessage> message) {
    msgT deserialized_message;
    rclcpp::Serialization<msgT> serializer;
    serializer.deserialize_message(message.get(), &deserialized_message);

    auto recv = this->get_data_from_msg(deserialized_message);
    // auto recv = deserialized_message;

#if 0 // verify
    size_t counter = 0;
    RCLCPP_INFO(node_->get_logger(), "\n<<receive2[type] %ld>> : %s", counter,
                typeid(recv).name());

    RCLCPP_INFO(node_->get_logger(), "\n<<receive2[type] %ld>> : %s", counter,
                typeid(recv).name());

    for (auto v : recv.data) {
      RCLCPP_INFO(node_->get_logger(), "\n<<receive2 %ld>> : %d", counter, (v));
    }
#endif

    auto time_offset_ns = (node_->now() - recv.header.stamp).nanoseconds();
    auto timestamp_offset_ns =
        (rclcpp::Time(recv.header.stamp) - m_last_cloud_ts).nanoseconds();
    auto time_offset_ms = time_offset_ns / 1000000.0F;
    auto timestamp_offset_ms = timestamp_offset_ns / 1000000.0F;
    RCLCPP_INFO(node_->get_logger(), "get-pcl1m-transport-time: %.3f [ms]",
                time_offset_ms);
    if (m_last_cloud_ts.nanoseconds() > 0.0) {
      RCLCPP_INFO(node_->get_logger(), "get-pcl1m-timestamp_offset-time: %.3f",
                  timestamp_offset_ms);
    }
    m_last_cloud_ts = recv.header.stamp;
  }
#if 1
  template <typename T2>
  void subscribe_raw_messages(size_t expected_recv_msg_count,
                              const std::string &topic_name,
                              const std::string &type) {
    size_t counter = 0;

    sub_ = node_->create_generic_subscription(
        topic_name, type, rclcpp::QoS(1),
        std::bind(&genSub2<msgT>::topic_callback, this, _1));

    return;
  }
#endif

private:
  DualThreadedNode *node_;
  std::shared_ptr<rclcpp::GenericSubscription> sub_;
  rclcpp::Time m_last_cloud_ts{0, 0, RCL_ROS_TIME};
};
////////////////////
template <typename msgT>
inline void genSub2<msgT>::recv(std::string topic_name, std::string type) {

  subscribe_raw_messages<sensor_msgs::msg::PointCloud2>(1, topic_name, type);
}
//////////////////////////////////////////////////
////////////////////////////////////////////////// {{{
//////////////////////////////////////////////////
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
//}}}
