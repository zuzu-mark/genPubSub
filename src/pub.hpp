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

class PublisherNode;
//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////
template <typename msgT> class genPub2 {
public:
  using Ptr = std::shared_ptr<genPub2>;
  genPub2(PublisherNode *node) : node_(node), count_(0) {}

  void init(std::string topic_name, std::string type);

  void send(msgT a);

private:
  void write_message(const msgT &data, msgT &message) { message = data; }

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

template <typename msgT>
void genPub2<msgT>::init(std::string topic_name, std::string type) {

  publisher2_ =
      node_->create_generic_publisher(topic_name, type, rclcpp::QoS(1));
}
template <typename msgT> void genPub2<msgT>::send(msgT msg) {

  ////////////////////////////////
  // execute PUB
  ////////////////////////////////
  {
    // publish
    this->publisher2_->publish(serialize_message<msgT, msgT>(msg));
  }
}

//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////
class genPub {

public:
  using Ptr = std::shared_ptr<genPub>;
  genPub(PublisherNode *node) : node_(node), count_(0) {}

  // void send(msgT msg);
  void send(void);
  void init(void);

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
