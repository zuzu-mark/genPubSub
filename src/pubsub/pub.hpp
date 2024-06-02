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

#include "shm_msgs/msg/point_cloud2.hpp"
#include "shm_msgs/pcl_conversions.h"

class PublisherNode;
//////////////////////////////////////////////////
using namespace shm_msgs;
//////////////////////////////////////////////////
//////////////////////////////////////////////////
template <typename msgT> class genPub2 {
private:
  using Topic = shm_msgs::msg::PointCloud1m;

public:
  using Ptr = std::shared_ptr<genPub2>;
  ~genPub2() {
    if (publisher2_ != nullptr) {
      std::cout << "genpub-dtor" << std::endl;
    }
  }
  genPub2(PublisherNode *node) : node_(node), count_(0) {}

  void init(std::string topic_name, std::string type);

  void send(msgT a);
  void stop() {

    if (publisher2_ != nullptr) {
      std::cout << "deleter" << std::endl;
      publisher2_.reset();
      publisher2_ = nullptr;
    }
  }

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

private:
  void populateLoanedMessage(msgT _msg,
                             rclcpp::LoanedMessage<Topic> &loanedMsg) {
    Topic &msg = loanedMsg.get();

#if 1
    // Create the data.
    // In general this will not be constant.
    // Ideally we would create it in place but the ROS API does not allow
    // that. Therefore we need to copy it to the loaned message.

    // We can track a quasi dynamic (bounded) size like this to avoid
    // copying more data than needed.
    // msg.size = (uint8_t)std::min(payload.size(), (size_t)Topic::MAX_SIZE);

    // Note that msg.data is a std::array generated by the IDL compiler
    // toROSMsg(*m_input_cloud, msg);
    // toROSMsg(*_msg, msg);
#endif

    msg.header.stamp = node_->now();
    set_str(msg.header.frame_id, "shm_pcl_1m");

    RCLCPP_INFO(node_->get_logger(), "Publishing ");
  }
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
  if (publisher2_ != nullptr) {
    // publish
    this->publisher2_->publish(serialize_message<msgT, msgT>(msg));

    // auto loanedMsg = this->publisher2_->borrow_loaned_message();
    // RCLCPP_INFO(node_->get_logger(), "Publishing:%s ",
    //             typeid(loanedMsg).name());
    //  void publish_as_loaned_msg(const rclcpp::SerializedMessage & message);
    //  this->publisher2_->publish_as_loaned_msg(serialize_message<msgT,
    //  msgT>(msg)); populateLoanedMessage(msg,loanedMsg);
    //   m_publisher->publish(std::move(loanedMsg));
    //  this->publisher2_->publish(serialize_message<msgT, msgT>(loanedMsg));
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