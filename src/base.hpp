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

using namespace std::chrono_literals;
///////////////////////////////////////
///////////////////////////////////////
///////////////////////////////////////
/**
 * A small convenience function for converting a thread ID to a string
 **/
inline std::string string_thread_id() {
  auto hashed = std::hash<std::thread::id>()(std::this_thread::get_id());
  return std::to_string(hashed);
}

///////////////////////////////////////
///////////////////////////////////////
///////////////////////////////////////
#include "pub.hpp"
class genPub;
class PublisherNode : public rclcpp::Node {
  void write_message(const std::string &data, std_msgs::msg::String &message) {
    message.data = data;
  }

  void write_message(const sensor_msgs::msg::PointCloud2 &data,
                     sensor_msgs::msg::PointCloud2 &message) {
    message = data;
  }

  void write_message(const sensor_msgs::msg::PointCloud2 &data,
                     std_msgs::msg::String &message) {
    std::string str;
    for (auto &v : data.data) {
      str = std::to_string(v);
      std::cout << "str:" << str << std::endl;
    }
    message.data = str;
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

public:
  // ctor
  PublisherNode() : Node("PublisherNode"), count_(0), gp_(new genPub(this)) {

    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    auto timer_callback = [this]() -> void {
      auto message = std_msgs::msg::String();
      message.data = "Hello World! " + std::to_string(this->count_++);

      // Extract current thread
      auto curr_thread = string_thread_id();

      // Prep display message
      RCLCPP_INFO(this->get_logger(), "\n<<THREAD %s>> Publishing '%s'",
                  curr_thread.c_str(), message.data.c_str());
      gp_->send();
    };
    timer_ = this->create_wall_timer(500ms, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std::shared_ptr<rclcpp::GenericPublisher> publisher2_;
  size_t count_;

  genPub *gp_;
};
///////////////////////////////////////
///////////////////////////////////////
///////////////////////////////////////
#include "sub.hpp"
class genSub;
class DualThreadedNode : public rclcpp::Node {

  genSub *gs_;

public:
  DualThreadedNode() : Node("DualThreadedNode"), gs_(new genSub(this)) {
    // recv
    gs_->recv();
  }

private:
  /**
   * Simple function for generating a timestamp
   * Used for somewhat ineffectually demonstrating that the multithreading
   * doesn't cripple performace
   */
  std::string timing_string() {
    rclcpp::Time time = this->now();
    return std::to_string(time.nanoseconds());
  }
};
