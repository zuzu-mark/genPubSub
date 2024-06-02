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
// class genPub;
class PublisherNode : public rclcpp::Node {

public:
  ~PublisherNode() {
    if (gp2_ != nullptr) {
      std::cout << "dtor(send)" << std::endl;
      delete gp2_;
    }
  }
  // ctor
  PublisherNode() : Node("PublisherNode"), count_(0), gp_(new genPub(this)) {

#if 0
    gp2_ = new genPub2<sensor_msgs::msg::PointCloud2>(this);
    // auto gp2 = std::make_shared<genPub2<int>>(this);
    std::string topic_name = "/string_topic";
    std::string type = "sensor_msgs/msg/PointCloud2";
    gp2_->init(topic_name, type);

    auto timer_callback = [this]() -> void {
      auto message = std_msgs::msg::String();
      message.data = "Hello World! " + std::to_string(this->count_++);

      // Extract current thread
      auto curr_thread = string_thread_id();

      // Prep display message
      RCLCPP_INFO(this->get_logger(), "\n<<THREAD %s>> Publishing '%s'",
                  curr_thread.c_str(), message.data.c_str());

      ///////////////////////
      // SEND
      ///////////////////////
      // gp_->send();
#if 1
      {
        pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
        sensor_msgs::msg::PointCloud2 bbb;

        // make-data
        std::vector<std::uint8_t> data;
        std::uint8_t a = count_ + 1000;
        data.push_back(a);
        cloud_filtered->data = data;
        // this->count_++;

        // convert
        pcl_conversions::fromPCL(*cloud_filtered, bbb);
        gp2_->send(bbb);
      }
#endif
    };
    timer_ = this->create_wall_timer(500ms, timer_callback);
#endif
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std::shared_ptr<rclcpp::GenericPublisher> publisher2_;
  size_t count_;

  genPub2<sensor_msgs::msg::PointCloud2> *gp2_;
  // genPub2<int> *gp2_;
  genPub *gp_;
};
///////////////////////////////////////
///////////////////////////////////////
///////////////////////////////////////
#include "sub.hpp"
class genSub;
class DualThreadedNode : public rclcpp::Node {

  genSub *gs_;
  genSub2<sensor_msgs::msg::PointCloud2> *gs2_;

public:
  ~DualThreadedNode() {
    if (gs2_ != nullptr) {
      std::cout << "dtor(recv)" << std::endl;
      delete gs_;
    }
  }
  DualThreadedNode() : Node("DualThreadedNode"), gs_(new genSub(this)) {

    gs2_ = new genSub2<sensor_msgs::msg::PointCloud2>(this);

    std::string topic_name = "/string_topic";
    std::string type = "sensor_msgs/msg/PointCloud2";
    // recv
    // gs_->recv();
    gs2_->recv(topic_name, type);
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
