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
template <typename msgT> class genSub2 {
public:
  using Ptr = std::shared_ptr<genSub2>;
  genSub2(DualThreadedNode *node) : node_(node) {}
  void recv(std::string topic_name, std::string type);

  ~genSub2() {
    if (sub_ != nullptr) {
      std::cout << "gensub-dtor" << std::endl;
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
  sensor_msgs::msg::PointCloud2::SharedPtr
  get_data_from_msg(const msgT &message) { 
     sensor_msgs::msg::PointCloud2::SharedPtr bbb(
         new sensor_msgs::msg::PointCloud2());
     bbb=message;
	  return bbb;
  }

#if 1
  template <typename T2>
  void subscribe_raw_messages(size_t expected_recv_msg_count,
                              const std::string &topic_name,
                              const std::string &type) {
    size_t counter = 0;

    sub_ = node_->create_generic_subscription(
        topic_name, type, rclcpp::QoS(1),
        [&counter, this](std::shared_ptr<rclcpp::SerializedMessage> message) {
          T2 deserialized_message;
          rclcpp::Serialization<T2> serializer;
          serializer.deserialize_message(message.get(), &deserialized_message);

          auto recv = this->get_data_from_msg(deserialized_message);
          counter++;

#if 0 // verify
          RCLCPP_INFO(node_->get_logger(), "\n<<receive2[type] %ld>> : %s",
                      counter, typeid(recv).name());

          for (auto v : recv.data) {
            //RCLCPP_INFO(node_->get_logger(), "\n<<receive2[aa] %ld>> : %s",
            //            counter, typeid(v).name());

            RCLCPP_INFO(node_->get_logger(), "\n<<receive2 %ld>> : %d", counter,
                        (v));
          }
#endif
        });

    return; // messages;
  }
#endif

private:
  DualThreadedNode *node_;
  std::shared_ptr<rclcpp::GenericSubscription> sub_;
};
////////////////////
template <typename msgT>
inline void genSub2<msgT>::recv(std::string topic_name, std::string type) {

  subscribe_raw_messages<sensor_msgs::msg::PointCloud2>(1, topic_name, type);
}
//////////////////////////////////////////////////
//////////////////////////////////////////////////
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
