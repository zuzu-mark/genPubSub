#include "sub.hpp"
#include "base.hpp"
//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////
void genSub::recv(void) {
  std::string topic_name = "/string_topic";
  std::string type = "sensor_msgs/msg/PointCloud2";
  subscribe_raw_messages<std::string, sensor_msgs::msg::PointCloud2>(
      1, topic_name, type);
}
//////////////////////////////////
// generic subscriber
//////////////////////////////////
template <typename T1, typename T2>
void genSub::subscribe_raw_messages(size_t expected_recv_msg_count,
                                    const std::string &topic_name,
                                    const std::string &type) {
  std::vector<T1> messages;
  size_t counter = 0;

  sub_ = node_->create_generic_subscription(
      topic_name, type, rclcpp::QoS(1),
      [&counter, &messages,
       this](std::shared_ptr<rclcpp::SerializedMessage> message) {
        T2 deserialized_message;
        rclcpp::Serialization<T2> serializer;
        serializer.deserialize_message(message.get(), &deserialized_message);

        auto recv = this->get_data_from_msg(deserialized_message);
        counter++;

        RCLCPP_INFO(node_->get_logger(), "\n<<receive[type] %ld>> : %s",
                    counter, typeid(recv).name());

#if 1 // verify
        for (auto v : recv.data) {
          RCLCPP_INFO(node_->get_logger(), "\n<<receive[aa] %ld>> : %s",
                      counter, typeid(v).name());

          RCLCPP_INFO(node_->get_logger(), "\n<<receive %ld>> : %d", counter,
                      (v));
        }
#endif
      });

  return; // messages;
}
