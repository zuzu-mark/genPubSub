#include "base.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // You MUST use the MultiThreadedExecutor to use, well, multiple threads
  rclcpp::executors::MultiThreadedExecutor executor;
  auto pubnode = std::make_shared<PublisherNode>();
  auto subnode =
      std::make_shared<DualThreadedNode>(); // This contains BOTH subscriber
                                            // callbacks. They will still run on
                                            // different threads One Node. Two
                                            // callbacks. Two Threads
  executor.add_node(pubnode);
  executor.add_node(subnode);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
