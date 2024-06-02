SHELL:=/bin/bash
all:
	@echo help

b:
	./build.sh

BIN:=install/examples_rclcpp_multithreaded_executor/lib/examples_rclcpp_multithreaded_executor/multithreaded_executor
t: test
test:
	source install/setup.bash && \
	valgrind $(BIN)
	#valgrind ros2 run examples_rclcpp_multithreaded_executor  multithreaded_executor
fmt:
	find -name "*.cpp" | xargs clang-format -i
	find -name "*.hpp" | xargs clang-format -i
