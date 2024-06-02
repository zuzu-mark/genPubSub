SHELL:=/bin/bash
all:
	@echo help

b:
	./build.sh

BIN:=install/examples_rclcpp_multithreaded_executor/lib/examples_rclcpp_multithreaded_executor/multithreaded_executor
e:
	source install/setup.bash && \
	$(BIN)

t: test
test:
	source install/setup.bash && \
	valgrind $(BIN)
	#valgrind ros2 run examples_rclcpp_multithreaded_executor  multithreaded_executor
fmt:
	find src/pubsub -name "*.cpp" | xargs clang-format -i
	find src/pubsub -name "*.hpp" | xargs clang-format -i
