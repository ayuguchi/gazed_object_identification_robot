# CMake generated Testfile for 
# Source directory: /root/catkin_ws/src/urg_node
# Build directory: /root/catkin_ws/build/urg_node
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_urg_node_roslint_package "/root/catkin_ws/build/urg_node/catkin_generated/env_cached.sh" "/usr/bin/python" "/opt/ros/kinetic/share/catkin/cmake/test/run_tests.py" "/root/catkin_ws/build/urg_node/test_results/urg_node/roslint-urg_node.xml" "--working-dir" "/root/catkin_ws/build/urg_node" "--return-code" "/opt/ros/kinetic/share/roslint/cmake/../../../lib/roslint/test_wrapper /root/catkin_ws/build/urg_node/test_results/urg_node/roslint-urg_node.xml make roslint_urg_node")
add_test(_ctest_urg_node_roslaunch-check_launch_urg_lidar.launch "/root/catkin_ws/build/urg_node/catkin_generated/env_cached.sh" "/usr/bin/python" "/opt/ros/kinetic/share/catkin/cmake/test/run_tests.py" "/root/catkin_ws/build/urg_node/test_results/urg_node/roslaunch-check_launch_urg_lidar.launch.xml" "--return-code" "/usr/bin/cmake -E make_directory /root/catkin_ws/build/urg_node/test_results/urg_node" "/opt/ros/kinetic/share/roslaunch/cmake/../scripts/roslaunch-check -o '/root/catkin_ws/build/urg_node/test_results/urg_node/roslaunch-check_launch_urg_lidar.launch.xml' '/root/catkin_ws/src/urg_node/launch/urg_lidar.launch' ")
subdirs(gtest)
