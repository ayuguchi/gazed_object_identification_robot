# CMake generated Testfile for 
# Source directory: /root/catkin_ws/src/scan_tools/laser_scan_matcher
# Build directory: /root/catkin_ws/build/laser_scan_matcher
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_laser_scan_matcher_rostest_test_run.test "/root/catkin_ws/build/laser_scan_matcher/catkin_generated/env_cached.sh" "/usr/bin/python" "/opt/ros/kinetic/share/catkin/cmake/test/run_tests.py" "/root/catkin_ws/build/laser_scan_matcher/test_results/laser_scan_matcher/rostest-test_run.xml" "--return-code" "/opt/ros/kinetic/share/rostest/cmake/../../../bin/rostest --pkgdir=/root/catkin_ws/src/scan_tools/laser_scan_matcher --package=laser_scan_matcher --results-filename test_run.xml --results-base-dir \"/root/catkin_ws/build/laser_scan_matcher/test_results\" /root/catkin_ws/src/scan_tools/laser_scan_matcher/test/run.test ")
add_test(_ctest_laser_scan_matcher_rostest_test_covariance.test "/root/catkin_ws/build/laser_scan_matcher/catkin_generated/env_cached.sh" "/usr/bin/python" "/opt/ros/kinetic/share/catkin/cmake/test/run_tests.py" "/root/catkin_ws/build/laser_scan_matcher/test_results/laser_scan_matcher/rostest-test_covariance.xml" "--return-code" "/opt/ros/kinetic/share/rostest/cmake/../../../bin/rostest --pkgdir=/root/catkin_ws/src/scan_tools/laser_scan_matcher --package=laser_scan_matcher --results-filename test_covariance.xml --results-base-dir \"/root/catkin_ws/build/laser_scan_matcher/test_results\" /root/catkin_ws/src/scan_tools/laser_scan_matcher/test/covariance.test ")
subdirs(gtest)
