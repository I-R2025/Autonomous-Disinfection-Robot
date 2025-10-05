# CMake generated Testfile for 
# Source directory: /home/izza/Documents/slam_ws/src/openzenros
# Build directory: /home/izza/Documents/slam_ws/build/openzenros
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_openzen_sensor_rostest_launch_openzen_test.launch "/home/izza/Documents/slam_ws/build/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/izza/Documents/slam_ws/build/test_results/openzen_sensor/rostest-launch_openzen_test.xml" "--return-code" "/usr/bin/python3 /opt/ros/noetic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/izza/Documents/slam_ws/src/openzenros --package=openzen_sensor --results-filename launch_openzen_test.xml --results-base-dir \"/home/izza/Documents/slam_ws/build/test_results\" /home/izza/Documents/slam_ws/src/openzenros/launch/openzen_test.launch ")
set_tests_properties(_ctest_openzen_sensor_rostest_launch_openzen_test.launch PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/rostest/cmake/rostest-extras.cmake;52;catkin_run_tests_target;/home/izza/Documents/slam_ws/src/openzenros/CMakeLists.txt;98;add_rostest;/home/izza/Documents/slam_ws/src/openzenros/CMakeLists.txt;0;")
subdirs("openzen")
