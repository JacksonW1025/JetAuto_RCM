
cmake_minimum_required(VERSION 3.8)
project(xf_mic_asr_offline)

if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(xf_mic_asr_offline_msgs REQUIRED)
find_package(ros_robot_controller_msgs REQUIRED)

include_directories(
  include
)

link_directories(
  lib/arm64
)

# Create the executable target
add_executable(voice_control src/voice_control.cpp)
ament_target_dependencies(voice_control rclcpp std_msgs xf_mic_asr_offline_msgs ros_robot_controller_msgs)
target_link_libraries(voice_control ${catkin_LIBRARIES} offline_record_lib rt dl pthread stdc++ asound)
install(TARGETS voice_control
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
# Build the target
install(TARGETS voice_control
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/awake_node.py
  scripts/asr_node.py
  scripts/voice_control_move.py
  scripts/voice_control_navigation.py
  scripts/voice_control_arm.py
  scripts/voice_control_color_detect.py
  scripts/voice_control_color_sorting.py
  scripts/voice_control_color_track.py
  scripts/voice_control_garbage_classification.py
  scripts/voice_control_navigation_transport.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config feedback_voice
  DESTINATION share/${PROJECT_NAME})

# Add the test directories if needed
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Create the ROS 2 package
ament_package()

