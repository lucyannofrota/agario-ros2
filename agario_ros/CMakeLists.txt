cmake_minimum_required(VERSION 3.8)
project(agario_ros)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Create custom srvs
rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/PlayerCommands.msg"
    "srv/RegisterPlayer.srv"
)

ament_export_dependencies(rosidl_default_runtime)

# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(agario_server_node src/agario_server_node.cpp)

ament_target_dependencies(
    agario_server_node
    rclcpp
    std_msgs
    std_srvs
)

# Install Cpp executables
install(TARGETS
    agario_server_node
    DESTINATION lib/${PROJECT_NAME}
)

rosidl_get_typesupport_target(cpp_typesupport_target
    ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(agario_server_node "${cpp_typesupport_target}")

# rosidl_target_interfaces(agario_server_node
#     ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install Python modules
ament_python_install_package(${PROJECT_NAME}_game)

# Install Python executables
install(PROGRAMS
    scripts/agario_client_node.py
    scripts/agario_server_node.py
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
