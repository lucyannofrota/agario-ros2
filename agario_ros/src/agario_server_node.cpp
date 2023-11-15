#include "agario_ros/agario_server.hpp"

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <stdio.h>

int main( int argc, char* argv[] )
{
    rclcpp::init( argc, argv );
    rclcpp::spin( std::make_shared< agario_ros_server >() );
    rclcpp::shutdown();
    return 0;
}
