#include "agario_ros/srv/register_player.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"

#include <chrono>
#include <cstdlib>
#include <functional>
#include <memory>
#include <string>

// TODO: Add new players to a data structure and maintain that structure

using namespace std::chrono_literals;

class agario_ros_server : public rclcpp::Node
{
  public:

    agario_ros_server() : Node( "agario_ros_server" ) {}

    // agario_ros::msg::Reg();

    void register_player(
        const std::shared_ptr< agario_ros::srv::RegisterPlayer::Request > request,
        std::shared_ptr< agario_ros::srv::RegisterPlayer::Response > response )
    {
        response->player_id = rand() % 100;
        RCLCPP_INFO(
            this->get_logger(), "Player Register Request [nick: %s, id: %i]", request->player_name.c_str(),
            (int) response->player_id );
    }

  private:

    rclcpp::Service< agario_ros::srv::RegisterPlayer >::SharedPtr register_player_srv =
        this->create_service< agario_ros::srv::RegisterPlayer >(
            "agario_ros_register_player",
            std::bind( &agario_ros_server::register_player, this, std::placeholders::_1, std::placeholders::_2 ) );
};
