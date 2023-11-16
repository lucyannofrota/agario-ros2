#include "agario_ros/srv/register_player.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"

#include <chrono>
#include <cstdlib>
// #include <functional>
#include <map>
#include <memory>
#include <string>

// TODO: Add new players to a data structure and maintain that structure

using namespace std::chrono_literals;

#define BOUNDARY_X 1000
#define BOUNDARY_Y 1000
#define CELL_COUNT 150

struct Player
{
    double x;
    double y;

    Player( void )
    {
        this->x = rand() % BOUNDARY_X;
        this->y = rand() % BOUNDARY_Y;
    }
};

struct Chunk
{
};

class agario_ros_server : public rclcpp::Node

{
  public:

    agario_ros_server() : Node( "agario_ros_server" ) {}

    void register_player(
        const std::shared_ptr< agario_ros::srv::RegisterPlayer::Request > request,
        std::shared_ptr< agario_ros::srv::RegisterPlayer::Response > response )
    {
        // Check if player already exists
        if( this->registered_players.find( request->player_name ) != this->registered_players.end() )
        {
            response->success = false;
            RCLCPP_INFO( this->get_logger(), "Player \"%s\" already exists", request->player_name.c_str() );
            return;
        }
        else
        {
            this->registered_players[ request->player_name ] = Player();
            response->success                                = true;
            RCLCPP_INFO( this->get_logger(), "Player \"%s\" registered", request->player_name.c_str() );
        }
        // RCLCPP_INFO(
        //     this->get_logger(), "Player Register Request [nick: %s, id: %i]", request->player_name.c_str(),
        //     (int) response->player_id );
    }

  private:

    std::map< std::string, Player > registered_players;

    rclcpp::Service< agario_ros::srv::RegisterPlayer >::SharedPtr register_player_srv =
        this->create_service< agario_ros::srv::RegisterPlayer >(
            "agario_ros_register_player",
            std::bind( &agario_ros_server::register_player, this, std::placeholders::_1, std::placeholders::_2 ) );
};
