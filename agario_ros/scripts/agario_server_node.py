#!/usr/bin/env python3

import rclpy
from agario_ros_game.agario import agario_ros_server


def main(args=None):

    print("Hello from agario_ros server")

    rclpy.init(args=None)

    Node = agario_ros_server()

    rclpy.spin(Node)

    # node = agario_ros_server()
    # response = node.send_request(args.player_name)
    # if response.success:
    #     print("Player Registered")
    # else:
    #     print("Invalid Player Name")
    # # print("Player ID: {}".format(response.player_id))
    # node.get_logger().info("Result")

    # client.start(args.width, args.height, args.player_name, random.randint(0,1000))

    # print(add(1,2))

    rclpy.shutdown()


if __name__ == '__main__':
    main()