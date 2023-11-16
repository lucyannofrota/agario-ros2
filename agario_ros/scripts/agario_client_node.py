#!/usr/bin/env python3

import rclpy
import argparse
import random
import threading
# import ..agario_ros_game.view
from agario_ros_game.view import View
from agario_ros_game.agario import agario_ros_client

def main(args=None):

    parser = argparse.ArgumentParser(
    description="Python implementation of game agar.io")
    parser.add_argument(
        '-wt', '--width',
        dest='width',
        type=int,
        default=900,
        help='screen width')
    parser.add_argument(
        '-ht', '--height',
        dest='height',
        type=int,
        default=600,
        help='screen height')
    parser.add_argument(
        '-n', '--name',
        dest='player_name',
        type=str,
        default='player_{}'.format(random.randint(0, 50)),
        help='screen height')

    args = parser.parse_args()

    print("Hello from agario_ros client")

    rclpy.init(args=None)

    node = agario_ros_client()
    
    if not node.start(args.width, args.height, args.player_name):
        rclpy.try_shutdown()
        return 1
    
    node.view = View(node.screen, None, None)

    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    rate = node.create_rate(10)

    try:
        while rclpy.ok():
            # rclpy.spin_once(node)
            # node.create_rate()
            node.game_cycle()
            rate.sleep()
    except KeyboardInterrupt:
        pass
        

    # print(add(1,2))

    rclpy.try_shutdown()
    thread.join()


if __name__ == '__main__':
    main()