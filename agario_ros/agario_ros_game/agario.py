# Python Module addition

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from agario_ros.srv import RegisterPlayer

class agario_ros_client(Node):

    def __init__(self):
        super().__init__('agario_ros_client')
        self.cli = self.create_client(RegisterPlayer, 'agario_ros_register_player')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = RegisterPlayer.Request()

    def send_request(self, player_name):
        self.req.player_name = player_name
        
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
