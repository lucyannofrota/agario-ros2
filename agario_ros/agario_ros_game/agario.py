# Python Module addition

import rclpy
import pygame
import pickle
from rclpy.node import Node
from agario_ros.srv import RegisterPlayer
from agario_ros.msg import PlayerCommands, GameState
from .model import Model
from .view import View
from .entities.player import Player
import codecs
import sys

BACKGROUND_COLOR = (24, 26, 50)
BACKGROUND_COLOR = (40, 0, 40)

MODEL_BYTE_SIZE = 2**16

FPS = 60

class agario_ros_client(Node):

    def __init__(self):
        super().__init__('agario_ros_client')
        self.screen = None
        self.player_id = None
        self.player_name = None
        self.is_in_lobby = False
        self.registered = False
        self.clock = pygame.time.Clock()
        self.cli = self.create_client(RegisterPlayer, 'agario_ros_register_player')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = RegisterPlayer.Request()
        self.player_update_msg = PlayerCommands()
        self.player_update_pub = self.create_publisher(PlayerCommands,"agario_ros_player_update",10)
        self.game_state_sub = self.create_subscription(GameState,'agario_ros_game_state_update',self.game_state_update_handler,10)
        self.d_count = 10

    def send_request(self, player_name):
        self.req.player_name = player_name
        
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def start(self,width=900, height=600, user_name=''):
        self.player_name = user_name
        self.player_update_msg.player_name = self.player_name
        # pygame initialization
        pygame.init()
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption('agar.io')
        icon = pygame.image.load('/workspace/src/agario_ros/agario_ros/agario_ros_game/img/logo.png')
        pygame.display.set_icon(icon)

        response = self.send_request(user_name)
        if response.success:
            self.get_logger().info("Player Registered")
            self.player_id = response.id
            self.view = View(self.screen, None, None)
            self.registered = True
            self.input_capture_tim = self.create_timer(1/FPS, self.input_capture)
            return True
        else:
            self.get_logger().info("Invalid Player Name")
            return False
    
    def input_capture(self):
        if not self.registered:        
            return
        # getting list of pressed buttons
        self.player_update_msg.keys = []
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                exit()
            elif event.type == pygame.KEYDOWN:
                self.player_update_msg.keys.append(event.key)

        # get mouse position (velocity vector)
        mouse_pos = self.view.mouse_pos_to_polar()
        self.player_update_msg.mouse_pos[0] = mouse_pos[0]
        self.player_update_msg.mouse_pos[1] = mouse_pos[1]

        # sending velocity vector and list of pressed keys
        self.player_update_pub.publish(self.player_update_msg)

    def game_state_update_handler(self,msg):
        if not self.registered:
            return
        unpickled = pickle.loads(codecs.decode(msg.data.encode(), "base64"))

        # update view and redraw
        self.view.player = None
        this_player = None
        for player in unpickled.players:
            if player.nick == self.player_name:
                this_player = player
        if this_player == None:
            self.d_count = self.d_count - 1
            if self.d_count == 0:
                self.get_logger().warning("Player was killed!")
                self.destroy_node()
                sys.exit()
                exit()
        
        if this_player:
            self.view.model = unpickled.copy_for_client(this_player.center())
            for pl in self.view.model.players:
                if pl.nick == self.player_name:
                    self.view.player = pl
                    break

            if self.view.player is None:
                self.get_logger().warning("Player was killed!")
                return
        
            self.view.redraw()

SPAWN_COUNT_MAX = 500

class agario_ros_server(Node):

    bounds = [1000, 1000]
    cell_num = 250
    model = Model(list(), bounds=bounds)
    model.spawn_cells(cell_num)

    agario_clients = dict()

    player_input_buffer = []

    p = Player.make_random("Jetraid", bounds)

    def __init__(self):
        super().__init__('agario_ros_server')
        self.srv = self.create_service(RegisterPlayer,'agario_ros_register_player',self.player_register_handler)
        self.player_update_sub = self.create_subscription(PlayerCommands,'agario_ros_player_update',self.player_update_handler,10)
        self.game_state_pub = self.create_publisher(GameState,'agario_ros_game_state_update',10)
        self.game_state_msg = GameState()
        self.wall_timer = self.create_timer(0.1/4, self.server_cycle_handler)
        self.spawn_counter = 500

    def player_register_handler(self,request,response):

        self.get_logger().info('Player register request received')

        # Check if player already exists
        if request.player_name in self.agario_clients:
            self.get_logger().info('Player \'{}\' already registered'.format(request.player_name))
            response.success = False
            return response


        # Creating New Player
        new_player = Player.make_random(request.player_name, self.bounds)
        # add client to list of clients
        self.agario_clients[request.player_name] = new_player
        # add player to game model
        self.model.add_player(new_player)

        # # Sending New Player to Client
        response.success = True
        response.id = new_player.id

        self.get_logger().info('Player \'{}\' registered'.format(request.player_name))

        return response

    def player_update_handler(self,msg):
        mouse_pos = msg.mouse_pos
        keys = msg.keys

        # define player according to client address
        if msg.player_name in self.agario_clients:
            player = self.agario_clients[msg.player_name]

            # simulate player actions
            for key in keys:
                if key == pygame.K_w:
                    self.model.shoot(
                        player,
                        mouse_pos[0])
                elif key == pygame.K_SPACE:
                    self.model.split(
                        player,
                        mouse_pos[0])
            # update player velocity and update model state
            self.model.update_velocity(player, *mouse_pos)
            self.agario_clients = self.model.update(self.agario_clients)

    def server_cycle_handler(self):
        # send player state and game model state to clients
        self.spawn_counter = self.spawn_counter - 1
        if self.spawn_counter <= 0:
            self.spawn_counter = SPAWN_COUNT_MAX
            self.model.spawn_cells(50)
        self.game_state_msg.data = codecs.encode(pickle.dumps(self.model), "base64").decode()
        self.game_state_pub.publish(self.game_state_msg)
