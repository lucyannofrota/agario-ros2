# Python Module addition

import rclpy
import pygame
from rclpy.node import Node
from agario_ros.srv import RegisterPlayer
from agario_ros.msg import PlayerCommands
from .model import Model
from .entities.player import Player

BACKGROUND_COLOR = (24, 26, 50)
BACKGROUND_COLOR = (40, 0, 40)

FPS = 60

class agario_ros_client(Node):

    def __init__(self):
        super().__init__('agario_ros_client')
        self.screen = None
        self.player_id = None
        self.player_name = None
        self.is_in_lobby = False
        self.clock = pygame.time.Clock()
        self.cli = self.create_client(RegisterPlayer, 'agario_ros_register_player')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = RegisterPlayer.Request()
        self.player_update_msg = PlayerCommands()
        self.player_update_pub = self.create_publisher(PlayerCommands,"agario_ros_player_update",10)

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
        icon = pygame.image.load('/workspace/src/agario_ros/agario_ros_game/img/logo.png')
        pygame.display.set_icon(icon)

        # init class with game connection
        # gameconn = GameConnection(screen)
        # create menu
        # menu = MyMenu(width*0.9, height*0.9)
        # bind connection method to menu button
        # gameconn.connect_to_game
        # menu.update_start_menu(gameconn.connect_to_game)
        # gameconn.connect_to_game({'nick': user_name, 'addr': 'localhost:9999'})

        response = self.send_request(user_name)
        if response.success:
            self.get_logger().info("Player Registered")
            self.player_id = response.id
            return True
        else:
            self.get_logger().info("Invalid Player Name")
            return False


        # FPS = 30
        # clock = pygame.time.Clock()

        # # start pygame loop
        # while True:
        #     events = pygame.event.get()
        #     for event in events:
        #         if event.type == pygame.QUIT:
        #             exit()
            
        #     self.screen.fill(BACKGROUND_COLOR)
            
        #     # if menu.get_main_menu().is_enabled():
        #     #     menu.get_main_menu().draw(screen)
        #     # menu.get_main_menu().update(events)
        #     pygame.display.flip()
        #     clock.tick(FPS)
    
    def game_cycle(self):
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

        # # getting current player and game model state
        # data = sock.recv(2**16)
        # msg = pickle.loads(data)

        # # update view and redraw
        # view.player = None
        # view.model = msg
        # for pl in view.model.players:
        #     if pl.id == self.player_id:
        #         view.player = pl
        #         break

        # if view.player is None:
        #     logger.debug("Player was killed!")
        #     return

        # view.redraw()
        # time.sleep(1/40)
        self.clock.tick(FPS)

class agario_ros_server(Node):

    bounds = [1000, 1000]
    cell_num = 150
    model = Model(list(), bounds=bounds)
    model.spawn_cells(cell_num)

    agario_clients = dict()

    p = Player.make_random("Jetraid", bounds)

    def __init__(self):
        super().__init__('agario_ros_server')
        self.srv = self.create_service(RegisterPlayer,'agario_ros_register_player',self.player_register_handler)
        self.player_update_sub = self.create_subscription(PlayerCommands,'agario_ros_player_update',self.player_update_handler,10)

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
        print(msg)
        mouse_pos = msg.mouse_pos
        keys = msg.keys

        # define player according to client address
        player = self.agario_clients[msg.player_name]
        if not player:
            return

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
        self.model.update()

        # send player state and game model state to client
        # data = pickle.dumps(model.copy_for_client(player.center()))
        # socket = self.request[1]
        # socket.sendto(data, self.client_address)
