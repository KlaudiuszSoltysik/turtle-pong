#!/usr/bin/python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from turtlesim.srv import Spawn, TeleportAbsolute, SetPen
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from interfaces.msg import Move

from math import pow, sqrt
from random import randint


class Game(Node):
    
    def __init__(self):
        super().__init__('game')
        
        # VARIABLE INITIALIZATION
        self.vel_x = -2.0
        self.vel_y = randint(-200, 200)/100
        
        self.computer_score = 0
        self.player_score = 0
        
        self.ball_pose = Pose()
        self.computer_pose = Pose()
        self.player_pose = Pose()
        self.direction = Move()

        # CONNECT WITH SPAWN SERVICES
        self.spawn_cli = self.create_client(Spawn, 'spawn')

        while not self.spawn_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.spawn_turtle_request = Spawn.Request()
        
        self.send_spawn_turtle_computer_request()
        self.send_spawn_turtle_player_request()
        
        # CONNECT WITH SET_PEN SERVICES
        self.set_pen_ball_cli = self.create_client(SetPen, 'turtle1/set_pen')

        while not self.set_pen_ball_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
            
        self.set_pen_computer_cli = self.create_client(SetPen, 'computer/set_pen')

        while not self.set_pen_computer_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
            
        self.set_pen_player_cli = self.create_client(SetPen, 'player/set_pen')

        while not self.set_pen_player_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.set_pen_request = SetPen.Request()
        
        # CONNECT WITH TELEPORT SERVICES
        self.teleport_cli = self.create_client(TeleportAbsolute, 'turtle1/teleport_absolute')

        while not self.teleport_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.teleport_turtle_request = TeleportAbsolute.Request()

        # CREATE SUBSCRIBERS TO POSE TOPICS
        self.ball_pose_sub = self.create_subscription(Pose, 'turtle1/pose', self.ball_pose_sub_callback, 10)
        self.ball_pose_sub

        self.computer_pose_sub = self.create_subscription(Pose, 'computer/pose', self.computer_pose_sub_callback, 10)
        self.computer_pose_sub

        self.player_pose_sub = self.create_subscription(Pose, 'player/pose', self.player_pose_sub_callback, 10)
        self.player_pose_sub

        # CREATE PUBLISHERS ON CMD_VEL TOPICS
        self.ball_pose_pub = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)        
        self.computer_pose_pub = self.create_publisher(Twist, 'computer/cmd_vel', 10)
        self.player_pose_pub = self.create_publisher(Twist, 'player/cmd_vel', 10)
        
        # CREATE SUBSCRIBER TO MOVE_PLAYER_TURTLE TOPIC
        self.move_player_turtle_sub = self.create_subscription(Move, 'move_player_turtle', self.move_player_turtle_sub_callback, 10)
        self.move_player_turtle_sub
        
        # SWITCH OFF PENS
        self.send_set_pen_ball_request()
        self.send_set_pen_computer_request()
        self.send_set_pen_player_request()
        
        # RUN GAME LOOP FUNCTION
        self.timer = self.create_timer(1 / 30, self.game_loop)
        
        # PROMPT USER
        self.get_logger().info('----------------------')
        self.get_logger().info('Game has been started.')


    def game_loop(self):
        distance_from_computer = sqrt(pow(self.ball_pose.x - self.computer_pose.x, 2) + pow(self.ball_pose.y - self.computer_pose.y, 2))
        distance_from_player = sqrt(pow(self.ball_pose.x - self.player_pose.x, 2) + pow(self.ball_pose.y - self.player_pose.y, 2))
        
        # COMPUTER TURTLE AI
        computer_msg = Twist()
        
        if self.ball_pose.y > self.computer_pose.y:
            computer_msg.linear.y = 2.0
        elif self.ball_pose.y < self.computer_pose.y:
            computer_msg.linear.y = -2.0
            
        self.computer_pose_pub.publish(computer_msg)
            
        # MOVE PLAYER TURTLE
        player_msg = Twist()
        
        if self.direction == 0:
            player_msg.linear.y = 0.0
        elif self.direction == 1:
            player_msg.linear.y = 2.0
        elif self.direction == -1:
            player_msg.linear.y = -2.0
            
        if self.player_pose.y > 10.5 and player_msg.linear.y == -2.0:
            player_msg.linear.y = 0.0
        elif self.player_pose.y < 0.5 and player_msg.linear.y == 2.0:
            player_msg.linear.y = 0.0
        
        self.player_pose_pub.publish(player_msg)
        
        # BOUNCE FROM TURTLES
        if distance_from_computer < 0.5 and self.computer_pose.x < self.ball_pose.x:
            if computer_msg.linear.y == 2.0 and self.vel_y <= 1.5:
                self.vel_y += 0.5
            elif computer_msg.linear.y == -2.0 and self.vel_y >= -1.5:
                self.vel_y -= 0.5
                
            self.vel_x = abs(self.vel_x)
        elif distance_from_player < 0.5 and self.player_pose.x > self.ball_pose.x:
            self.vel_x = -abs(self.vel_x)
            
            if player_msg.linear.y == -2.0 and self.vel_y <= 1.5:
                self.vel_y += 0.5
            elif player_msg.linear.y == 2.0 and self.vel_y >= -1.5:
                self.vel_y -= 0.5
            
        # BOUNCE FROM TOP/BOTTOM WALL
        if self.ball_pose.y > 10.5:
            self.vel_y = -abs(self.vel_y)
        elif self.ball_pose.y < 1.0:
            self.vel_y = abs(self.vel_y)
        
        # DETECT SCORING
        if self.ball_pose.x > 10.5:
            self.computer_score += 1
            self.vel_x += 0.1
            self.get_logger().info('----------------------')
            self.get_logger().info('Computer scored.')
            self.get_logger().info('COMPUTER %s : %s PLAYER' % (self.computer_score, self.player_score))
            self.send_teleport_turtle_request()
        elif self.ball_pose.x < 1.0:
            self.player_score += 1
            self.vel_x -= 0.1
            self.get_logger().info('----------------------')
            self.get_logger().info('Player scored.')
            self.get_logger().info('COMPUTER %s : %s PLAYER' % (self.computer_score, self.player_score))
            self.send_teleport_turtle_request()
        
        # MOVE BALL
        ball_msg = Twist()      
        ball_msg.linear.x = self.vel_x
        ball_msg.linear.y = self.vel_y      
        self.ball_pose_pub.publish(ball_msg)


    def send_spawn_turtle_computer_request(self):
        self.spawn_turtle_request.x = 1.0
        self.spawn_turtle_request.y = 5.5
        self.spawn_turtle_request.theta = 0.0
        self.spawn_turtle_request.name = 'computer'

        future = self.spawn_cli.call_async(self.spawn_turtle_request)
        rclpy.spin_until_future_complete(self, future)


    def send_spawn_turtle_player_request(self):
        self.spawn_turtle_request.x = 10.0
        self.spawn_turtle_request.y = 5.5
        self.spawn_turtle_request.theta = 3.14
        self.spawn_turtle_request.name = 'player'

        future = self.spawn_cli.call_async(self.spawn_turtle_request)
        rclpy.spin_until_future_complete(self, future)


    def send_set_pen_ball_request(self):
        self.set_pen_request.off = 1

        future = self.set_pen_ball_cli.call_async(self.set_pen_request)
        rclpy.spin_until_future_complete(self, future)


    def send_set_pen_computer_request(self):
        self.set_pen_request.off = 1

        self.future5 = self.set_pen_computer_cli.call_async(self.set_pen_request)
        rclpy.spin_until_future_complete(self, self.future5)

        return self.future5.result()


    def send_set_pen_player_request(self):
        self.set_pen_request.off = 1

        self.future6 = self.set_pen_player_cli.call_async(self.set_pen_request)
        rclpy.spin_until_future_complete(self, self.future6)

        return self.future6.result()


    def send_teleport_turtle_request(self):
        self.teleport_turtle_request.x = 5.5
        self.teleport_turtle_request.y = 5.5
        
        self.teleport_cli.call_async(self.teleport_turtle_request)


    def ball_pose_sub_callback(self, msg):
        self.ball_pose = msg


    def computer_pose_sub_callback(self, msg):
        self.computer_pose = msg


    def player_pose_sub_callback(self, msg):
        self.player_pose = msg


    def move_player_turtle_sub_callback(self, msg):
        self.direction = msg.direction


def main(args=None):
    rclpy.init(args=args)

    game = Game()

    rclpy.spin(game)
    game.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
