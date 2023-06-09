#!/usr/bin/python
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

from math import atan2, pow, sqrt
from random import randint


class Game(Node):

    def __init__(self):
        super().__init__('game')
        
        self.vel_x = 2.0
        self.vel_y = randint(-200, 200)/100
        self.angle = atan2(self.vel_y, self.vel_x)

        self.spawn_cli = self.create_client(Spawn, 'spawn')

        while not self.spawn_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.spawn_turtle_request = Spawn.Request()

        self.ball_pose_sub = self.create_subscription(Pose, 'turtle1/pose', self.ball_pose_sub_callback, 10)
        self.ball_pose_sub

        self.computer_pose_sub = self.create_subscription(Pose, 'computer/pose', self.computer_pose_sub_callback, 10)
        self.computer_pose_sub

        self.player_pose_sub = self.create_subscription(Pose, 'player/pose', self.player_pose_sub_callback, 10)
        self.player_pose_sub

        self.ball_pose_pub = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)        
        self.computer_pose_pub = self.create_publisher(Twist, 'computer/cmd_vel', 10)
        self.player_pose_pub = self.create_publisher(Twist, 'player/cmd_vel', 10)
        
        self.send_spawn_turtle_computer_request()
        self.send_spawn_turtle_player_request()
        
        self.timer = self.create_timer(1 / 30, self.ball_pose_pub_callback)

        self.get_logger().info('Game has been started.')

    def ball_pose_pub_callback(self):
        distance_from_computer = sqrt(pow(self.ball_pose.x - self.computer_pose.x, 2) + pow(self.ball_pose.y - self.computer_pose.y, 2))
        distance_from_player = sqrt(pow(self.ball_pose.x - self.player_pose.x, 2) + pow(self.ball_pose.y - self.player_pose.y, 2))
        
        if self.ball_pose.y > self.computer_pose.y:
            computer_msg = Twist()
            computer_msg.linear.y = self.ball_pose.y / 2
            self.computer_pose_pub.publish(computer_msg)
        elif self.ball_pose.y < self.computer_pose.y:
            computer_msg = Twist()
            computer_msg.linear.y = self.ball_pose.y / 2
            self.computer_pose_pub.publish(computer_msg)
        
        ball_msg = Twist()      
        ball_msg.linear.x = self.vel_x
        ball_msg.linear.y = self.vel_y      
        self.ball_pose_pub.publish(ball_msg)

    def ball_pose_sub_callback(self, msg):
        self.ball_pose = msg

    def computer_pose_sub_callback(self, msg):
        self.computer_pose = msg

    def player_pose_sub_callback(self, msg):
        self.player_pose = msg

    def send_spawn_turtle_computer_request(self):
        self.spawn_turtle_request.x = 1.0
        self.spawn_turtle_request.y = 5.5
        self.spawn_turtle_request.theta = 0.0
        self.spawn_turtle_request.name = 'computer'

        self.future = self.spawn_cli.call_async(self.spawn_turtle_request)
        rclpy.spin_until_future_complete(self, self.future)

        return self.future.result()

    def send_spawn_turtle_player_request(self):
        self.spawn_turtle_request.x = 10.0
        self.spawn_turtle_request.y = 5.5
        self.spawn_turtle_request.theta = 3.14
        self.spawn_turtle_request.name = 'player'

        self.future = self.spawn_cli.call_async(self.spawn_turtle_request)
        rclpy.spin_until_future_complete(self, self.future)

        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    game = Game()

    rclpy.spin(game)
    game.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
