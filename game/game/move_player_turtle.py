import rclpy
from rclpy.node import Node

from interfaces.msg import Move

from pynput import keyboard


class MovePlayerTurtle(Node):

    def __init__(self):
        super().__init__('move_player_turtle')
        
        # VARIABLE INITIALIZATION
        self.direction = 0
        
        # CREATE PUBLISHER TO MOVE_PLAYER_TURTLE TOPIC
        self.publisher = self.create_publisher(Move, 'move_player_turtle', 10)
        
        # INITIALIZATION TO LISTENING TO KEYBOARD INPUT
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()
        
        # RUN PUBLISHER AT SET TIME
        self.timer = self.create_timer(1 / 30, self.timer_callback)


    def on_press(self, key):
        if key == keyboard.Key.up:
            self.direction = -1
        elif key == keyboard.Key.down:
            self.direction = 1
            
            
    def on_release(self, key):
        self.direction = 0
    

    def timer_callback(self):
        msg = Move()
        msg.direction = self.direction
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    move_player_turtle = MovePlayerTurtle()

    rclpy.spin(move_player_turtle)

    move_player_turtle.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    