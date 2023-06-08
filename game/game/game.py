import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn


class Game(Node):

    def __init__(self):
        super().__init__('game')
        
        self.cli = self.create_client(Spawn, 'spawn')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
            
        self.spawn_turtle_request = Spawn.Request()
        
        self.send_spawn_turtle_request()
        
        self.get_logger().info('Game has been started.')       
        

    def send_spawn_turtle_request(self):
        self.spawn_turtle_request.x = 1.0
        self.spawn_turtle_request.y = 1.0
        self.spawn_turtle_request.theta = 0.0
        
        self.future = self.cli.call_async(self.spawn_turtle_request)
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