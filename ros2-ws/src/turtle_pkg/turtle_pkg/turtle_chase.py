#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import random
import math
from std_msgs.msg import Int32
from turtlesim.srv import Spawn, Kill
from turtlesim.msg import Pose

ENEMY_COUNT = 3
MAP_MIN = 0.5
MAP_MAX = 10.5
COLLISION_DIST = 0.05

class turtlechase(Node):
    def __init__(self):
        super().__init__('turtle_chase')
        self.get_logger().info('started turtle chase game')

        self.score_pub = self.create_publisher(Int32, '/score', 10)
        self.score = 0

        self.player_pos = None
        self.enemies_pos = {f'enemy{i}': None for i in range(1, ENEMY_COUNT + 1)}

        self.spawn_client = self.create_client(Spawn, 'spawn')
        self.kill_client = self.create_client(Kill, 'kill')

        self.create_subscription(Pose, '/turtle1/pos', self.player_pos_cb, 10)


        for i in range(1, ENEMY_COUNT + 1):
            topic = f'/enemy{i}/pos'
            self.create_subscription(Pose, topic, self.make_enemy_cb(f'enemy{i}'), 10)

        self.spawn_initial_enemies()

        self.create_timer(0.1, self.check_collisions)


        self.publish_score()

    def publish_score(self):
        msg = Int32()
        msg.data = self.score
        self.score_pub.publish(msg)

    def player_pos_cb(self, msg):
        self.player_pos = msg

    def make_enemy_cb(self, name):

        def cb(msg):
            self.enemies_pos[name] = msg
        return cb

    def spawn_first_enemies(self):
        for i in range(1, ENEMY_COUNT + 1):
            name = f'enemy{i}'
            self.spawn_enemy(name)

    def random_position(self):
        x = random.uniform(MAP_MIN, MAP_MAX)
        y = random.uniform(MAP_MIN, MAP_MAX)
        return x, y

    def spawn_enemy(self, name):
        req = Spawn.Request()
        x, y = self.random_position()
        req.x = float(x)
        req.y = float(y)
        req.name = name
        try:
            future = self.spawn_client.call_async(req)
            
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.result() is not None:
                resp = future.result()
                self.get_logger().info(f'Spawned {name}')
            else:
                self.get_logger().error(f'error when spawn {name}')
        except Exception as e:
            self.get_logger().error(f'Error when spawning {name}: {e}')

    def kill_enemy(self, name):
        req = Kill.Request()
        req.name = name
        try:
            future = self.kill_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.result() is not None:
                self.get_logger().info(f'Killed {name}')
            else:
                self.get_logger().warn(f'Kill {name}')
        except Exception as e:
            self.get_logger().error(f'Error when killing {name}: {e}')

    def check_collision(self):
        
        if self.player_pos is None:
            return

        px = self.player_pos.x
        py = self.player_pos.y

        for name, pos in list(self.enemies_pos.items()):
            if pos is None:
                continue
            ex = pos.x
            ey = pos.y
            dist = math.hypot(px - ex, py - ey)
            if dist < COLLISION_DIST:
                self.get_logger().info(f'Collision with {name} dist={dist:.4f}')
                self.kill_enemy(name)
                self.spawn_enemy(name)
        
                self.enemies_pos[name] = None
                
                self.score += 1
                self.publish_score()

def main(args=None):
    rclpy.init(args=args)
    node = turtlechase()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down turtle_chase')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
