#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist


class FollowerController(Node):
    def __init__(self):
        super().__init__('follower_controller')

        self.actor_name = 'actor'
        self.robot_name = 'follower_bot'

        self.follow_distance = 0.8
        self.max_linear = 0.20
        self.max_angular = 0.8

        self.latest_states = None
        self.last_state_time = 0.0

        self.sub = self.create_subscription(
            ModelStates, '/model_states', self.states_callback, 10
        )
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Follower controller started')

    def states_callback(self, msg):
        self.latest_states = msg
        self.last_state_time = time.monotonic()

    def stop_robot(self):
        self.pub.publish(Twist())

    @staticmethod
    def get_yaw(q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    @staticmethod
    def wrap_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def control_loop(self):
        states = self.latest_states

        if states is None or time.monotonic() - self.last_state_time > 1.0:
            self.stop_robot()
            return

        try:
            actor_i = states.name.index(self.actor_name)
            robot_i = states.name.index(self.robot_name)
        except ValueError:
            self.stop_robot()
            return

        actor = states.pose[actor_i]
        robot = states.pose[robot_i]

        dx = actor.position.x - robot.position.x
        dy = actor.position.y - robot.position.y
        distance = math.hypot(dx, dy)

        cmd = Twist()

        if distance <= self.follow_distance:
            self.pub.publish(cmd)
            return

        target_yaw = math.atan2(dy, dx)
        robot_yaw = self.get_yaw(robot.orientation)
        error = self.wrap_angle(target_yaw - robot_yaw)

        # First turn toward the actor; move forward when reasonably aligned.
        cmd.angular.z = max(
            -self.max_angular,
            min(self.max_angular, 1.5 * error)
        )

        if abs(error) < 0.5:
            cmd.linear.x = min(
                self.max_linear,
                0.4 * (distance - self.follow_distance)
            )

        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = FollowerController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
