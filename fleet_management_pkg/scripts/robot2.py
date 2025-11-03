#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

from fleet_management_pkg.srv import GetRobotState

MOVE_DELAY_SEC = 1


class RobotNode(Node):
	def __init__(self, robot_name: str, instruction_topic: str, service_name: str, start_pos=(0, 1)):
		super().__init__(robot_name)
		self.current_x = int(start_pos[0])
		self.current_y = int(start_pos[1])
		self.final_x = int(start_pos[0])
		self.final_y = int(start_pos[1])

		self.sub = self.create_subscription(Point, instruction_topic, self.instruction_callback, 10)
		self.srv = self.create_service(GetRobotState, service_name, self.handle_get_state)

	def handle_get_state(self, request, response):
		response.current_x = int(self.current_x)
		response.current_y = int(self.current_y)
		response.final_x = int(self.final_x)
		response.final_y = int(self.final_y)
		return response

	def instruction_callback(self, point_msg: Point):
		self.final_x = int(point_msg.x)
		self.final_y = int(point_msg.y)
		self.get_logger().info(f"Received instruction to move to ({self.final_x}, {self.final_y})")
		self.delay_timer = self.create_timer(MOVE_DELAY_SEC, self._complete_move_once)

	def _complete_move_once(self):
		self.current_x = self.final_x
		self.current_y = self.final_y
		# Destroy timer
		self.delay_timer.cancel()
		self.get_logger().info(f"Arrived at ({self.current_x}, {self.current_y})")


def main(args=None):
	rclpy.init()
	robot = RobotNode(
		robot_name="robot2_node",
		instruction_topic="/robot2/instruction",
		service_name="/robot2/get_state",
		start_pos=(0, 1)
	)
	print("Robot2 Node Running...")
	try:
		rclpy.spin(robot)
	except KeyboardInterrupt:
		print("Terminating Node...")
	finally:
		robot.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()


