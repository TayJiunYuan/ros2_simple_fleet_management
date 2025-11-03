#!/usr/bin/env python3

import json
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt


class Visualizer(Node):
	def __init__(self):
		super().__init__("visualizer_node")
		self.sub = self.create_subscription(String, "/fleet/state", self.state_callback, 10)
		self.state = None
		self._start_plot_thread()

	def state_callback(self, msg: String):
		try:
			self.state = json.loads(msg.data)
		except Exception:
			self.state = None

	def _start_plot_thread(self):
		thread = threading.Thread(target=self._plot_loop, daemon=True)
		thread.start()

	def _plot_loop(self):
		plt.ion()
		fig, ax = plt.subplots(figsize=(5, 5))
		while True:
			ax.clear()
			ax.set_title('Fleet State')
			ax.set_aspect('equal')
			ax.set_xlim(-0.5, 5.5)
			ax.set_ylim(-0.5, 5.5)
			ax.grid(True)
			if self.state is not None:
				gsz = int(self.state.get("grid_size", 6))
				ax.set_xlim(-0.5, gsz - 0.5)
				ax.set_ylim(-0.5, gsz - 0.5)
				# Obstacles
				for ox, oy in self.state.get("obstacles", []):
					ax.add_patch(plt.Rectangle((ox - 0.5, oy - 0.5), 1, 1, color='black'))
				# Goals
				for gx, gy in self.state.get("goals", []):
					ax.add_patch(plt.Circle((gx, gy), 0.25, color='green'))
				# Robots
				for robot in self.state.get("robots", []):
					cx, cy = robot["current"]
					fx, fy = robot["final"]
					ax.add_patch(plt.Circle((cx, cy), 0.3, color='blue'))
					ax.add_patch(plt.Circle((fx, fy), 0.15, color='orange'))
					ax.text(cx + 0.3, cy + 0.3, robot["name"], color='blue')
			plt.pause(0.1)


def main(args=None):
	rclpy.init()
	viz = Visualizer()
	print("Visualizer Node Running...")
	try:
		rclpy.spin(viz)
	except KeyboardInterrupt:
		print("Terminating Node...")
	finally:
		viz.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()


