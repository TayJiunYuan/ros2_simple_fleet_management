#!/usr/bin/env python3

import heapq
import json
from typing import List, Tuple, Set

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String

from fleet_management_pkg.srv import GetRobotState

TICK_SEC = 3
GRID_SIZE = 6
OBSTACLES: Set[Tuple[int, int]] = {(2, 2), (2, 3), (3, 2), (3, 3)}


class FleetManager(Node):
	def __init__(self):
		super().__init__("fleet_manager_node")
		# Publishers per robot for instructions
		self.robot1_pub = self.create_publisher(Point, "/robot1/instruction", 10)
		self.robot2_pub = self.create_publisher(Point, "/robot2/instruction", 10)
		# State publisher for visualizer
		self.state_pub = self.create_publisher(String, "/fleet/state", 10)
		# Subscribe to new goals
		self.new_goal_sub = self.create_subscription(Point, "/fleet/new_goal", self.new_goal_callback, 10)
		# Service clients
		self.robot1_client = self.create_client(GetRobotState, "/robot1/get_state")
		self.robot2_client = self.create_client(GetRobotState, "/robot2/get_state")
		# Future of the service calls to the robots
		self.robot1_pos_future = None
		self.robot2_pos_future = None
		# Results
		self.robot1_pos_result = None
		self.robot2_pos_result = None

		# Goals list as tuples
		self.goals: List[Tuple[int, int]] = [(1, 4), (4, 1)]

		self.timer = self.create_timer(TICK_SEC, self.on_tick)

	def new_goal_callback(self, msg: Point):
		gx, gy = int(msg.x), int(msg.y)
		if 0 <= gx < GRID_SIZE and 0 <= gy < GRID_SIZE and (gx, gy) not in OBSTACLES:
			self.goals.append((gx, gy))
			self.get_logger().info(f"Added new goal ({gx}, {gy})")
		else:
			self.get_logger().warn("Ignored invalid new goal")

	def on_tick(self):
		# 1) Query robot states
		self.robot1_pos_result = None
		self.robot2_pos_result = None
		if not self.robot1_client.wait_for_service():
			return None
		if not self.robot2_client.wait_for_service():
			return None
		robot1_request = GetRobotState.Request()
		robot2_request = GetRobotState.Request()
		self.robot1_pos_future = self.robot1_client.call_async(robot1_request)
		self.robot1_pos_future.add_done_callback(self.after_robot1)

		self.robot2_pos_future = self.robot2_client.call_async(robot2_request)
		self.robot2_pos_future.add_done_callback(self.after_robot2)

	def after_robot1(self, future):
		try:
			self.robot1_pos_result = future.result()
			self.get_logger().info(f"Received Robot 1 response: {future.result()}")
		except Exception as e:
			self.get_logger().info("Robot 1 failed to get response")
			self.robot1_pos_result = None
		self.check_both_robots_done()

	def after_robot2(self, future):
		try:
			self.robot2_pos_result = future.result()
			self.get_logger().info(f"Received Robot 2 response: {future.result()}")
		except Exception as e:
			self.get_logger().info("Robot 2 failed to get response")
			self.robot2_pos_result = None
		self.check_both_robots_done()

	def check_both_robots_done(self):

		if self.robot1_pos_result is not None and self.robot2_pos_result is not None:
			self.get_logger().info("Both robots result obtained. Continuing")
			self.process_new_state()


	def process_new_state(self):
		state1 = self.robot1_pos_result
		state2 = self.robot2_pos_result
		c1 = (state1.current_x, state1.current_y)
		f1 = (state1.final_x, state1.final_y)
		c2 = (state2.current_x, state2.current_y)
		f2 = (state2.final_x, state2.final_y)

		idle1 = c1 == f1
		idle2 = c2 == f2

		# Publish full state
		state_msg = {
			"grid_size": GRID_SIZE,
			"obstacles": list([list(p) for p in OBSTACLES]),
			"robots": [
				{"name": "robot1", "current": [c1[0], c1[1]], "final": [f1[0], f1[1]], "idle": idle1},
				{"name": "robot2", "current": [c2[0], c2[1]], "final": [f2[0], f2[1]], "idle": idle2}
			],
			"goals": list([list(g) for g in self.goals])
		}
		out = String()
		out.data = json.dumps(state_msg)
		self.get_logger().info(f"Full State: {out.data}")
		self.state_pub.publish(out)

		# Assign goals to idle robots and send next step avoiding collisions
		occupied = set(OBSTACLES)
		# Treat other robot's current and final as obstacles to avoid collision and target conflict
		occupied.update([c1, f1, c2, f2])

		robot1_sorted_goals = []
		robot2_sorted_goals = []
		assignments = []
		if self.goals:
			# simple greedy: nearest goal to each idle robot
			if idle1:
				robot1_sorted_goals = self._sorted_goals(c1, self.goals)
			if idle2:
				robot2_sorted_goals = self._sorted_goals(c2, self.goals)

			if idle1 and not idle2:
				assignments.append(("robot1", c1, robot1_sorted_goals[0]))
			
			elif idle2 and not idle1:
				assignments.append(("robot2", c2, robot2_sorted_goals[0]))
			
			elif not idle1 and not idle2:
				return
			
			elif len(self.goals) == 1: # both robots idle and only 1 goal, assign to closer robot (or robot1 if tie)
				if self._manhattan(c1, robot1_sorted_goals[0]) > self._manhattan(c2, robot2_sorted_goals[0]):
					assignments.append(("robot2", c2, robot2_sorted_goals[0]))
				else:
					assignments.append(("robot1", c1, robot1_sorted_goals[0]))
			elif len(self.goals) == 2: # both robots idle and 2 or more goals
				if robot1_sorted_goals[0] == robot2_sorted_goals[0]:	# If tie on first goal, then the lower second closest goal will take the second point
					if self._manhattan(c1, robot1_sorted_goals[1]) < self._manhattan(c2, robot2_sorted_goals[1]):
						assignments.append(("robot1", c1, robot1_sorted_goals[1]))
						assignments.append(("robot2", c2, robot2_sorted_goals[0]))
					else:
						assignments.append(("robot1", c1, robot1_sorted_goals[0]))
						assignments.append(("robot2", c2, robot2_sorted_goals[1]))
				else:	# no tie
						assignments.append(("robot1", c1, robot1_sorted_goals[0]))
						assignments.append(("robot2", c2, robot2_sorted_goals[0]))

			for name, cur, goal in assignments:
				next_step = self._next_step_astar(cur, goal, occupied - {cur})
				if next_step is None:
					continue
				pt = Point()
				pt.x = float(next_step[0])
				pt.y = float(next_step[1])
				pt.z = 0.0
				if name == "robot1":
					self.robot1_pub.publish(pt)
				elif name == "robot2":
					self.robot2_pub.publish(pt)
				# If robot reached the goal, remove it from goals
				if next_step == goal:
					try:
						self.goals.remove(goal)
					except ValueError:
						pass
		

	def _sorted_goals(self, start: Tuple[int, int], goals: List[Tuple[int, int]]):
		if not goals:
			return None
		sorted_goals = sorted(goals, key=lambda g: self._manhattan(start, g))
		return sorted_goals

	def _manhattan(self, a: Tuple[int, int], b: Tuple[int, int]) -> int:
		return abs(a[0] - b[0]) + abs(a[1] - b[1])

	def _neighbors(self, node: Tuple[int, int]) -> List[Tuple[int, int]]:
		x, y = node
		candidates = [(x+1, y), (x-1, y), (x, y+1), (x, y-1)]
		valid = []
		for nx, ny in candidates:
			if 0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE:
				valid.append((nx, ny))
		return valid

	def _next_step_astar(self, start: Tuple[int, int], goal: Tuple[int, int], blocked: Set[Tuple[int, int]]):
		if start == goal:
			return goal
		open_set = []
		heapq.heappush(open_set, (0 + self._manhattan(start, goal), 0, start, None))
		came_from = {}
		g_score = {start: 0}
		visited = set()
		while open_set:
			_, g, node, parent = heapq.heappop(open_set)
			if node in visited:
				continue
			visited.add(node)
			came_from[node] = parent
			if node == goal:
				break
			for nb in self._neighbors(node):
				if nb in blocked and nb != goal:
					continue
				ng = g + 1
				if ng < g_score.get(nb, 1e9):
					g_score[nb] = ng
					f = ng + self._manhattan(nb, goal)
					heapq.heappush(open_set, (f, ng, nb, node))
		# Reconstruct path
		if goal not in came_from:
			return None
		path = []
		cur = goal
		while cur is not None:
			path.append(cur)
			cur = came_from.get(cur)
		path.reverse()
		# Return next step
		return path[1] if len(path) > 1 else goal


def main(args=None):
	rclpy.init()
	manager = FleetManager()
	print("Fleet Manager Node Running...")
	try:
		rclpy.spin(manager)
	except KeyboardInterrupt:
		print("Terminating Node...")
	finally:
		manager.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()


