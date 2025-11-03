from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
	return LaunchDescription([
		Node(
			package="fleet_management_pkg",
			executable="robot1.py",
			name="robot1_node"
		),
		Node(
			package="fleet_management_pkg",
			executable="robot2.py",
			name="robot2_node"
		),
		Node(
			package="fleet_management_pkg",
			executable="fleet_manager.py",
			name="fleet_manager_node"
		),
		Node(
			package="fleet_management_pkg",
			executable="visualizer.py",
			name="visualizer_node"
		)
	])


