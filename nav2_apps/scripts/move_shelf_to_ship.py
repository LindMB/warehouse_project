#! /usr/bin/env python3

import rclpy

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator


def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Initial position of the robot in the warehouse map
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()

    # Robot Initial Pose received on /initialpose topic 
    # when 2D Pose Estimate is set in RViz
    initial_pose.pose.position.x = 0.014764785766601562
    initial_pose.pose.position.y = 0.016197383403778076
    initial_pose.pose.orientation.z = -0.03322007063838769
    initial_pose.pose.orientation.w = 0.999448061135135

    print('Setting the initial robot pose...')

    navigator.setInitialPose(initial_pose)

    # The application must wait until AMCL and the Nav2 lifecycle
    # nodes are active before sending a navigation goal
    navigator.waitUntilNav2Active()

    print('Nav2 is active and the robot is localized.')

    exit(0)


if __name__ == '__main__':
    main()