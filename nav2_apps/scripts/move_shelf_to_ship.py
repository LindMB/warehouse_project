#! /usr/bin/env python3

import rclpy

from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Initial position of the robot in the warehouse map
    init_position = PoseStamped()
    init_position.header.frame_id = 'map'
    init_position.header.stamp = navigator.get_clock().now().to_msg()

    # Robot Initial Pose received on /initialpose topic 
    # when 2D Pose Estimate is set in RViz
    init_position.pose.position.x = 0.014764785766601562
    init_position.pose.position.y = 0.016197383403778076
    init_position.pose.orientation.z = -0.03322007063838769
    init_position.pose.orientation.w = 0.999448061135135

    print('Setting the initial robot pose...')

    navigator.setInitialPose(init_position)

    # The application must wait until AMCL and the Nav2 lifecycle
    # nodes are active before sending a navigation goal
    navigator.waitUntilNav2Active()

    print('Nav2 is active and the robot is localized.')

    # Position in front of the shelf
    #
    # This goal is not placed underneath the shelf 
    # because Nav2 detects the shelf as an obstacle. 
    loading_position = PoseStamped()
    loading_position.header.frame_id = 'map'
    loading_position.header.stamp = navigator.get_clock().now().to_msg()

    # Robot Loading Position received from /goal_pose topic 
    # when 2D Goal Pose is set in RViz
    loading_position.pose.position.x = 5.806089878082275
    loading_position.pose.position.y = -0.3760889172554016
    loading_position.pose.orientation.z = -0.750448112923485
    loading_position.pose.orientation.w = 0.6609293682456395

    print('Navigating to loading_position...')

    navigator.goToPose(loading_position)

    i = 0

    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()

        if feedback and i % 5 == 0:
            estimated_time = Duration.from_msg(
                feedback.estimated_time_remaining
            ).nanoseconds / 1e9

            print(
                'Estimated time to loading_position: '
                + '{0:.0f}'.format(estimated_time)
                + ' seconds.'
            )

    result = navigator.getResult()

    if result == TaskResult.SUCCEEDED:
        print('The robot has reached loading_position.')

    elif result == TaskResult.CANCELED:
        print('Navigation to loading_position was canceled.')
        exit(1)

    elif result == TaskResult.FAILED:
        print('Navigation to loading_position failed.')
        exit(1)

    exit(0)


if __name__ == '__main__':
    main()