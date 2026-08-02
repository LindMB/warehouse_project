#! /usr/bin/env python3

import rclpy
import time
import math

from rclpy.duration import Duration
from rclpy.parameter import Parameter

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point32, Polygon
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from shelf_approach import ShelfApproach

def create_circular_footprint(robot_radius, nb_points=16):

    footprint = []

    for i in range(nb_points):

        angle = 2.0 * math.pi * i / nb_points

        footprint.append(
            Point32(
                x=robot_radius * math.cos(angle),
                y=robot_radius * math.sin(angle),
                z=0.0
            )
        )
    
    return footprint

def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Nav2 Simple Commander has to use the same clock as Gazebo, 
    # TF, Nav2, LaserScan, Odometry
    navigator.set_parameters([
        Parameter(
            'use_sim_time',
            Parameter.Type.BOOL,
            True
        )
    ])

    shelf_approach = ShelfApproach(use_simulation=False)

    # Initial position of the robot in the warehouse map
    init_position = PoseStamped()
    init_position.header.frame_id = 'map'
    init_position.header.stamp = navigator.get_clock().now().to_msg()

    # Robot Initial Pose received on /initialpose topic 
    # when 2D Pose Estimate is set in RViz
    init_position.pose.position.x =  -0.3507683277130127
    init_position.pose.position.y = -0.12274336814880371
    init_position.pose.orientation.z = 0.0027034062069907624
    init_position.pose.orientation.w = 0.9999963457907634

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
    loading_position.pose.position.x = 4.329412937164307
    loading_position.pose.position.y = -0.321516215801239
    loading_position.pose.orientation.z = 0.0027034062069907624
    loading_position.pose.orientation.w = 0.9999963457907634

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

            print('Estimated time to loading_position: '
                + '{0:.0f}'.format(estimated_time) + ' seconds.'
            )

    result = navigator.getResult()

    if result == TaskResult.SUCCEEDED:
        print('The robot has reached loading_position.')

        if not shelf_approach.rotate_by_angle(math.radians(-92.0)):
            print('The robot could not align itself with the shelf.')
            exit(1)

        if not shelf_approach.move_under_shelf():
            print('The final shelf approach failed.')
            exit(1)

        ### Put the elevator up
        shelf_approach.put_elevator_up()

        ### Update footprint shape from robot only to robot + shelf 
        global_footprint_pub = navigator.create_publisher(
            Polygon,
            '/global_costmap/footprint',
            10
        )

        local_footprint_pub = navigator.create_publisher(
            Polygon,
            '/local_costmap/footprint',
            10
        )

        loaded_footprint = Polygon()

        safety_margin = 0.05
        x_val = 0.40 + safety_margin
        y_val = 0.30 + safety_margin

        # Rectangular shape definition (robot + shelf)
        loaded_footprint.points = [
            Point32(x=x_val, y=y_val, z=0.0),
            Point32(x=x_val, y=-y_val, z=0.0),
            Point32(x=-x_val, y=-y_val, z=0.0),
            Point32(x=-x_val, y=y_val, z=0.0)
        ]

        print('Updating the robot footprint for shelf transport...')

        # Publish the footprint several times so both Costmaps
        # reliably receive the new geometry
        for _ in range(5):
            global_footprint_pub.publish(loaded_footprint)
            local_footprint_pub.publish(loaded_footprint)
            time.sleep(0.1)

        ### Move out of loading position by moving the robot backward
        if not shelf_approach.move_out_of_loading_area(distance=1.2):
            print('The robot could not leave the loading area.')
            exit(1)

        ### Navigate to the middle point of the shipping position
        # Robot Middle Position (between both tables)
        middle_position = PoseStamped()
        middle_position.header.frame_id = 'map'
        middle_position.header.stamp = navigator.get_clock().now().to_msg()

        # Robot Middle Position received from /goal_pose topic 
        # when 2D Goal Pose is set in RViz
        middle_position.pose.position.x = 1.980424404144287
        middle_position.pose.position.y = -0.28854334354400635
        middle_position.pose.orientation.z = 0.708902819833417
        middle_position.pose.orientation.w = 0.7053061690019661

        print('Navigating to middle_position...')

        navigator.goToPose(middle_position)

        while not navigator.isTaskComplete():
            i = i + 1
            feedback = navigator.getFeedback()

            if feedback and i % 5 == 0:
                estimated_time = Duration.from_msg(
                    feedback.estimated_time_remaining
                ).nanoseconds / 1e9

                print('Estimated time to middle_position: '
                    + '{0:.0f}'.format(estimated_time) + ' seconds.'
                )

        result = navigator.getResult()

        if result == TaskResult.SUCCEEDED:
                print('The robot has returned to middle_position.')

                ### Navigate to the shipping position
                # Robot Shipping Position (between both tables)
                shipping_position = PoseStamped()
                shipping_position.header.frame_id = 'map'
                shipping_position.header.stamp = navigator.get_clock().now().to_msg()

                # Robot Shipping Position received from /goal_pose topic 
                # when 2D Goal Pose is set in RViz
                shipping_position.pose.position.x = 2.004753828048706
                shipping_position.pose.position.y = 1.128702998161316
                shipping_position.pose.orientation.z = 0.7091976675988844
                shipping_position.pose.orientation.w = 0.7050096937434991

                print('Navigating to shipping_position...')

                navigator.goToPose(shipping_position)

                i = 0

                while not navigator.isTaskComplete():
                    i = i + 1
                    feedback = navigator.getFeedback()

                    if feedback and i % 5 == 0:
                        estimated_time = Duration.from_msg(
                            feedback.estimated_time_remaining
                        ).nanoseconds / 1e9

                        print('Estimated time to shipping_position: '
                            + '{0:.0f}'.format(estimated_time) + ' seconds.'
                        )

                result = navigator.getResult()

                if result == TaskResult.SUCCEEDED:
                    print('The robot has reached shipping_position.')

                    ### Put the elevator down
                    shelf_approach.put_elevator_down()

                    ### Update footprint shape from robot + shelf to robot only
                    global_footprint_pub = navigator.create_publisher(
                        Polygon,
                        '/global_costmap/footprint',
                        10
                    )

                    local_footprint_pub = navigator.create_publisher(
                        Polygon,
                        '/local_costmap/footprint',
                        10
                    )

                    robot_footprint = Polygon()

                    # Circular shape definition
                    robot_footprint.points = create_circular_footprint(0.30)

                    print('Restoring the normal robot footprint...')

                    for i in range(5):
                        global_footprint_pub.publish(robot_footprint)
                        local_footprint_pub.publish(robot_footprint)
                        time.sleep(0.1)

                    ### Move out of loading position by moving the robot backward
                    if not shelf_approach.move_out_of_shipping_area(distance=1.5):
                        print('The robot could not leave the shipping area.')
                        exit(1)

                    # Refresh the timestamp before reusing the initial pose.
                    init_position.header.stamp = (
                        navigator.get_clock().now().to_msg()
                    )

                    print('Returning to init_position...')

                    navigator.goToPose(init_position)

                    while not navigator.isTaskComplete():
                        pass

                    result = navigator.getResult()

                    if result == TaskResult.SUCCEEDED:
                        print('The robot has returned to init_position.')

                    elif result == TaskResult.CANCELED:
                        print('Navigation to init_position was canceled.')
                        exit(1)

                    elif result == TaskResult.FAILED:
                        print('Navigation to init_position failed.')
                        exit(1)


                elif result == TaskResult.CANCELED:
                    print('Navigation to shipping_position was canceled.')
                    exit(1)

                elif result == TaskResult.FAILED:
                    print('Navigation to shipping_position failed.')
                    exit(1) 


        elif result == TaskResult.CANCELED:
            print('Navigation to middle_position was canceled.')
            exit(1)

        elif result == TaskResult.FAILED:
            print('Navigation to middle_position failed.')
            exit(1) 


    elif result == TaskResult.CANCELED:
        print('Navigation to loading_position was canceled.')
        exit(1)

    elif result == TaskResult.FAILED:
        print('Navigation to loading_position failed.')
        exit(1)

    exit(0)


if __name__ == '__main__':
    main()