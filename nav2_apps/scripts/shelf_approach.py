#! /usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rclpy.parameter import Parameter

from geometry_msgs.msg import PointStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


class ShelfApproach(Node):

    def __init__(self):
        super().__init__('shelf_approach', 
            parameter_overrides=[
                Parameter(
                    'use_sim_time',
                    Parameter.Type.BOOL,
                    True
                )
            ]
        )

        self.robot_frame = 'robot_base_footprint'
        self.target_frame = 'cart_frame'
        self.odom_frame = 'odom'

        self.kp_yaw = 1.5
        self.linear_speed = 0.1

        self.distance_to_move_under_shelf = 0.40

        self.distance_error_threshold = 0.04
        self.leg_intensity_threshold = 6000.0
        self.same_leg_group_threshold = 5

        self.last_scan = None

        self.cart_x = 0.0
        self.cart_y = 0.0

        # Transform odom -> cart_fram
        self.cart_frame_transform = None
        self.cart_frame_ready = False
        self.cart_frame_available = False

        self.cart_frame_reached = False
        self.distance_under_shelf_travelled = False

        self.first_odom = True
        self.previous_x = 0.0
        self.previous_y = 0.0
        self.accumulated_distance = 0.0

        self.kp_forward_yaw = 1.5

        # Limit for the robot rotation underneath the shelf
        self.maximum_forward_angular_speed = 0.50

        # Last orientation received from odom
        self.current_odom_yaw = None
        
        # Orientation of the entry axe of the shelf
        # in laser frame and in odom frame
        self.cart_yaw_in_laser_frame = 0.0
        self.cart_yaw_in_odom = 0.0

        self.final_yaw_error_threshold = math.radians(2.0)
        self.kp_final_yaw = 1.5
        self.maximum_final_angular_speed = 0.25

        self.need_to_measure_travelled_distance = False


        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.laserscan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_scan_callback,
            qos
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/diffbot_base_controller/cmd_vel_unstamped',
            10
        )

        self.elevator_up_pub = self.create_publisher(
            String,
            '/elevator_up',
            10
        )

        self.elevator_down_pub = self.create_publisher(
            String,
            '/elevator_down',
            10
        )


        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
            spin_thread=False
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.cart_frame_timer = self.create_timer(
            0.1,
            self.publish_cart_frame_callback
        )

    def laser_scan_callback(self, message):

        self.last_scan = message

    def identify_shelf_leg_index_groups(self, detected_indices):

        if not detected_indices:
            return []

        leg_groups = []
        current_group = [detected_indices[0]]

        for index in detected_indices[1:]:
            previous_index = current_group[-1]

            is_index_in_same_leg_group = (
                index - previous_index < self.same_leg_group_threshold
            )

            if is_index_in_same_leg_group:
                current_group.append(index)

            else:
                leg_groups.append(current_group)
                current_group = [index]

        leg_groups.append(current_group)

        return leg_groups

    @staticmethod
    def is_legs_center_computable(leg_groups):

        return len(leg_groups) >= 2

    def compute_legs_center(self, leg_groups):

        leg_1_group = leg_groups[0]
        leg_2_group = leg_groups[1]

        leg_1_index = leg_1_group[len(leg_1_group) // 2]
        leg_2_index = leg_2_group[len(leg_2_group) // 2]

        leg_1_angle = (
            self.last_scan.angle_min
            + leg_1_index * self.last_scan.angle_increment
        )

        leg_2_angle = (
            self.last_scan.angle_min
            + leg_2_index * self.last_scan.angle_increment
        )

        leg_1_range = self.last_scan.ranges[leg_1_index]
        leg_2_range = self.last_scan.ranges[leg_2_index]

        leg_1_x = leg_1_range * math.cos(leg_1_angle)
        leg_1_y = leg_1_range * math.sin(leg_1_angle)

        leg_2_x = leg_2_range * math.cos(leg_2_angle)
        leg_2_y = leg_2_range * math.sin(leg_2_angle)

        self.cart_x = (leg_1_x + leg_2_x) / 2.0
        self.cart_y = (leg_1_y + leg_2_y) / 2.0

        # Orientation of the line linking both legs
        shelf_edge_yaw = math.atan2(
            leg_2_y - leg_1_y,
            leg_2_x - leg_1_x
        )

        candidate_yaw_1 = self.normalize_angle(
            shelf_edge_yaw + math.pi / 2.0
        )

        candidate_yaw_2 = self.normalize_angle(
            shelf_edge_yaw - math.pi / 2.0
        )

        # Direction from laser frame towards the shelf center
        direction_to_center = math.atan2(self.cart_y, self.cart_x)

        error_1 = abs(self.normalize_angle(candidate_yaw_1 - direction_to_center))
        error_2 = abs(self.normalize_angle(candidate_yaw_2 - direction_to_center))

        # Keep the normal pointing towards the shelf
        if error_1 < error_2:
            self.cart_yaw_in_laser_frame = candidate_yaw_1
        else:
            self.cart_yaw_in_laser_frame = candidate_yaw_2

        print('Shelf center detected at x=' + '{0:.3f}'.format(self.cart_x)
            + ', y=' + '{0:.3f}'.format(self.cart_y) + ' in the laser frame.'
        )

        print('Shelf approach yaw in laser frame: ' 
            + '{0:.2f}'.format(math.degrees(self.cart_yaw_in_laser_frame))
            + ' degrees.'
        )

    def prepare_cart_frame_transform(self):

        laser_frame = self.last_scan.header.frame_id

        cart_point_laser = PointStamped()
        cart_point_laser.header.frame_id = laser_frame
        cart_point_laser.header.stamp = self.last_scan.header.stamp

        cart_point_laser.point.x = self.cart_x
        cart_point_laser.point.y = self.cart_y
        cart_point_laser.point.z = 0.0

        # Since the TF Buffer is still being filled,
        # we have to wait when laser -> odom is available before creating cart_frame
        start_time = time.monotonic()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.tf_buffer.can_transform(
                self.odom_frame,
                laser_frame,
                Time()
            ):
                break

            if (time.monotonic() - start_time > 5.0) :
                print('TF from ' + laser_frame
                    + ' to ' + self.odom_frame + ' is unavailable.'
                )
                return False

        try:
            # Time() <=> Python equivalent of tf2::TimePointZero 
            # => Ask for the most recent TF
            laser_to_odom = self.tf_buffer.lookup_transform(
                self.odom_frame,
                laser_frame,
                Time()
            )

        except TransformException as exception:
            print('Could not transform the shelf center to odom: '+ str(exception) )
            return False

        cart_point_odom = do_transform_point(
            cart_point_laser,
            laser_to_odom
        )

        laser_yaw_in_odom = self.quaternion_to_yaw(
            laser_to_odom.transform.rotation
        )

        self.cart_yaw_in_odom = self.normalize_angle(
            laser_yaw_in_odom
            + self.cart_yaw_in_laser_frame
        )

        self.cart_frame_transform = TransformStamped()
        self.cart_frame_transform.header.frame_id = self.odom_frame
        self.cart_frame_transform.child_frame_id = self.target_frame

        self.cart_frame_transform.transform.translation.x = (
            cart_point_odom.point.x
        )
        self.cart_frame_transform.transform.translation.y = (
            cart_point_odom.point.y
        )
        self.cart_frame_transform.transform.translation.z = 0.0

        self.cart_frame_transform.transform.rotation.x = 0.0
        self.cart_frame_transform.transform.rotation.y = 0.0

        self.cart_frame_transform.transform.rotation.z = math.sin(
            self.cart_yaw_in_odom / 2.0
        )

        self.cart_frame_transform.transform.rotation.w = math.cos(
            self.cart_yaw_in_odom / 2.0
        )

        self.cart_frame_ready = True

        print('cart_frame prepared in odom at x='
            + '{0:.3f}'.format(cart_point_odom.point.x) 
            + ', y=' + '{0:.3f}'.format(cart_point_odom.point.y) + '.'
        )

        return True

    def publish_cart_frame_callback(self):

        if not self.cart_frame_ready :
            return

        self.cart_frame_transform.header.stamp = (self.get_clock().now().to_msg())

        self.tf_broadcaster.sendTransform(self.cart_frame_transform)

        self.cart_frame_available = True

    def compute_robot_to_cart_error(self):

        if (not self.cart_frame_available or self.cart_frame_reached) :
            return None

        try:
            transform = self.tf_buffer.lookup_transform(
                self.robot_frame,
                self.target_frame,
                Time()
            )

        except TransformException as exception:
            print('TF from ' + self.robot_frame
                + ' to ' + self.target_frame + ' unavailable: '
                + str(exception)
            )
            return None

        x = transform.transform.translation.x
        y = transform.transform.translation.y

        error_distance = math.sqrt(x * x + y * y)
        error_heading = math.atan2(y, x)

        # cart_frame rotation given in robot_base_footprint frame
        # represents directly the gap of orientation remaining
        error_yaw = self.quaternion_to_yaw(transform.transform.rotation)
        error_yaw = self.normalize_angle(error_yaw)

        return (error_distance, error_heading, error_yaw)

    def move_robot_to_cart_frame(self, error_distance, error_heading, error_yaw):

        move_msg = Twist()

        if (error_distance > self.distance_error_threshold) :

            move_msg.linear.x = self.linear_speed

            angular_command = (self.kp_yaw * error_heading)

            move_msg.angular.z = max(
                -1.0,
                min(angular_command, 1.0)
            )

            print( 'Distance to cart_frame: '
                + '{0:.3f}'.format(error_distance) + ' m | heading error: '
                + '{0:.2f}'.format(math.degrees(error_heading)) + ' degrees'
            )

        # If the robot is at the right position but its orientation 
        # is not corresponding yet to the entry axis of the shelf
        elif (abs(error_yaw) > self.final_yaw_error_threshold) :

            move_msg.linear.x = 0.0

            angular_command = (self.kp_final_yaw * error_yaw)

            move_msg.angular.z = max(
                -self.maximum_final_angular_speed,
                min(angular_command, self.maximum_final_angular_speed)
            )

            print('Aligning with cart_frame: '
                + '{0:.2f}'.format(math.degrees(error_yaw)) + ' degrees remaining'
            )

        else:
            move_msg.linear.x = 0.0
            move_msg.angular.z = 0.0

            self.cart_frame_reached = True

            self.first_odom = True
            self.accumulated_distance = 0.0

            self.need_to_measure_travelled_distance = True

            print('The robot has reached cart_frame and is aligned with the shelf.')

        self.cmd_vel_pub.publish(move_msg)

    @staticmethod
    def quaternion_to_yaw(quaternion):

        sin_yaw = 2.0 * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        )

        cos_yaw = 1.0 - 2.0 * (
            quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
        )

        return math.atan2(sin_yaw, cos_yaw)

    @staticmethod
    def normalize_angle(angle):

        if (angle > math.pi) :
            angle -= 2.0 * math.pi

        elif (angle < -math.pi) :
            angle += 2.0 * math.pi

        return angle

    def odom_callback(self, message):

        self.current_odom_yaw = self.quaternion_to_yaw(
            message.pose.pose.orientation
        )

        """
        if not self.cart_frame_reached:
            return

        if self.distance_under_shelf_travelled:
            return
        """
        if not self.need_to_measure_travelled_distance:
            return

        current_x = message.pose.pose.position.x
        current_y = message.pose.pose.position.y

        if self.first_odom:
            self.previous_x = current_x
            self.previous_y = current_y

            self.first_odom = False
            return

        dx = current_x - self.previous_x
        dy = current_y - self.previous_y

        self.accumulated_distance += math.sqrt(dx * dx + dy * dy)

        self.previous_x = current_x
        self.previous_y = current_y

    def move_forward(self):

        move_forward_msg = Twist()
        move_forward_msg.linear.x = self.linear_speed

        if self.current_odom_yaw is None:
            move_forward_msg.angular.z = 0.0

        else:
            yaw_error = self.normalize_angle(
                self.cart_yaw_in_odom - self.current_odom_yaw
            )

            angular_command = (self.kp_forward_yaw * yaw_error)

            move_forward_msg.angular.z = max(
                - self.maximum_forward_angular_speed,
                min(
                    angular_command,
                    self.maximum_forward_angular_speed
                )
            )

            print('Forward yaw error: ' + '{0:.2f}'.format(math.degrees(yaw_error))
                + ' degrees'
            )

        self.cmd_vel_pub.publish(move_forward_msg)

    def move_backward(self):

        move_backward_msg = Twist()
        move_backward_msg.linear.x = - self.linear_speed

        if self.current_odom_yaw is None:
            move_backward_msg.angular.z = 0.0

        else:

            yaw_error = self.normalize_angle(
                self.cart_yaw_in_odom - self.current_odom_yaw
            )

            angular_command = (self.kp_forward_yaw * yaw_error)

            move_backward_msg.angular.z = max(
                -self.maximum_forward_angular_speed,
                min(
                    angular_command,
                    self.maximum_forward_angular_speed
                )
            )

            print('Backward yaw error: '
                + '{0:.2f}'.format(math.degrees(yaw_error)) + ' degrees'
            )

        self.cmd_vel_pub.publish(move_backward_msg)

    def stop_robot(self):

        self.cmd_vel_pub.publish(Twist())

    def wait_for_first_scan(self, timeout_seconds=10.0):

        start_time = time.monotonic()

        while (rclpy.ok() and self.last_scan is None) :

            rclpy.spin_once(self, timeout_sec=0.1)

            if (time.monotonic() - start_time > timeout_seconds) :
                print('No LaserScan message was received.')
                return False

        return True

    def detect_shelf_center(self):

        if not self.wait_for_first_scan() :
            return False

        shelf_leg_detected_indices = []

        for index, intensity in enumerate(self.last_scan.intensities) :
            if index >= len(self.last_scan.ranges):
                break

            laser_range = self.last_scan.ranges[index]

            is_valid_range = math.isfinite(laser_range)

            is_shelf_leg_detected = (intensity > self.leg_intensity_threshold)

            if (is_valid_range and is_shelf_leg_detected):
                shelf_leg_detected_indices.append(index)

        leg_groups = (
            self.identify_shelf_leg_index_groups(
                shelf_leg_detected_indices
            )
        )

        print('Number of detected leg groups: ' + str(len(leg_groups)) )

        if not self.is_legs_center_computable(leg_groups):
            print('The center between the shelf legs cannot be computed.')
            return False

        self.compute_legs_center(leg_groups)

        return self.prepare_cart_frame_transform()

    def reset_approach_state(self):

        self.cart_frame_reached = False
        self.distance_under_shelf_travelled = False

        self.first_odom = True
        self.previous_x = 0.0
        self.previous_y = 0.0
        self.accumulated_distance = 0.0

        self.cart_frame_transform = None
        self.cart_frame_ready = False
        self.cart_frame_available = False

    def move_under_shelf(self, timeout_seconds=60.0):

        self.reset_approach_state()

        print('Detecting the shelf legs...')

        if not self.detect_shelf_center():
            self.stop_robot()
            return False

        print('Starting the final approach under the shelf...')

        start_time = time.monotonic()

        try:
            while (rclpy.ok() and not self.distance_under_shelf_travelled) :
            
                rclpy.spin_once(self, timeout_sec=0.1)

                if not self.cart_frame_reached:
                    errors = (self.compute_robot_to_cart_error())

                    if errors is not None:
                        (error_distance, error_heading, error_yaw) = errors

                        self.move_robot_to_cart_frame(
                            error_distance,
                            error_heading,
                            error_yaw
                        )

                else:
                    if (self.accumulated_distance < self.distance_to_move_under_shelf) :
                        
                        self.move_forward()

                        print('Distance travelled under the shelf: '
                            + '{0:.3f}'.format(self.accumulated_distance) + ' m'
                        )

                    else:
                        self.stop_robot()

                        self.distance_under_shelf_travelled = True
                        self.need_to_measure_travelled_distance = False

                        print('The robot is correctly positioned under the shelf.')

                if (time.monotonic() - start_time > timeout_seconds) :

                    print('The final shelf approach timed out.')

                    self.stop_robot()
                    return False
            


        except KeyboardInterrupt:
            self.stop_robot()
            raise

        self.stop_robot()
        return True

    def put_elevator_up(self):

        elevator_msg = String()
        elevator_msg.data = ''

        print('Raising the elevator...')

        # Publish elevator_msg 3 times as it asks for the real robot
        for _ in range(3):
            self.elevator_up_pub.publish(elevator_msg)

            rclpy.spin_once(self, timeout_sec=0.2)

        # Wait before Nav2 takes back of the robot control
        time.sleep(2.0)

        print('The robot lifted the shelf successfully.')

    def put_elevator_down(self):

        elevator_msg = String()
        elevator_msg.data = ''

        print('Putting down the elevator...')

        # Publish elevator_msg 3 times as it asks for the real robot
        for _ in range(3):
            self.elevator_down_pub.publish(elevator_msg)

            rclpy.spin_once(self, timeout_sec=0.2)

        # Wait before Nav2 takes back of the robot control
        time.sleep(2.0)

        print('The robot put down the shelf successfully.')

    def move_out_of_loading_area(self, distance=0.50, timeout_seconds=20.0):

        if self.current_odom_yaw is None:

            print('No odometry orientation is available for the backward maneuver.')
            self.stop_robot()
            return False

        # New distance measurement starts for the backward maneuver
        self.first_odom = True
        self.accumulated_distance = 0.0

        self.need_to_measure_travelled_distance = True

        print('Moving backward from the loading area...')

        start_time = time.monotonic()

        try:
            while (rclpy.ok() and self.accumulated_distance < distance):

                rclpy.spin_once(self, timeout_sec=0.1)

                self.move_backward()

                #print('Backward distance travelled: '
                #    + '{0:.3f}'.format(self.accumulated_distance)
                #    + ' / ' + '{0:.3f}'.format(distance) + ' m'
                #)

                if (time.monotonic() - start_time > timeout_seconds):
                    print('The backward maneuver timed out.')

                    self.stop_robot()
                    self.need_to_measure_travelled_distance = False
                    return False

        except KeyboardInterrupt:
            self.stop_robot()
            self.need_to_measure_travelled_distance = False
            raise

        self.stop_robot()
        self.need_to_measure_travelled_distance = False

        print('The robot has cleared the loading area.')

        return True