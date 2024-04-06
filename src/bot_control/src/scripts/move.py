#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

class WaypointFollower:
    def __init__(self):
        rospy.init_node('waypoint_follower', anonymous=True)
        self.waypoints = [[0, 0, 0], [5, 0, 45], [5, 5, 90], [0, 5, 0], [0, 0, 135]]
        self.current_waypoint_index = 0
        self.robot_position = [0, 0, 0]
        self.yaw_threshold = 5  # degrees
        self.distance_threshold = 0.1  # meters
        self.angular_velocity = 0.5  # rad/s
        self.linear_velocity = 0.7  # m/s

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)

    def odom_callback(self, odom_msg):
        position = odom_msg.pose.pose.position
        self.robot_position = [position.x, position.y, self.get_yaw(odom_msg)]

    def imu_callback(self, imu_msg):
        pass  # For now, we are not using IMU data

    def get_yaw(self, odom_msg):
        orientation_q = odom_msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        roll, pitch, yaw = euler_from_quaternion(orientation_list)
        return math.degrees(yaw)

    def distance_to_waypoint(self):
        dx = self.waypoints[self.current_waypoint_index][0] - self.robot_position[0]
        dy = self.waypoints[self.current_waypoint_index][1] - self.robot_position[1]
        return math.sqrt(dx**2 + dy**2)

    def angle_to_waypoint(self):
        target_yaw = math.degrees(math.atan2(
            self.waypoints[self.current_waypoint_index][1] - self.robot_position[1],
            self.waypoints[self.current_waypoint_index][0] - self.robot_position[0]
        ))
        return (target_yaw - self.robot_position[2] + 180) % 360 - 180

    def move_to_waypoint(self):
        twist_msg = Twist()
        twist_msg.linear.x = min(self.linear_velocity, self.distance_to_waypoint())  # linear velocity
        twist_msg.angular.z = self.angle_to_waypoint() * 0.01  # angular velocity
        self.cmd_vel_pub.publish(twist_msg)

    def spin(self):
        twist_msg = Twist()
        twist_msg.angular.z = math.radians(10)  # 10 degrees/s clockwise
        rate = rospy.Rate(10)
        for _ in range(360 * 3):  # 3 rotations
            self.cmd_vel_pub.publish(twist_msg)
            rate.sleep()

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.current_waypoint_index < len(self.waypoints):
                if self.distance_to_waypoint() < self.distance_threshold:
                    if abs(self.angle_to_waypoint()) < self.yaw_threshold:
                        self.current_waypoint_index += 1
                    else:
                        self.move_to_waypoint()
                else:
                    self.move_to_waypoint()
            else:
                self.spin()
                break
            rate.sleep()

if __name__ == '__main__':
    try:
        waypoint_follower = WaypointFollower()
        waypoint_follower.run()
    except rospy.ROSInterruptException:
        pass
