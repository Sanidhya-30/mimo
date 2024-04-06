#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

class LaserReader:
    def __init__(self):
        rospy.init_node('laser_reader', anonymous=True)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.filtered_scan_pub = rospy.Publisher('/filtered_scan', LaserScan, queue_size=10)

    def scan_callback(self, data):
        filtered_data = self.filter_scan(data)
        self.filtered_scan_pub.publish(filtered_data)

    def filter_scan(self, scan_data):
        filtered_scan = LaserScan()

        filtered_scan.header = scan_data.header
        filtered_scan.angle_min = -1.5708  # -90 degrees in radians
        filtered_scan.angle_max = 2.0944   # 120 degrees in radians
        filtered_scan.angle_increment = scan_data.angle_increment

        angle_min_index = int(((-1.5708 - scan_data.angle_min) / scan_data.angle_increment))
        angle_max_index = int(((2.0944 - scan_data.angle_min) / scan_data.angle_increment))

        filtered_scan.ranges = scan_data.ranges[angle_min_index:angle_max_index+1]
        filtered_scan.range_min = scan_data.range_min
        filtered_scan.range_max = scan_data.range_max

        return filtered_scan

if __name__ == '__main__':
    try:
        laser_reader = LaserReader()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
