#!/usr/bin/env python
import rospy
import csv
import sys
from trajectory_execution_msgs.msg import PointArray
from geometry_msgs.msg import Point

def main():
	rospy.init_node('csv_to_motion')
	pub = rospy.Publisher('trajectory_points', PointArray, queue_size=10)
	file = open(sys.argv[1], 'r')
	rd = csv.reader(file)
	waypoints = PointArray()
	for row in rd:
		point = Point()
		point.x = float(row[0])
		point.y = float(row[1])
		point.z = float(row[2])
		waypoints.points.append(point)
	rospy.sleep(0.2)
	pub.publish(waypoints)
	print ("Published the waypoints")


main()