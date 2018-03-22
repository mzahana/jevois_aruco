#!/usr/bin/env python

import rospy
import serial

from jevois_aruco.msg import Tag, TagArray



class Aruco():

	def __init__(self):
		# open serial port
		dev 					= rospy.get_param("dev", "/dev/ttyACM0")
		baud					= rospy.get_param("baud", 115200)
		to						= rospy.get_param("sertout", 1.0)
		self.ser			= serial.Serial(dev, baud,timeout=to)

		# send camera configuration commands
		#rospy.sleep(1.) # sleep for 1 sec to make sure port is open
		# configure marker length
		#markerlen = rospy.get_param("markerlen", 142)
		#cmd = "setpar markerlen "+str(markerlen)
		#self.ser.write(cmd)

		self.DEBUG = rospy.get_param("debug", False)

		# number of attempts to read from serial port before publishing to topic
		# helps to get many tags
		self.Nattempts = rospy.get_param("nAttempts", 10)

		# used to publish only if new data is received
		self.newData = False
		
		# Create Markers msg
		self.tagArray	= TagArray()

	def readTags(self):
			if (self.ser.is_open):
				ID_list = []

				self.tagArray	= TagArray()
				self.tagArray.tags = []
				for t in range(self.Nattempts):
					line = self.ser.readline()
					# remove "\n" charcter
					line = line.strip("\n")
					if self.DEBUG:
						rospy.logwarn("Read line: %s",line )
				
					# make sure we have something readable
					if len(line) > 0:
						# get the data fields
						line_split = line.split()

						if self.DEBUG:
							rospy.logwarn("Line after splitting: %s", line_split)

						# get parsing character
						parse_chr = line_split[0]
						# make sure the parsing character is correct
						if parse_chr == "N3":
							# this is "Normal" style in 3D
							# 8 fields are expected: N3 U<id> x y z w h d
							# Example: N3 U24 123 456 345 142 142 1
							# x y z are in milimeters; will be converted to meters
							if len(line_split) == 8:
								self.newData = True
								ID = line_split[1]
								ID = int(ID.replace("U", ""))
								x = int(line_split[2])/1000.0
								y = int(line_split[3])/1000.0
								z = int(line_split[4])/1000.0

								if self.DEBUG:
									rospy.logwarn("Received 3D position of Aruco ID=%s with position x=%s, y=%s, z=%s", ID, x, y, z)

								# filling the topic msg
								if ID not in ID_list:
									ID_list.append(ID)
									tag			= Tag()
									tag.header.stamp = rospy.Time.now()
									tag.id = ID
									tag.pose.position.x = x
									tag.pose.position.y = y
									tag.pose.position.z = z
									self.tagArray.tags.append(tag)
							else:
								rospy.logwarn("Got corrupted data!")
					
			else:
				rospy.logwarn("Serial port is not open.")
				return
					


def main():

	rospy.init_node('jevois_aruco_node', anonymous=True)
	tags_pub = rospy.Publisher('tags', TagArray, queue_size=1)
	
	freq = rospy.get_param("loop_rate", 20)
	rate = rospy.Rate(freq)


	tags = Aruco()

	# Main loop
	while not rospy.is_shutdown():
		tags.readTags()
		if (tags.newData):
			tags.newData = False
			tags_pub.publish(tags.tagArray)

		rate.sleep()

	tags.ser.close()             # close serial port

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
