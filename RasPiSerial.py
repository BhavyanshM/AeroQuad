import rospy
import serial

import rospy
from std_msgs.msg import String

#ser = serial.Serial('/dev/ttyUSB1',115200)


def callback(data):
	rospy.loginfo("%s",data.data)

def listener():
	rospy.init_node('sub_node')
	rospy.Subscriber("chatter", String, callback)
	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

listener()
#s = [0,1]
#while True:
#	read_serial=ser.readline()
	#s[0] = str(int (ser.readline(),16))
	#print s[0]
#	print read_serial
