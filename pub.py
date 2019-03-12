import serial
import rospy
from std_msgs.msg import String

pub = rospy.Publisher('chatter', String, queue_size=1)
rospy.init_node('pub_node')
r = rospy.Rate(1000) # 10hz
ser = serial.Serial('/dev/ttyUSB1', 115200)

while not rospy.is_shutdown():
	line = ser.readline()
	pub.publish(line)
	r.sleep()
