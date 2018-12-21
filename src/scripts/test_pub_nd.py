#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from nd_msg.numpy_nd_msg import numpy_nd_msg
import numpy

def cb(data):
   print "I heard\n",data.data
   print ("len function : {0}".format(data.data.shape))

rospy.init_node('mynode')
pub = rospy.Publisher('mytopic', numpy_nd_msg(Float32MultiArray))
rospy.Subscriber("mytopic", numpy_nd_msg(Float32MultiArray), cb)


r=rospy.Rate(1);
while not rospy.is_shutdown():
      a=numpy.array(numpy.ones((54,250,250)), dtype=numpy.float32) #please ensure the dtype in identifical to the topic type
      print "sending\n", a
      pub.publish(data=a)
      rospy.loginfo("dimension of arr{0}".format(a.shape))
      r.sleep()
