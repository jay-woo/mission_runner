#!/usr/bin/env python
import rospy
from std_msgs.msg import *
from std_srvs.srv import *
from sensor_msgs.msg import *
from geometry_msgs.msg import TwistStamped
from mavros.msg import *
from mavros.srv import *

# TEST Remove this function if it works
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

# TEST Remove this function if it works
def listener():
    topic = '/mavros/fix'
    rospy.Subscriber(topic, NavSatFix, callback)
    rospy.loginfo('Just subscribed to %s', topic)
    rospy.spin()
    rospy.loginfo('Ran listener')

def launch(latitude, longitude, min_pitch = 0, yaw = 0, altitude = 4):
    topic = '/mavros/cmd/takeoff'
    # rospy.wait_for_service(topic)
    try:
        command = rospy.ServiceProxy(topic, CommandTOL)
        res = command(min_pitch, yaw, latitude, longitude, altitude)
        print res
        if str(res) == 'success: True':
            print 'successfully launching'
            return True
        else:
            print 'error launching'
            return False
    except rospy.ServiceException, e:
        rospy.logwarn('Error encountered: %s', str(e))
        return False
    rospy.loginfo('Ran launch')

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    launch(latitude = 0.0, longitude = 0.0)
    listener()
    
