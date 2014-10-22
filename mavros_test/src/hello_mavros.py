#!/usr/bin/env python
import rospy
from std_msgs.msg import *
from std_srvs.srv import Empty
from sensor_msgs.msg import *
from geometry_msgs.msg import TwistStamped
from mavros.msg import *
from mavros.srv import *


# TODO: Change given altitude and observe results
# TODO: Change pitch/yaw and see the results
def launch(latitude, longitude, min_pitch = 0, yaw = 0, altitude = 4):
    service = '/mavros/cmd/takeoff'
    rospy.wait_for_service(service)
    rospy.loginfo('Succesfully found service %s', service)
    try:
        command = rospy.ServiceProxy(service, CommandTOL)
        res = command(min_pitch, yaw, latitude, longitude, altitude)
        print 'res: ',res
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

def land(latitude, longitude, min_pitch = 0, yaw = 0, altitude = 0):
    service = '/mavros/cmd/land'
    rospy.wait_for_service(service)
    rospy.loginfo('Succesfully found service %s', service)
    try:
        command = rospy.ServiceProxy(service, CommandTOL)
        res = command(min_pitch, yaw, latitude, longitude, altitude)
        print 'res: ',res
        if str(res) == 'success: True':
            print 'successfully landing'
            return True
        else:
            print 'error landing'
            return False
    except rospy.ServiceException, e:
        rospy.logwarn('Error encountered: %s', str(e))
        return False
        rospy.loginfo('Ran land')

# TEST Remove this function if it works
def callback(msg):
    rospy.loginfo(rospy.get_caller_id() + 'latitude: %f\tlongitude: %f',
        msg.latitude, msg.longitude)

# TEST Remove this function if it works
def listener():
    topic = '/mavros/fix'
    rospy.Subscriber(topic, NavSatFix, callback)
    rospy.loginfo('Just subscribed to %s', topic)
    rospy.spin()
    rospy.loginfo('Ran listener')

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    lat = 42.292861
    lon = -71.263073
    launch(lat, lon)
    listener()
