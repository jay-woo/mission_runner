#!/usr/bin/env python
import rospy
from std_msgs.msg import *
from std_srvs.srv import Empty
from sensor_msgs.msg import *
from geometry_msgs.msg import TwistStamped
from mavros.msg import *
from mavros.srv import *
from service import *


class Quadcopter(object):
    def __init__(self):
        super(Quadcopter, self).__init__()
        self.SUBSCRIBE_TIMEOUT = 3.0

        # Subscribe to necessary topics
        self.latest_longitude = -1.0
        self.latest_latitude = -1.0
        topic = '/mavros/fix'
        rospy.Subscriber(topic, NavSatFix, gps_callback)
        rospy.loginfo('Just subscribed to %s', topic)
        subscribe_timer = 0.0
        while (self.latest_longitude == -1 or self.latest_latitude == -1) and\
              subscribe_timer < self.SUBSCRIBE_TIMEOUT:
            rospy.sleep(0.1)
            subscribe_timer += 0.1
        if not did_subscribe_succeed(subscribe_timer, self.SUBSCRIBE_TIMEOUT,
                                     topic):
            # TODO: decide what the right thing to do if subscription fails
            pass

        # Create necessary service proxies
        name = '/mavros/cmd/takeoff'
        self.launcher = Service(name, self.SUBSCRIBE_TIMEOUT, 'success: True',
                         CommandTOL)
        name = '/mavros/cmd/land'
        self.lander = Service(name, self.SUBSCRIBE_TIMEOUT, 'success: True',
                 CommandTOL)

    def launch(self, latitude, longitude, min_pitch = 0, yaw = 0, altitude = 4):
        # Uncomment this after testing with hand_entered points and confirming
        # that latest_latitude/longitude is consistently right
        # current_longitude = self.latest_longitude
        # current_latitude = self.latest_latitude
        try:
            res = self.cmd_takeoff(min_pitch, yaw, latitude, longitude, altitude)
            return evaluate_service(res)
        except rospy.ServiceException, e:
            rospy.logwarn('Error encountered: %s', str(e))
            return False
        rospy.loginfo('Ran launch')

    def land(self, latitude, longitude, min_pitch = 0, yaw = 0, altitude = 4):
        # Uncomment this after testing with hand_entered points and confirming
        # that latest_latitude/longitude is consistently right
        # current_longitude = self.latest_longitude
        # current_latitude = self.latest_latitude
        try:
            res = self.cmd_land(min_pitch, yaw, latitude, longitude, altitude)
            return evaluate_service(res)
        except rospy.ServiceException, e:
            rospy.logwarn('Error encountered: %s', str(e))
            return False
        rospy.loginfo('Ran land')

    def gps_callback(self, msg):
        rospy.loginfo(rospy.get_caller_id() + 'latitude: %f\tlongitude: %f',
                      msg.latitude, msg.longitude)
        self.latest_longitude = msg.longitude
        self.latest_latitude  = msg.latitude

def did_subscribe_succeed(timer, timeout, topic):
    if timer >= timout:
        rospy.logwarn("Quadcopter FAILED to subscribe to topic %s", topic)
        return False
    else:
        rospy.loginfo("Quadcopter successfully subscribed to topic %s", topic)
        return True

def annotated_timer(wait_time = 10.0):
    """ Waits for a certain amount of time and prints out updates as it does so.
    Intended to make testing easier so we can time things. Time in seconds"""
    sleep_unit = 5.0
    timer = 0.0
    rospy.loginfo('Waiting for 0.0/%.1f seconds', wait_time)
    while not rospy.is_shutdown() and timer < wait_time:
        rospy.sleep(sleep_unit)
        rospy.loginfo('Waiting for %.1f/%.1f seconds', timer, wait_time)
        timer += sleep_unit
    rospy.loginfo("Done waiting! Do a thing!")
    rospy.sleep(0.1)
    return

if __name__ == '__main__':
    rospy.init_node('Quadcopter')
    rospy.sleep(1.0)

    quad = Quadcopter()
    lat = 42.292829
    lon = -71.263084
    quad.launch(lat, lon)
    annotated_timer(30)
    quad.land(lat, lon)

    rospy.spin()
