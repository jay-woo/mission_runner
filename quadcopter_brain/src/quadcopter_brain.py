#!/usr/bin/env python
# # 10/22/2014
# # Charles O. Goddard

import rospy
import time
#import tf

import roscopter
import roscopter.msg
import roscopter.srv
from std_srvs.srv import *
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu


class QuadcopterBrain(object):
    '''
    High-level quadcopter controller.
    '''
    def __init__(self):
        rospy.Subscriber("filtered_pos",
                         roscopter.msg.FilteredPosition,
                         self.on_position_update)

        self.command_service = rospy.ServiceProxy(
            'command', roscopter.srv.APMCommand
        )
        self.waypoint_service = rospy.ServiceProxy(
            'waypoint', roscopter.srv.SendWaypoint
        )
        self.waypoint_list_service = rospy.ServiceProxy(
            'waypoint_list', roscopter.srv.SendWaypointList
        )
        self.trigger_auto_service = rospy.ServiceProxy(
            'trigger_auto', Empty
        )
        self.adjust_throttle_service = rospy.ServiceProxy(
            'adjust_throttle', Empty
        )
        # self.land_service = rospy.ServiceProxy(
        #     'land', Empty
        # )

    def fly_path(self, waypoint_data):
        waypoints = roscopter.msg.WaypointList()
        waypoints.waypoints = [
            build_waypoint(datum) for datum in waypoint_data
        ]

        # Execute flight plan
        self.command_service(roscopter.srv.APMCommandRequest.CMD_ARM)
        print('Armed')
        self.command_service(roscopter.srv.APMCommandRequest.CMD_LAUNCH)
        print('Launched')
        self.trigger_auto_service()
        self.adjust_throttle_service()
        if len(waypoints.waypoints) == 1:
            self.waypoint_service(waypoints.waypoints[0])
            time.sleep(15)
        else:
            print "Ready to send"
            self.waypoint_list_service(waypoints)
            print "Sent waypoints"
            time.sleep(15)
        print('Landing')
        self.command_service(roscopter.srv.APMCommandRequest.CMD_LAND)

    def on_position_update(self, data):
        '''
        data: GPS + IMU
        '''
        pass


def build_waypoint(data):
    latitude = data['latitude']
    longitude = data['longitude']
    altitude = data.get('altitude', 8)
    hold_time = data.get('hold_time', 15.0)

    waypoint = roscopter.msg.Waypoint()
    waypoint.latitude = gps_to_mavlink(latitude)
    waypoint.longitude = gps_to_mavlink(longitude)
    waypoint.altitude = int(altitude * 1000)
    waypoint.hold_time = int(hold_time * 1000)  # in ms
    waypoint.waypoint_type = roscopter.msg.Waypoint.TYPE_NAV
    return waypoint


def gps_to_mavlink(coordinate):
    '''
    coordinate: decimal degrees
    '''
    return int(coordinate * 1e+7)


if __name__ == '__main__':
    #rospy.init_node("quadcopter_brain")
    carl = QuadcopterBrain()
    carl.fly_path([
        {'latitude': 42.2917443, 'longitude': -71.2626758},
        {'latitude': 42.2915593, 'longitude': -71.2625504}
        # {'latitude': 42.2916597, 'longitude': -71.2622852},
        # {'latitude': 42.2918761, 'longitude': -71.262421}
    ])
