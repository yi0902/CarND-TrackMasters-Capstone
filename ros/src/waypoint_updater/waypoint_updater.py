#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

from std_msgs.msg import Int32

import numpy as np

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    
    def __init__(self):
        
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        # Subscriber for /traffic_waypoint
        # assuming msg type is Int64
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.current_position = None
        self.base_waypoints = None
        self.waypoints_to_pub = None

        # Add a member variable to store the starting index of final waypoints
        self.starting_index_of_final_wps = None

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        
        # Set current position
        self.current_position = msg.pose.position
        #rospy.loginfo("Current car position: %f, %f, %f", self.current_position.x, self.current_position.y, self.current_position.z)
        
        # Get waypoints that are ahead of car position to waypoints_to_pub list
        self.waypoints_to_pub = []
        for wp in self.base_waypoints:
        	wp_pos = wp.pose.pose.position
        	#rospy.loginfo("Waypoint %d - position: %f, %f, %f", i, wp_pos.x, wp_pos.y, wp_pos.z)
        	if wp_pos.x > self.current_position.x:
        		self.waypoints_to_pub.append(wp)
        		if len(self.waypoints_to_pub) >= LOOKAHEAD_WPS:
        			break
        #rospy.loginfo("Length of waypoints to publish: %d", len(self.waypoints_to_pub))

        # Publish waypoints to final_waypoints topic
        msg = Lane()
        msg.waypoints = self.waypoints_to_pub
        self.final_waypoints_pub.publish(msg)

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        # Store base waypoints to a list as it's only published once
        self.base_waypoints = waypoints.waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        stopline_index = msg
        # If there is a red light in the final waypoints list
        if stopline_index >= self.starting_index_of_final_wps and stopline_index < self.starting_index_of_final_wps + LOOKAHEAD_WPS:
            # Choose a deceleration mode depending on the distance between current position and stopline
            d = self.distance(self.waypoints_to_pub, 0, stopline_index-self.starting_index_of_final_wps)
            rospy.loginfo("Distance till red light: %f", d)
            starting_v = self.get_waypoint_velocity(0)
            rospy.loginfo("Starting velocity: %f", starting_v)
            avg_decel = starting_v**2/(2.*d)
            rospy.loginfo("Estimated average deceleration: %f", avg_decel)
            decel_max = 1
            decel = min(avg_decel*1.5, decel_max)
            # Set target velocity on base_waypoints starting from the stopline
            decel_scale = [.01, .04, .09, .16, .25, .36, .49, .64, .81]
            decel_sequence = decel*decel_scale
            # Set velocity at the stop line at 0.
            self.set_waypoint_velocity(self.base_waypoints, stopline_index, 0.)
            v1 = 0.
            wp1 = stopline_index
            for i, a in enumerate(decel_sequence):
                wp2 = stopline_index - (i+1)
                #v2 = math.sqrt(v1**2 + 2*a*)
            self.base_waypoints
            # Publish final waypoints
            print(stopline_index)
        rospy.loginfo("Adjusted velocities for a red light at index %d", stopline_index)

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

def JMT(start, end, T):
    """
    Calculate jerk minimizing trajectory for start, end, and T
    """
    a0, a1, a2 = start[0], start[1], start[2]/2.0
    c0 = a0 + a1*T + a2*T**2
    c1 = a1 + 2*a2*T
    c2 = 2*a2

    A = np.array([
            [T**3,   T**4,    T**5],
            [3*T**2, 4*T**3,  5*T**4],
            [6*T,    12*T**2, 20*T**3],
        ])
    B = np.array([
            end[0] - c0,
            end[1] - c1,
            end[2] - c2
        ])

    a_3_4_5 = np.linalg.solve(A, B)
    alphas = np.concatenate([np.array([a0, a1, a2]), a_3_4_5])
    return alphas

def get_fn_s(alphas):
    def fn_s(t):
        #a0, a1, a2, a3, a4, a5 = alphas
        ts = np.array([1, t, t**2, t**3, t**4, t**5])
        return np.inner(alphas, ts)
    return fn_s

def get_fn_v(alphas):
    def fn_v(t):
        a0, a1, a2, a3, a4, a5 = alphas
        return a1 + 2*a2*t + 3*a3*t**2 + 4*a4*t**3 + 5*a5*t**4
    return fn_v

def newton_solve(f, df, s, bound, tolerance=0.0001):
    x0 = bound/2.0
    dx = x0
    while abs(dx) > tolerance:
        x1 = x0 - (f(x0)-s)/df(x0)
        dx = x0 - x1
        x0 = x1
    ruturn x0

def sample_points():
    pass

def fn_interp1d():
    pass

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')