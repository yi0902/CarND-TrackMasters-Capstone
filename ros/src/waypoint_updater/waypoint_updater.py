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

        # Add a member variable to store index of next waypoints
        self.next_wps_id = None

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
        # Get stopline index stop_id
        stop_id = msg
        # If there is a red light in the final waypoints list
        id_diff = stop_id - self.next_wps_id
        if id_diff >= 0 and id_diff < LOOKAHEAD_WPS:
            # Choose a deceleration mode depending on the distance between current position and stopline
            # Calculate distance till next red light
            total_dist = self.distance(self.base_waypoints, self.next_wps_id, stop_id)
            rospy.loginfo("Distance till red light: %f", total_dist)
            starting_v = self.get_waypoint_velocity(self.base_waypoints[self.next_wps_id])
            rospy.loginfo("Starting velocity: %f", starting_v)

            # Estimate average deceleration
            #avg_decel = starting_v**2/(2.*total_dist)
            #rospy.loginfo("Estimated average deceleration: %f", avg_decel)

            # Estimate how much time it will take to decelerate
            T_est = 2*total_dist / starting_v
            T = T_est * .9
            # Generate a trajectory
            start = [0., starting_v, 0.]
            end = [total_dist, 0., 0.]
            alphas = JMT(start, end, T)
            # Test this is actually a good trajectory
            # Good trajectory            
            # Get distance and velocity formula
            fn_s = get_fn_s(alphas)
            fn_v = get_fn_v(alphas)
            # Set target velocity to each waypoint
            d = 0.
            for wps_id in range(self.next_wps_id+1, stop_id):
                d += self.distance(self.base_waypoints, wps_id-1, wps_id)
                t = newton_solve(fn_s, fn_v, d, T)
                target_v = fn_v(t)
                self.set_waypoint_velocity(self.base_waypoints, wps_id, target_v)

            # Set velocity at the stop line at 0.
            self.set_waypoint_velocity(self.base_waypoints, stop_id, 0.)
            # Set target velocity for waypoints after red light stop line
            # Not sure this needs to be implemented

            # Publish final waypoints
            self.waypoints_to_pub = self.base_waypoints[self.next_wps_id:self.next_wps_id+LOOKAHEAD_WPS]
            msg = Lane()
            msg.waypoints = self.waypoints_to_pub
            self.final_waypoints_pub.publish(msg)
            rospy.loginfo("Adjusted velocities for a red light at index %d", stop_id)

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
    return x0

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')