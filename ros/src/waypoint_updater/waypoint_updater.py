#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math
import tf

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

        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.current_pose = None
        self.base_waypoints = None
        self.waypoints_to_pub = None # store the index of waypoints to publish
        self.closest_wp_index = 0 # index of the closest waypoint ahead of the car

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        
        # Set current pose
        self.current_pose = msg.pose

        # Get closest waypoint that is ahead of the car
        self.closest_wp_index = self.get_closest_waypoint(self.current_pose)
        #rospy.loginfo('car x=%s, y=%s, closest wp_x=%s, y=%s',
        #    self.current_pose.position.x, self.current_pose.position.y,
        #    self.base_waypoints[self.closest_wp_index].pose.pose.position.x,
        #    self.base_waypoints[self.closest_wp_index].pose.pose.position.y)
                    
        # Get indexes of waypoints that are ahead of car position to waypoints_to_pub list
        self.waypoints_to_pub = []
        for i in range(0, LOOKAHEAD_WPS):
            self.waypoints_to_pub.append(self.closest_wp_index + i)
        
        # Publish waypoints to final_waypoints topic
        msg = Lane()
        msg.waypoints = [self.base_waypoints[index] for index in self.waypoints_to_pub]
        self.final_waypoints_pub.publish(msg)

    def get_closest_waypoint(self, pose):
        closest_wp_index = 0
        closest_wp_dist = 100000
        
        # car's position
        x_car = pose.position.x
        y_car = pose.position.y

        if self.base_waypoints != None:
            for i in range(len(self.base_waypoints)):
                # current waypoint's position
                x_wp = self.base_waypoints[i].pose.pose.position.x
                y_wp = self.base_waypoints[i].pose.pose.position.y
                # compute distance
                dist = math.hypot(x_car - x_wp, y_car - y_wp)
                # compare and set
                if dist < closest_wp_dist:
                    closest_wp_dist = dist
                    closest_wp_index = i

            # get car's current orientation
            yaw = self.yaw_from_quaternion(self.current_pose)
            # get car's heading if going to the closest waypoint
            closest_wp = self.base_waypoints[closest_wp_index]
            heading = math.atan2(closest_wp.pose.pose.position.y - y_car, 
                closest_wp.pose.pose.position.x - x_car) # btw (-pi, pi)
            if heading < 0:
                heading = heading + 2*math.pi; # set to btw (0, 2pi) as yaw is btw (0, 2pi)
            # get difference on heading, if diff is bigger than pi/4, take the next waypoint
            diff_heading = abs(yaw - heading)
            if diff_heading > math.pi/4:
                closest_wp_index = closest_wp_index + 1
        
        return closest_wp_index

    def yaw_from_quaternion(self, pose):
        return tf.transformations.euler_from_quaternion((pose.orientation.x, 
            pose.orientation.y, pose.orientation.z, pose.orientation.w))[2]

    def waypoints_cb(self, msg):
        # TODO: Implement
        # Store base waypoints to a list as it's only published once
        self.base_waypoints = msg.waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

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


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')