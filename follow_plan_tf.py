#!/usr/bin/env python

import rospy

import geometry_msgs.msg
from geometry_msgs.msg import Twist

from geometry_msgs.msg import TransformStamped, PoseStamped, Point
from geometry_msgs.msg import Quaternion, Vector3
from tf2_geometry_msgs import PointStamped
from tf2_geometry_msgs import do_transform_point, do_transform_pose

import std_msgs

import numpy as np

import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler
#https://docs.ros.org/en/melodic/api/tf/html/python/transformations.html

import sys


class UseCases_TF2():
        
    def __init__(self):
        rospy.init_node('UseCases_TF')

        self.robot_model = rospy.get_param('~robot_name', 'turtlebot3_waffle')   
        print 'Robot model: ', self.robot_model

        if 'turtlebot2' in self.robot_model:
            self.vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=5)
        else:
	        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        # What function to call when ctrl + c is issued
        rospy.on_shutdown(self.shutdown)

        # set execution rate in Hz
        rate = rospy.get_param('/rate', 200)
        self.rate = rospy.Rate(rate)

        # get velocities to be used for moving the robot
        self.v = rospy.get_param('~lin_velocity', 0.22)
        self.omega = rospy.get_param('~ang_velocity', 0.8)

        # Creates a var of msg type Twist for velocity
        self.velocity = Twist()

        # initialize all twist velocity variables to zero
        self.velocity.linear = Vector3(0.0, 0.0, 0.0)
        self.velocity.angular = Vector3(0.0, 0.0, 0.0)

        self.current_angle = 0

        # rate of actions
        self.Rate = rospy.Rate(100)

        # define a tf2 transform buffer and pass it to a listener
        self.tf2_buffer = tf2_ros.Buffer(rospy.Duration(5))
        tf2_ros.TransformListener(self.tf2_buffer)

        # just be sure that some entity is providing time and time is passing
        while rospy.get_rostime() == 0:
            rospy.sleep(0.5)

            
    def periodic_broadcast_start_frame(self, timer):
        self.start_frame.header.stamp = rospy.Time.now()
        self.broadcaster.sendTransform(self.start_frame)

        
    def rotate_for_angle_using_tf2(self, target_angle):
        '''Rotate the robot for target_angle based on the current pose. tf2 is used to keep track 
           of robot's pose changing and rotated angle.
           target_angle must be between -pi, +pi
           target_angle must be passed in RADIANS.
        '''

        # If target_angle is very small, return immediately
        if abs(target_angle) < 0.01:
            return

        # get the transform of the base_footprint to the odom frame at the beginning of the journey
        start_transform = self.tf2_buffer.lookup_transform('odom', 'base_footprint',
                                                           rospy.Time.now(),
                                                           timeout=rospy.Duration(2.0))

        # make a copy of the transform and set it as a new frame, start_frame, with the same parent
        self.start_frame = start_transform
        self.start_frame.child_frame_id = 'start_frame'

        # create a new frame with the transform and publish it in the /tf tree
        # first create a dynamic tf broadcaster
        self.broadcaster = tf2_ros.TransformBroadcaster()
        
        # then create a timer, a function which is called periodically to broadcast the transform
        self.timer = rospy.Timer(rospy.Duration(0.1), self.periodic_broadcast_start_frame)

        # at the beginning of the journey rotated yaw angle is 0
        rotated_yaw_tf2 = 0.0
        
        # set the velocity for advancing at each step
        self.velocity.linear.x = 0.0
        if target_angle > 0:
            self.velocity.angular.z = self.omega
        else:
            self.velocity.angular.z = -self.omega

        # keep sending velocity commands until the target_distance has been traveled
        while(abs(rotated_yaw_tf2) < abs(target_angle) - 0.01):

            # send the rotation velocity kick to the motors
            self.vel_pub.publish(self.velocity)

            # get current transform of base_footprint in start frame
            base_to_start_now = self.tf2_buffer.lookup_transform('start_frame', 'base_footprint',
                                                                 rospy.Time(0),
                                                                 timeout=rospy.Duration(3.0))
            # compute current angular displacement in yaw
            quaternion = base_to_start_now.transform.rotation
            (_r, _p, rotated_yaw_tf2) = euler_from_quaternion( (quaternion.x, quaternion.y, 
                                                                quaternion.z, quaternion.w) )
            #print 'yaw:', rotated_yaw_tf2
            self.rate.sleep()

        # bring v to zero
        self.velocity.angular.z = 0.0
        self.vel_pub.publish(self.velocity)
            
        # shutdown the timer broadcasting the start transform in the tf tree
        self.timer.shutdown()

        
    def move_straight_for_distance_using_tf2(self, target_distance):
        '''Move the robot for target_distance based on the current pose. tf2 is used to keep track 
           of robot's pose changing and traveled distance.
        '''
        # If target_distance is very small, return immediately
        if abs(target_distance) < 0.01:
            return

        # get the transform of the base_footprint to the odom frame at the beginning of the journey
        start_transform = self.tf2_buffer.lookup_transform('odom', 'base_footprint',
                                                           rospy.Time(0),
                                                           timeout=rospy.Duration(5.0))

        # make a copy of the transform and set it as a new frame, start_frame, with the same parent
        self.start_frame = start_transform
        self.start_frame.child_frame_id = 'start_frame'
        #print self.start_frame

        # create a new frame with the transform and publish it in the /tf tree
        # first create a dynamic tf broadcaster
        self.broadcaster = tf2_ros.TransformBroadcaster()
        
        # then create a timer, a function which is called periodically to broadcast the transform
        self.timer = rospy.Timer(rospy.Duration(0.1), self.periodic_broadcast_start_frame)

        # at the beginning of the journey traveled distance is 0
        distance_tf2 = 0.0

        # set the velocity for advancing at each step
        self.velocity.angular.z = 0.0
        if target_distance > 0:
            self.velocity.linear.x = self.v
        else:
            self.velocity.linear.x = -self.v

        # keep sending velocity commands until the target_distance has been traveled
        while(distance_tf2 < abs(target_distance)):

            # send the velocity kick to the motors
            self.vel_pub.publish(self.velocity)
            
            # get current transform of base_footprint in start frame
            base_to_start_now = self.tf2_buffer.lookup_transform('start_frame', 'base_footprint',
                                                                 rospy.Time(0),
                                                                 timeout=rospy.Duration(3.0))

            # compute current displacement in x and y and in terms of traveled distance
            delta_x = base_to_start_now.transform.translation.x
            delta_y = base_to_start_now.transform.translation.y
            distance_tf2 = self.euclidean_distance( (delta_x, delta_y), (0,0))

            #print 'distance: ', np.round(distance_tf2, 3)
            self.rate.sleep()

        # bring v to zero
        self.velocity.linear.x = 0.0
        self.vel_pub.publish(self.velocity)
                    
        # shutdown the timer broadcasting the start transform in the tf tree
        self.timer.shutdown()


    def move_to_pose_using_tf2(self, pose):
        '''get a pose as a tuple (x,y,theta) and move there by first 
           making the in-place rotation to align with the target position (x,y)
           then move to (x,y) in a straight line, and finally, in (x,y) rotate in place to
           align with theta'''

        # first step consists in getting relative distance and angle of (x,y) part of pose
        local_in_reference = self.tf2_buffer.lookup_transform('base_footprint', 'odom',
                                                              rospy.Time(0), rospy.Duration(5))
        # create a PointStamped object to encapsulate the point to reach since
        # do_transform_point requires such an object
        point_stamped = PointStamped()
        point_stamped.point = Point(pose[0], pose[1], 0)
        point_stamped.header.frame_id = 'odom'
        point_stamped.header.stamp = rospy.Time.now()
        point_stamped.header.seq = 0
        #print '\n ***> Point stamped: \n', point_stamped

        # ---> transform the point coordinates into the local (i.e., robot) reference frame
        relative_position_of_point = do_transform_point(point_stamped, local_in_reference)

        # the relative angle of the point w.r.t. robot's orientation (local frame)
        # is arctan2 of point's (y, x)
        delta_y = relative_position_of_point.point.y
        delta_x = relative_position_of_point.point.x
        
        target_rotation = np.arctan2(delta_y, delta_x)
        #print 'rotation yaw', np.round(target_rotation,3)

        # relative distance (the robot is locally in (0,0) 
        target_distance = self.euclidean_distance((0,0), (relative_position_of_point.point.x,
                                                          relative_position_of_point.point.y) )
        #print 'distance', np.round(distance,3)
                
        # make the rotation to move in straight line to (x,y) first
        correction_for_inertia = target_rotation * 0.01
        target_rotation -= correction_for_inertia
        self.rotate_for_angle_using_tf2(target_rotation)

        self.velocity.angular.z = 0.0                    
        self.vel_pub.publish(self.velocity)                                        

        # move to the target (x,y)
        correction_for_inertia = target_distance * 0.01
        target_distance -= correction_for_inertia
        self.move_straight_for_distance_using_tf2(target_distance)

        # Now robot is at (x,y)
        # set the linear velocity to zero
        self.velocity.linear.x = 0.0                    
        self.vel_pub.publish(self.velocity)

        # make the rotation to align with the orientation of target pose
        # first, get the relative pose of the target pose w.r.t. current robot's pose
        pose_stamped = PoseStamped()
        pose_stamped.pose.position = Point(x=pose[0], y=pose[1], z=0)
        pose_stamped.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, np.radians(pose[2])))
        pose_stamped.header.stamp = rospy.Time(0)
        pose_stamped.header.frame_id = 'odom'
                
        relative_pose = self.tf2_buffer.transform(pose_stamped, 'base_footprint')
                
        # compute current angular displacement in yaw
        quaternion = relative_pose.pose.orientation 
        (_r, _p, target_rotation) = euler_from_quaternion( (quaternion.x, quaternion.y, 
                                                            quaternion.z, quaternion.w) )
                
        correction_for_inertia = target_rotation * 0.01
        target_rotation -= correction_for_inertia
        self.rotate_for_angle_using_tf2(target_rotation)
                                    
        self.velocity.angular.z = 0.0                    
        self.vel_pub.publish(self.velocity)                                        
                                                                                                    
        
    def euclidean_distance(self, u, v):
        '''Compute the straight line distance between two points in 2D.'''
        return np.sqrt( (u[0] - v[0])*(u[0] - v[0]) + (u[1] - v[1])*(u[1] - v[1]) )
        
    def keep_track_of_transform_to_given_pose(self):
        '''Get current robot's transform in odom. Then, the robot keeps moving
           more or less randomly but keeps track of the current relative pose 
           w.r.t. the starting one.'''
        # move over a circle 
        pass

    def shutdown(self):
        '''Ensure that on shutdown the robot stops moving.'''
        rospy.loginfo("**** Stopping TurtleBot! ****")
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = 0.0
        self.vel_pub.publish(self.velocity)
        rospy.sleep(1)

        
if __name__ == '__main__':

    try:
        move_with_tf2 = UseCases_TF2()

        plan = [(1, 0), (1.6, 0.7), (1.6, 2), (1.3, 2.7), (0.4, 3.2), (1, 4), (2.5, 4)]
        current_angle = 0
        plan_keep = (0, 0, 0)
        for i in range(len(plan) - 1):
            (x_prev, y_prev) = plan[i]
            (x_next, y_next) = plan[i+1]
            target_rotation = np.degrees(np.arctan2(y_next-y_prev, x_next-x_prev)) - current_angle
            current_angle = current_angle + target_rotation
            plan[i] = (x_prev, y_prev, current_angle)
            if (i == len(plan) - 2):
                plan[i+1] = (x_next, y_next, 0.0)
            

        for i in range(len(plan)):
            print(plan[i])
            move_with_tf2.move_to_pose_using_tf2(plan[i])
        
                
    except:
        msg = '[{}] ERROR: Node terminated'.format(rospy.get_name())
        rospy.loginfo(msg)
        error_type = sys.exc_info()[0]
        error_msg = sys.exc_info()[1]
        rospy.logerr(error_type)
        rospy.logerr(error_msg)
        

    