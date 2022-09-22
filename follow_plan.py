#! /usr/bin/env python
# Import the Python library for ROS
import rospy

# Import the Twist message 
from geometry_msgs.msg import Twist

# Import the Odometry message 
from nav_msgs.msg import Odometry 

import tf
import numpy as np

class followPlan():
    def __init__(self, plan = []):
        rospy.init_node('followPlan', anonymous = 'False')
        rospy.on_shutdown(self.shutdown)

        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        # Set a publish velocity rate of in Hz
        rate = rospy.get_param('/rate', 200)
        self.rate = rospy.Rate(rate)

        # this must correspond to what the spawn() service has generated in gazebo
        self.robot_model = rospy.get_param('~robot_name', 'turtlebot3_waffle')
        print 'Robot model:', self.robot_model

        # get the pose from the odometry
        self.odom_sub = rospy.Subscriber('/odom', Odometry,
                                         self.callback_odometry, queue_size=1)

         # get velocities to be used for moving the robot
        self.v = rospy.get_param('~lin_velocity', 0.18)
        self.omega = rospy.get_param('~ang_velocity', 0.5)

        print 'Robot velocity: ({:.2f}, {:.2f})'.format(self.v, self.omega)

        # Creates a var of msg type Twist for velocity
        self.velocity = Twist()

        # initialize all twist velocities to zero
        self.velocity.linear.x =  0.0
        self.velocity.linear.y =  0.0
        self.velocity.linear.z =  0.0
        self.velocity.angular.x =  0.0
        self.velocity.angular.y =  0.0
        self.velocity.angular.z =  0.0

        # pose variables
        self.x = -999
        self.y = -999
        self.yaw = -999

        # wait for an input from the odometry topic to be sure that pose is correctly set when start
        print 'Wait for first odometry data...',
        while ((self.x == -999) or (self.y == -999) or (self.yaw == -999)):
            self.rate.sleep()
        print ' done!'

        print ("Initial Pose: ({:.2f}, {:.2f}) {:.2f}".format(self.x, self.y, self.yaw))

        # this is the plan to actuate, as a list of translations and rotations
        self.plan = plan[:]
        self.current_angle = 0

    def callback_odometry(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = self.quaternion_to_euler(msg)
    
    def quaternion_to_euler(self, msg):
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)
        #print "Roll: %5.2f  Pitch: %5.2f  Yaw: %5.2f" % (roll, pitch, yaw)
        return yaw

    # compute the straight line distance between two points in 2D    
    def euclidean_distance(self, u, v):
        return np.sqrt( (u[0] - v[0])*(u[0] - v[0]) + (u[1] - v[1])*(u[1] - v[1]) )

    # compute the difference between two angles according to atan2() quadrant rules
    def angle_distance(self, a1, a2):
        #atan2(y1, x1) +- atan2(y2, x2) = atan2(y1*x2 +- y2*x1,  x1 * x2 -+ y1*y2)
        x1 = np.cos(a1)
        y1 = np.sin(a1)

        x2 = np.cos(a2)
        y2 = np.sin(a2)

        sum_a1_a2 = np.arctan2(y1*x2 - y2*x1,  x1*x2 + y1*y2)
        return sum_a1_a2

    
    def move_straight_for_distance_using_odometry(self, target_distance):
        self.start_xy = (self.x, self.y)

        self.velocity.linear.x = self.v
        self.velocity.angular.z = 0.0
                
        while self.euclidean_distance(self.start_xy,
                                      (self.x, self.y)) < target_distance:
            self.vel_pub.publish(self.velocity)
            self.rate.sleep()

            
    def rotate_for_angle_using_odometry(self, target_rotation):
        self.start_yaw = self.yaw

        if target_rotation < 0:
            self.velocity.angular.z = -self.omega
        else:
            self.velocity.angular.z = self.omega
            
        while abs(self.angle_distance(self.start_yaw, self.yaw)) < abs(target_rotation):
            self.vel_pub.publish(self.velocity)
            self.rate.sleep()

            
    def execute_plan(self, repeat = 1):

        for r in range(repeat):
            print '------------- Starting iteration {} -------------'.format(r)
            
            for s, step in enumerate(plan):
                if (s == 0):
                    print("No move")
                else:
                    # first part of a plan step consists in moving for a given distance
                    target_distance = self.euclidean_distance(plan[s-1], plan[s])
                    print("Target distance is:" + str(target_distance))
                    self.move_straight_for_distance_using_odometry(target_distance - (target_distance * 0.018)) 

                    # set the linear velocity to zero
                    self.velocity.linear.x = 0.0                    
                    self.vel_pub.publish(self.velocity)

                    print '\t** Completed linear motion of Step {} in Iteration {} **'.format(s, r)

                    
                    # second part of a plan step consists in rotating for a given angle
                    
                    (x_prev, y_prev) = plan[s]
                    (x_next, y_next) = plan[s+1]
                    print("Plan is - x_prev: " + str(x_prev) + " y_prev: " + str(y_prev) + " x_next: " + str(x_next) + " y_next: " + str(y_next))
                    target_rotation = np.radians(np.degrees(np.arctan2(y_next-y_prev, x_next-x_prev) - self.current_angle))
                    print("Target angle is:" + str(np.degrees(np.arctan2(y_next-y_prev, x_next-x_prev) - self.current_angle)))
                    self.rotate_for_angle_using_odometry(target_rotation - (target_rotation * 0.018))
                    self.current_angle += target_rotation

                    self.velocity.angular.z = 0.0                    
                    self.vel_pub.publish(self.velocity)                                        

                    print '\t** Completed rotation motion of Step {} in Iteration {} **'.format(s, r)

            print '------------- Plan execution completed! -------------'


     # Ensure a smooth shutdown of the robot       
    def shutdown(self):
        
        rospy.loginfo("**** Stopping TurtleBot! ****")

        self.velocity.linear.x = 0.0
        self.velocity.angular.z = 0.0
        
        self.vel_pub.publish(self.velocity)

        rospy.sleep(1)

        
if __name__ == '__main__':

    # This is a plan for a square motion
    plan = [(0, 0), (1, 0), (1.6, 0.7), (1.6, 2), (1.3, 2.7), (0.4, 3.2), (1, 4), (2.5, 4)] 
    repeat = 1
    motion_controller = followPlan(plan)
    motion_controller.execute_plan(repeat)
