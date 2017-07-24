#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Define DH param symbols
            
            alpha0,alpha1,alpha2,alpha3,alpha4,alpha5,alpha6 = symbols('alpha0:7') # these are alpha(i-1)
            a0,a1,a2,a3,a4,a5,a6 = symbols('a0:7') # these are a(i-1)
            d1,d2,d3,d4,d5,d6,d7 = symbols('d1:8') # these are d(i)
            q1,q2,q3,q4,q5,q6,q7 = symbols('q1:8')  # these are seven theta(i) angles in DH table
            

            # Joint angle symbols
            theta1,theta2,theta3,theta4,theta5,theta6 = symbols('theta1:7')
            
            
            # Modified DH params
            s = {alpha0:     0, a0:      0, d1:  0.75,
                 alpha1: -pi/2, a1:   0.35, d2:     0,  q2: q2-pi/2,
                 alpha2:     0, a2:   1.25, d3:     0,
                 alpha3: -pi/2, a3: -0.054, d4:   1.5,
                 alpha4:  pi/2, a4:      0, d5:     0,
                 alpha5: -pi/2, a5:      0, d6:     0,
                 alpha6:     0, a6:      0, d7: 0.303,  q7: 0}


            
            # Define Modified DH Transformation matrix
            T0_1 = Matrix([[            cos(q1),            -sin(q1),     -sin(q1),              a0],
						   [sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
						   [sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
						   [                  0,                   0,            0,               1]])
        	T0_1 = T0_1.subs(s)   


            T1_2 = Matrix([[            cos(q2),            -sin(q2),     -sin(q2),              a1],
						   [sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha0)*d2],
						   [sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha0)*d2],
						   [                  0,                   0,            0,               1]])
			T1_2 = T1_2.subs(s)

			
            T2_3 = Matrix([[            cos(q3),            -sin(q3),     -sin(q3),              a2],
						   [sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
						   [sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
						   [                  0,                   0,            0,               1]])
			T2_3 = T2_3.subs(s)

			
            T3_4 = Matrix([[            cos(q4),            -sin(q4),     -sin(q4),              a3],
						   [sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
						   [sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
						   [                  0,                   0,            0,               1]])
			T3_4 = T3_4.subs(s)

			
            T4_5 = Matrix([[            cos(q5),            -sin(q5),     -sin(q5),              a0],
						   [sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
						   [sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
						   [                  0,                   0,            0,               1]])
			T4_5 = T4_5.subs(s)

			
            T5_6 = Matrix([[            cos(q6),            -sin(q6),     -sin(q6),              a5],
						   [sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
						   [sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
						   [                  0,                   0,            0,               1]])
			T5_6 = T5_6.subs(s)

			
            T6_7 = Matrix([[            cos(q7),            -sin(q7),     -sin(q7),              a6],
						   [sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
						   [sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
						   [                  0,                   0,            0,               1]])
			T6_G = T6_7.subs(s)




            # Create individual transformation matrices

			T0_1 = DHTransMatrix(alpha0, a0, d1, q1)
			T0_1 = T0_1.subs(s)

			T1_2 = DHTransMatrix(alpha1, a1, d2, q2)
			T1_2 = T1_2.subs(s)

			T2_3 = DHTransMatrix(alpha2, a2, d3, q3)
			T2_3 = T2_3.subs(s)

			T3_4 = DHTransMatrix(alpha3, a3, d4, q4)
			T3_4 = T3_4.subs(s)

			T4_5 = DHTransMatrix(alpha4, a4, d5, q5)
			T4_5 = T4_5.subs(s)

			T5_6 = DHTransMatrix(alpha5, a5, d6, q6)
			T5_6 = T5_6.subs(s)

			T6_G = DHTransMatrix(alpha6, a6, d7, q7)
			T6_G = T6_G.subs(s)
			
			T0_6 = T0_1*T1_2*T2_3*T3_4*T4_5*T5_6




            
            # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
     
            # Calculate joint angles using Geometric IK method
			# Calculate wrist center
# define the R_x, R_y, R_z
			def R_x(q):
					Mat_x = ([[1, 0, 0],
							  [0, cos(q), -sin(q)],
							  [0, sin(q), cos(q)]])
					return Mat_x
			
			def R_y(q):
					Mat_y = ([[cos(q), 0, -sin(q)],
							  [0, 1, 0],
							  [sin(q), 0, cos(q)]])
					return Mat_y

			def R_z(q):
					Mat_z = ([[cos(q), -sin(q), 0],
							  [sin(q), cos(q), 0],
							  [0, 0, 1]])
					return Mat_z
			#Construct rotation matrix from roll pitch and yaw and extract the WC position
			wc_x = d7*(cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll))
			wc_y = d7*(sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll))
			wc_z = d7*(cos(pitch)*cos(roll))



            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
