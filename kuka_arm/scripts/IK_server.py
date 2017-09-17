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

def matrix_transform( alpha, a, d, q) :
    transform = Matrix([[            cos(q),             -sin(q),            0,               a],
                        [ sin(q)*cos(alpha),   cos(q)*cos(alpha),  -sin(alpha),   -sin(alpha)*d],
                        [ sin(q)*sin(alpha),   cos(q)*sin(alpha),   cos(alpha),    cos(alpha)*d],
                        [                 0,                   0,            0,               1]])
    return transform

# Create symbols
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
theta0, theta1 = symbols('theta0:2')

# Create Modified DH parameters
s = {alpha0:     0,  a0:       0,  d1:  0.75,    q1:      q1,
     alpha1: -pi/2,  a1:    0.35,  d2:     0,    q2: q2-pi/2, 
     alpha2:     0,  a2:    1.25,  d3:     0,    q3:      q3,
     alpha3: -pi/2,  a3:  -0.054,  d4:   1.5,    q4:      q4,
     alpha4:  pi/2,  a4:       0,  d5:     0,    q5:      q5,
     alpha5: -pi/2,  a5:       0,  d6:     0,    q6:      q6,
     alpha6:     0,  a6:       0,  d7: 0.303,    q7:       0}

# Create individual transformation matrices
T0_1 = matrix_transform(alpha0, a0, d1, q1)
T0_1 = T0_1.subs(s)
T1_2 = matrix_transform(alpha1, a1, d2, q2)
T1_2 = T1_2.subs(s)
T2_3 = matrix_transform(alpha2, a2, d3, q3)
T2_3 = T2_3.subs(s)
T3_4 = matrix_transform(alpha3, a3, d4, q4)
T3_4 = T3_4.subs(s)
T4_5 = matrix_transform(alpha4, a4, d5, q5)
T4_5 = T4_5.subs(s)
T5_6 = matrix_transform(alpha5, a5, d6, q6)
T5_6 = T5_6.subs(s)
T6_G = matrix_transform(alpha6, a6, d7, q7)
T6_G = T6_G.subs(s)

def rotate_x(theta):
    R_x = Matrix([[ 1,        0,                 0],
                  [ 0,   cos(theta),   -sin(theta)],
                  [ 0,   sin(theta),    cos(theta)]])
    return R_x

def rotate_y(theta):
    R_y = Matrix([[  cos(theta),  0,  sin(theta)],
                   [        0,     1,           0],
                   [ -sin(theta),  0,  cos(theta)]])
    return R_y

def rotate_z(theta):
    R_z = Matrix([[ cos(theta), -sin(theta),        0],
                  [ sin(theta),  cos(theta),        0],
                  [          0,           0,        1]])
    return R_z

#define rotations for gripper orientation
G_z = rotate_z(theta0)
G_y = rotate_y(theta1)
G_correction = simplify(G_z * G_y)
R_corr = G_correction.evalf(subs={theta0: pi, theta1: -pi/2})

#full transformation
T0_G = simplify(T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G)

#partial transformations
T0_2 = simplify(T0_1 * T1_2)
T0_3 = simplify(T0_1 * T1_2 * T2_3)
T3_6 = simplify(T3_4 * T4_5* T5_6)

R0_3 = T0_3[0:3,0:3]
R0_3inv = R0_3.inv("LU")
R3_6_var = T3_6[0:3,0:3]
#print(R3_6_var)

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        
        # Create symbols
        #q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        #d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        #a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        #alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
        #theta1, theta2 = symbols('theta1:theta2')

        # Create Modified DH parameters
        #s = {alpha0:     0,  a0:       0,  d1:  0.75,    q1:      q1,
        #     alpha1: -pi/2,  a1:    0.35,  d2:     0,    q2: q2-pi/2, 
        #     alpha2:     0,  a2:    1.25,  d3:     0,    q3:      q3,
        #     alpha3: -pi/2,  a3:  -0.054,  d4:   1.5,    q4:      q4,
        #     alpha4:  pi/2,  a4:       0,  d5:     0,    q5:      q5,
        #     alpha5: -pi/2,  a5:       0,  d6:     0,    q6:      q6,
        #     alpha6:     0,  a6:       0,  d7: 0.303,    q7:       0}

        #define rotations for gripper orientation
        #gripper = {theta1: pi, theta2: -pi/2}

        #helper functions to expedite building DH Transform and rotation matrices
        #def matrix_transform( alpha, a, d, q) :
        #    transform = Matrix([[            cos(q),             -sin(q),            0,               a],
        #                        [ sin(q)*cos(alpha),   cos(q)*cos(alpha),   -sin(alpha,   -sin(alpha)*d],
        #                        [ sin(q)*sin(alpha),   cos(q)*sin(alpha),   cos(alpha),    cos(alpha)*d],
        #                        [                 0,                   0,            0,               1]])
        #    return transform

        #def rotate_x(theta):
        #    R_x = Matrix([[ 0,        0,                 0],
        #                  [ 0,   cos(theta),   -sin(theta)],
        #                  [ 1,   sin(theta),    cos(theta)]])
        #    return R_x

        #def rotate_y(theta):
        #     R_y = Matrix([[  cos(theta),  0,  sin(theta)],
        #                  [        0,     1,           0],
        #                   [ -sin(theta),  0,  cos(theta)]])
        #    return R_y

        #def rotate_z(theta):
        #    R_z = Matrix([[ cos(theta), -sin(theta),        0],
        #                  [ sin(theta),  cos(theta),        0],
        #                  [          0,           0,        1]])
        #    return R_z

        # Create individual transformation matrices
        #T0_1 = matrix_transform(alpha0, a0, d1, q1)
        #T0_1 = T0_1.subs(s)
        #T1_2 = matrix_transform(alpha1, a1, d2, q2)
        #T1_2 = T1_2.subs(s)
        #T2_3 = matrix_transform(alpha2, a2, d3, q3)
        #T2_3 = T2_3.subs(s)
        #T3_4 = matrix_transform(alpha3, a3, d4, q4)
        #T3_4 = T3_4.subs(s)
        #T4_5 = matrix_transform(alpha4, a4, d5, q5)
        #T4_5 = T4_5.subs(s)
        #T5_6 = matrix_transform(alpha5, a5, d6, q6)
        #T5_6 = T5_6.subs(s)
        #T6_G = matrix_transform(alpha6, a6, d7, q7)
        #T6_G = T6_G.subs(s)
            

        #full transformation
        #T0_G = simplify(T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G)

        # Extract rotation matrices from the transformation matrices

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

        # Extract end-effector position and orientation from request
        # px,py,pz = end-effector position
        # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z
            p = Matrix([[px],[py],[pz]])

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
     
            ### Your IK code here 
        # Compensate for rotation discrepancy between DH parameters and Gazebo
            Rrpy = rotate_z(yaw) * rotate_y(pitch) * rotate_x(roll) * R_corr
            wc = p - 0.303*Rrpy[:,2]
        #
        #
        # Calculate joint angles using Geometric IK method
            theta1 = atan2(wc[1],wc[0])
            B = sqrt(pow(sqrt(wc[0]*wc[0] + wc[1]*wc[1])-0.35,2) + pow((wc[2]-0.75),2))
            A = 1.5
            C = 1.25
            angle_a = acos((pow(B,2) + pow(C,2) - pow(A,2))/(2*B*C))
            angle_b = acos((pow(A,2) + pow(C,2) - pow(B,2))/(2*A*C))
            angle_c = acos((pow(A,2) + pow(B,2) - pow(C,2))/(2*A*B))

            theta2 = pi/2 - angle_a - atan2((wc[2]-0.75),(sqrt(wc[0]*wc[0] + wc[1]*wc[1])-0.35))
            theta3 = pi/2 - (angle_b + 0.036)
            R3_6 = R0_3inv.evalf(subs={q1:theta1, q2:theta2, q3:theta3}) * Rrpy
            theta4 = atan2(R3_6[2,2],-R3_6[0,2])
            #theta5_2 = atan2(R3_6[2,2], R3_6[1,2] * sin(theta4))
            theta5 = atan2(sqrt(R3_6[0,2]**2 + R3_6[2,2]**2), R3_6[1,2]) 
            theta6 = atan2(-R3_6[1,1],R3_6[1,0])
        
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
