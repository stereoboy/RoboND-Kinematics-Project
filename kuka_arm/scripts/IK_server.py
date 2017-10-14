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
from sympy.matrices import Matrix
import sys
from time import time

## Insert IK code here!
def rot_x(q):
    R_x = Matrix([
            [1, 0,      0,       0],
            [0, cos(q), -sin(q), 0],
            [0, sin(q), cos(q),  0],
            [0, 0,      0,       1]])

    return R_x

def rot_y(q):
    R_y = Matrix([
            [cos(q),  0, sin(q), 0],
            [0,       1, 0,      0],
            [-sin(q), 0, cos(q), 0],
            [0,       0, 0,      1]])

    return R_y

def rot_z(q):
    R_z = Matrix([
            [cos(q), -sin(q), 0, 0],
            [sin(q), cos(q),  0, 0],
            [0,      0,       1, 0],
            [0,      0,       0, 1]
    ])

    return R_z

def make_hmg(r, t):
    M = r.row_join(Matrix([t[0], t[1], t[2]]))
    M = M.col_join(Matrix([[0, 0, 0, 1]]))
    return M

def build_mat(alpha, a, d, q):
    return Matrix( [[            cos(q),           -sin(q),           0,             a],
                    [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                    [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                    [                  0,                  0,           0,             1]])

#
# Matrix
#

# initialize
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

# Create Modified DH parameters
S = {
    alpha0 : 0.00,   a0 : 0.00,   d1 : (0.33 + 0.42),
    alpha1 : -pi/2., a1 : 0.35,   d2 : 0.00,            q2 : q2 - pi/2,
    alpha2 : 0.00,   a2 : 1.25,   d3 : 0.00,
    alpha3 : -pi/2., a3 : -0.054, d4 : (0.96 + 0.54),
    alpha4 : pi/2.,  a4 : 0.00,   d5 : 0.00,
    alpha5 : -pi/2., a5 : 0.00,   d6 : 0.00,
    alpha6 : 0.00,   a6 : 0.00,   d7 : (0.193 + 0.11),  q7 : 0.00,
}

T0_1 = build_mat(alpha0, a0, d1, q1).subs(S)
T1_2 = build_mat(alpha1, a1, d2, q2).subs(S)
T2_3 = build_mat(alpha2, a2, d3, q3).subs(S)
T3_4 = build_mat(alpha3, a3, d4, q4).subs(S)
T4_5 = build_mat(alpha4, a4, d5, q5).subs(S)
T5_6 = build_mat(alpha5, a5, d6, q6).subs(S)
T6_G = build_mat(alpha6, a6, d7, q7).subs(S)
R_corr = rot_z(pi)*rot_y(-pi/2.)

T0_G = T0_1*T1_2*T2_3*T3_4*T4_5*T5_6*T6_G
T0_3 = T0_1*T1_2*T2_3
T0_5 = T0_1*T1_2*T2_3*T3_4*T4_5
T3_G = T3_4*T4_5*T5_6*T6_G
print('T0_1', T0_1)
print('T0_2', T0_1*T1_2)
print('T0_3', T0_1*T1_2*T2_3)
print('T0_3_2', T0_1[0:3, 0:3]*T1_2[0:3, 0:3]*T2_3[0:3, 0:3])
print('T3_G', T3_G)

r = symbols('r')
p = symbols('p')
y = symbols('y')

Rrpy = rot_z(y)*rot_y(p)*rot_x(r)

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
	rospy.loginfo(len(req.poses))
        ### Your FK code here
        # Create symbols
	#
	#   
	# Create Modified DH parameters
	#
	#            
	# Define Modified DH Transformation matrix
	#
	#
	# Create individual transformation matrices
	#
	#
	# Extract rotation matrices from the transformation matrices
	#
	#
        ###

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

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
     
            ### Your IK code here 
	    # Compensate for rotation discrepancy between DH parameters and Gazebo
	    #
	    #
	    # Calculate joint angles using Geometric IK method
	    #
	    #
            ###

            start_time = time()

            ## Calculate WC
            Rrpy_ = (Rrpy*R_corr).evalf(subs={r:roll, p:pitch, y:yaw})
            nx = Rrpy_[0,2]
            ny = Rrpy_[1,2]
            nz = Rrpy_[2,2]

            ## Inverse Position	
            wx = px - (S[d7])*nx
            wy = py - (S[d7])*ny
            wz = pz - (S[d7])*nz
            
            print ("\nA %04.4f seconds" % (time()-start_time))

            sideA = sqrt(pow(S[d4], 2) + pow(S[a3], 2))
            sideB = sqrt(pow(sqrt(pow(wx, 2) + pow(wy, 2)) - S[a1], 2)  + pow(wz - S[d1], 2))
            sideC = S[a2]

            angleA = acos((pow(sideA, 2) - pow(sideB, 2) - pow(sideC, 2))/(-2.*sideB*sideC))
            angleB = acos((pow(sideB, 2) - pow(sideC, 2) - pow(sideA, 2))/(-2.*sideC*sideA))
            angleC = acos((pow(sideC, 2) - pow(sideA, 2) - pow(sideB, 2))/(-2.*sideA*sideB))

            theta1 = atan2(wy, wx)
            theta2 = pi/2. - angleA - atan2(wz - S[d1], sqrt(pow(wx, 2) + pow(wy, 2)) - S[a1])
            theta3 = - (angleB + atan2(abs(S[a3]), S[d4]) - pi/2.)

            T0_3_ = T0_3.evalf(subs={q1:theta1, q2:theta2, q3:theta3})

            print ("\nB %04.4f seconds" % (time()-start_time))

            ## Inverse Orientation
            T3_G_ = (T0_3_.transpose())*Rrpy_ # use transpose() instead of inv('LU') for performance and accuracy
            #print('TEST', ((T0_3_[0:3,0:3]).inv())*T0_3_[0:3,0:3])
            #print('TEST', ((T0_3_[0:3,0:3]).transpose())*T0_3_[0:3,0:3])

            # using only Rotation
            # https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/87c52cd9-09ba-4414-bc30-24ae18277d24/concepts/a124f98b-1ed5-45f5-b8eb-6c40958c1a6b
            theta4 = atan2(T3_G_[2,2], -T3_G_[0,2])
            theta5 = atan2(sqrt(pow(T3_G_[1,0], 2) + pow(T3_G_[1,1],2)), T3_G_[1,2]) 
            theta6 = atan2(-T3_G_[1,1], T3_G_[1,0])
            print ("\nC %04.4f seconds" % (time()-start_time))
            #print('T3_G_2', T3_G.evalf(subs={q4:theta4, q5:theta5, q6:theta6}))

            FK = T0_G.evalf(subs={q1:theta1, q2:theta2, q3:theta3, q4:theta4, q5:theta5, q6:theta6})
            ee = [FK[0,3],FK[1,3],FK[2,3]]
            rospy.loginfo([ee[0] - px, ee[1] - py, ee[2] - pz])

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
