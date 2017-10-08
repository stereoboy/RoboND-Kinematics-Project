from sympy import *
from time import time
from mpmath import radians
import tf
import sys

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[],
              5:[]}


def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()
    
    ########################################################################################
    ## 

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
    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
    d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
    a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

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
#    print('T0_1', T0_1)
#    print('T0_2', T0_1*T1_2)
#    print('T0_3', T0_1*T1_2*T2_3)
#    print('T0_3_2', T0_1[0:3, 0:3]*T1_2[0:3, 0:3]*T2_3[0:3, 0:3])
#    print('T3_G', T3_G)

    r = symbols('r')
    p = symbols('p')
    y = symbols('y')

    Rrpy = rot_z(y)*rot_y(p)*rot_x(r)

#    print("====================================")
#    print(T0_3)
#    print("====================================")
#    print(T3_G)


    print ("\nBase %04.4f seconds" % (time()-start_time))

    px = position.x
    py = position.y
    pz = position.z

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
        [orientation.x, orientation.y,
            orientation.z, orientation.w])

    ## Calculate WC
    Rrpy_ = (Rrpy*R_corr).evalf(subs={r:roll, p:pitch, y:yaw})
    nx = Rrpy_[0,2]
    ny = Rrpy_[1,2]
    nz = Rrpy_[2,2]

#    print(Rrpy_)
    
    ## Inverse Position	
    wx = px - (S[d7])*nx
    wy = py - (S[d7])*ny
    wz = pz - (S[d7])*nz
    print ("\nA %04.4f seconds" % (time()-start_time))
    
#    print('WC----------')
#    print(wx)
#    print(wy)
#    print(wz)
#    print(test_case[1][0])
#    print(test_case[1][1])
#    print(test_case[1][2])

    sideA = sqrt(pow(S[d4], 2) + pow(S[a3], 2))
    sideB = sqrt(pow(sqrt(pow(wx, 2) + pow(wy, 2)) - S[a1], 2)  + pow(wz - S[d1], 2))
    sideC = S[a2]

    angleA = acos((pow(sideA, 2) - pow(sideB, 2) - pow(sideC, 2))/(-2.*sideB*sideC))
    angleB = acos((pow(sideB, 2) - pow(sideC, 2) - pow(sideA, 2))/(-2.*sideC*sideA))
    angleC = acos((pow(sideC, 2) - pow(sideA, 2) - pow(sideB, 2))/(-2.*sideA*sideB))
    print ("\nB %04.4f seconds" % (time()-start_time))
#    print('----------')
#    print(angleA)
#    print(angleB)
#    print(angleC)
#    print(angleA + angleB + angleC)

    theta1 = atan2(wy, wx)
    theta2 = pi/2. - angleA - atan2(wz - S[d1], sqrt(pow(wx, 2) + pow(wy, 2)) - S[a1])
    theta3 = - (angleB + atan2(abs(S[a3]), S[d4]) - pi/2.)

#    print('theta----------')
#    print(float(theta1))
#    print(float(theta2))
#    print(float(theta3))
#    print(test_case[2][0])
#    print(test_case[2][1])
#    print(test_case[2][2])

    T0_3_ = T0_3.evalf(subs={q1:theta1, q2:theta2, q3:theta3})
    
    ## Inverse Orientation
    T3_G_ = (T0_3_.transpose())*Rrpy_
    #print('TEST', ((T0_3_[0:3,0:3]).inv())*T0_3_[0:3,0:3])
    #print('TEST', ((T0_3_[0:3,0:3]).transpose())*T0_3_[0:3,0:3])
#    print('T3_G', T3_G)
#    print('T3_G_', T3_G_)
#    print((T3_G_[1,3] - 1.5)/0.303) 

    # using only Rotation
    # https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/87c52cd9-09ba-4414-bc30-24ae18277d24/concepts/a124f98b-1ed5-45f5-b8eb-6c40958c1a6b
    theta4 = atan2(T3_G_[2,2], -T3_G_[0,2])
    theta5 = atan2(sqrt(pow(T3_G_[1,0], 2) + pow(T3_G_[1,1],2)), T3_G_[1,2]) 
    theta6 = atan2(-T3_G_[1,1], T3_G_[1,0])
    print ("\nC %04.4f seconds" % (time()-start_time))
#    print('T3_G_2', T3_G.evalf(subs={q4:theta4, q5:theta5, q6:theta6}))
#   
#    print('----------')
#    print(theta4) 
#    print(theta5) 
#    print(theta6)
#    print(test_case[2][3])
#    print(test_case[2][4])
#    print(test_case[2][5])

    ## 
    ########################################################################################
    
    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    ## (OPTIONAL) YOUR CODE HERE!
    FK = T0_G.evalf(subs={q1:theta1, q2:theta2, q3:theta3, q4:theta4, q5:theta5, q6:theta6})

    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = [wx,wy,wz] # <--- Load your calculated WC values in this array
    your_ee = [FK[0,3],FK[1,3],FK[2,3]] # <--- Load your calculated end effector value from your forward kinematics
#    print('-----------------') 
#    print('T0_5_', T0_5.evalf(subs={q1:theta1, q2:theta2, q3:theta3, q4:theta4, q5:theta5, q6:theta6}))
#    print('Rrpy_', Rrpy_)
#    print('FK', FK)
#    print(your_ee)
    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
           \nconfirm whether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)




if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = int(sys.argv[1])

    test_code(test_cases[test_case_number])
