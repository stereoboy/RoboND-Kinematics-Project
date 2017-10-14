## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/DH.png
[image2]: ./misc_images/theta1.png
[image3]: ./misc_images/theta2_3.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

I extracted the right DH value from the kr210.urdf.xacro following diagram. Rotations with alpha along x axis are a bit confusing points for me.

![alt text][image1]

Here is my final DH Table. Important point is merging 'd' or 'a' value for optimzing calculation.
I have not done the 3 addition operation to show the intermediate process.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | (0.33 + 0.42) | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3 
3->4 |  -pi/2 | -0.054 | (0.96 + 0.54) | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | (0.193 + 0.11) | 0.0



#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Individual transformation matrices are as follows.
```
T0_1 = Matrix([
[cos(q1), -sin(q1), 0,  0.0],
[sin(q1),  cos(q1), 0,    0],
[      0,        0, 1, 0.75],
[      0,        0, 0,    1]])

T1_2 = Matrix([
[sin(q2),  cos(q2), 0, 0.35],
[      0,        0, 1,  0.0],
[cos(q2), -sin(q2), 0,    0],
[      0,        0, 0,    1]])

T2_3 = Matrix([
[cos(q3), -sin(q3), 0, 1.25],
[sin(q3),  cos(q3), 0,    0],
[      0,        0, 1,  0.0],
[      0,        0, 0,    1]])

T3_4 = Matrix([
[ cos(q4), -sin(q4), 0, -0.054],
[       0,        0, 1,    1.5],
[-sin(q4), -cos(q4), 0,      0],
[       0,        0, 0,      1]]

T4_5 = Matrix([
[cos(q5), -sin(q5),  0, 0.0],
[      0,        0, -1,   0],
[sin(q5),  cos(q5),  0,   0],
[      0,        0,  0,   1]])

T5_6 =  Matrix([
[ cos(q6), -sin(q6), 0, 0.0],
[       0,        0, 1, 0.0],
[-sin(q6), -cos(q6), 0,   0],
[       0,        0, 0,   1]])

T6_G = Matrix([
[1, 0, 0,   0.0],
[0, 1, 0,     0],
[0, 0, 1, 0.303],
[0, 0, 0,     1]])
```

Transformation from base link to gripper link is as follows: 

```
Matrix([
[(((sin(q2)*cos(q1)*cos(q3) + sin(q3)*cos(q1)*cos(q2))*cos(q4) + sin(q1)*sin(q4))*cos(q5) + (-sin(q2)*sin(q3)*cos(q1) + cos(q1)*cos(q2)*cos(q3))*sin(q5))*cos(q6) - ((sin(q2)*cos(q1)*cos(q3) + sin(q3)*cos(q1)*cos(q2))*sin(q4) - sin(q1)*cos(q4))*sin(q6), -(((sin(q2)*cos(q1)*cos(q3) + sin(q3)*cos(q1)*cos(q2))*cos(q4) + sin(q1)*sin(q4))*cos(q5) + (-sin(q2)*sin(q3)*cos(q1) + cos(q1)*cos(q2)*cos(q3))*sin(q5))*sin(q6) - ((sin(q2)*cos(q1)*cos(q3) + sin(q3)*cos(q1)*cos(q2))*sin(q4) - sin(q1)*cos(q4))*cos(q6), -((sin(q2)*cos(q1)*cos(q3) + sin(q3)*cos(q1)*cos(q2))*cos(q4) + sin(q1)*sin(q4))*sin(q5) + (-sin(q2)*sin(q3)*cos(q1) + cos(q1)*cos(q2)*cos(q3))*cos(q5), -0.303*((sin(q2)*cos(q1)*cos(q3) + sin(q3)*cos(q1)*cos(q2))*cos(q4) + sin(q1)*sin(q4))*sin(q5) + 0.303*(-sin(q2)*sin(q3)*cos(q1) + cos(q1)*cos(q2)*cos(q3))*cos(q5) - 1.5*sin(q2)*sin(q3)*cos(q1) - 0.054*sin(q2)*cos(q1)*cos(q3) + 1.25*sin(q2)*cos(q1) - 0.054*sin(q3)*cos(q1)*cos(q2) + 1.5*cos(q1)*cos(q2)*cos(q3) + 0.35*cos(q1)],
[(((sin(q1)*sin(q2)*cos(q3) + sin(q1)*sin(q3)*cos(q2))*cos(q4) - sin(q4)*cos(q1))*cos(q5) + (-sin(q1)*sin(q2)*sin(q3) + sin(q1)*cos(q2)*cos(q3))*sin(q5))*cos(q6) - ((sin(q1)*sin(q2)*cos(q3) + sin(q1)*sin(q3)*cos(q2))*sin(q4) + cos(q1)*cos(q4))*sin(q6), -(((sin(q1)*sin(q2)*cos(q3) + sin(q1)*sin(q3)*cos(q2))*cos(q4) - sin(q4)*cos(q1))*cos(q5) + (-sin(q1)*sin(q2)*sin(q3) + sin(q1)*cos(q2)*cos(q3))*sin(q5))*sin(q6) - ((sin(q1)*sin(q2)*cos(q3) + sin(q1)*sin(q3)*cos(q2))*sin(q4) + cos(q1)*cos(q4))*cos(q6), -((sin(q1)*sin(q2)*cos(q3) + sin(q1)*sin(q3)*cos(q2))*cos(q4) - sin(q4)*cos(q1))*sin(q5) + (-sin(q1)*sin(q2)*sin(q3) + sin(q1)*cos(q2)*cos(q3))*cos(q5), -0.303*((sin(q1)*sin(q2)*cos(q3) + sin(q1)*sin(q3)*cos(q2))*cos(q4) - sin(q4)*cos(q1))*sin(q5) + 0.303*(-sin(q1)*sin(q2)*sin(q3) + sin(q1)*cos(q2)*cos(q3))*cos(q5) - 1.5*sin(q1)*sin(q2)*sin(q3) - 0.054*sin(q1)*sin(q2)*cos(q3) + 1.25*sin(q1)*sin(q2) - 0.054*sin(q1)*sin(q3)*cos(q2) + 1.5*sin(q1)*cos(q2)*cos(q3) + 0.35*sin(q1)],
[                                                                                     -(-sin(q2)*sin(q3) + cos(q2)*cos(q3))*sin(q4)*sin(q6) + ((-sin(q2)*sin(q3) + cos(q2)*cos(q3))*cos(q4)*cos(q5) + (-sin(q2)*cos(q3) - sin(q3)*cos(q2))*sin(q5))*cos(q6),                                                                                       -(-sin(q2)*sin(q3) + cos(q2)*cos(q3))*sin(q4)*cos(q6) - ((-sin(q2)*sin(q3) + cos(q2)*cos(q3))*cos(q4)*cos(q5) + (-sin(q2)*cos(q3) - sin(q3)*cos(q2))*sin(q5))*sin(q6),                                                    -(-sin(q2)*sin(q3) + cos(q2)*cos(q3))*sin(q5)*cos(q4) + (-sin(q2)*cos(q3) - sin(q3)*cos(q2))*cos(q5),                                                                                                    -0.303*(-sin(q2)*sin(q3) + cos(q2)*cos(q3))*sin(q5)*cos(q4) + 0.303*(-sin(q2)*cos(q3) - sin(q3)*cos(q2))*cos(q5) + 0.054*sin(q2)*sin(q3) - 1.5*sin(q2)*cos(q3) - 1.5*sin(q3)*cos(q2) - 0.054*cos(q2)*cos(q3) + 1.25*cos(q2) + 0.75],
[                                                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                           0,                                                                                                                                                       0,                                                                                                                                                                                                                                                                                                                                     1]])
```
#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

After getting wrist center point the problem can be divided into 2 part, Inverse Position and Inverse Orientation.

All the key concepts for calculating theta 1,2,3 using Inverse Position are described in the following 2 diagram. Theta 1 can be calculated directly by provided values. But the intermediate calculations like 'diff2, diff3' in the second diagram are necessary for theta 2, 3. Theta3 can be confusing but I calculated using angle b and angle diff3

![alt text][image2]
![alt text][image3]
```
theta1 = atan2(wy, wx)
theta2 = pi/2. - angleA - atan2(wz - S[d1], sqrt(pow(wx, 2) + pow(wy, 2)) - S[a1])
theta3 = - (angleB + atan2(abs(S[a3]), S[d4]) - pi/2.)
```

Inverse Orientation is simpler. After calculating theta1, 2, 3, the transformation matrix from link 3 to gripper link, T3-G by using T0_3 and final gripper position.
Because inverse of rotation matrix is just its tranpose matrix we can calculate T3_G easily.

I can also calculate symbolic value of transformation matrix from link 3 to gripper link.
```
T3_G = Matrix([
[-sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6), -sin(q4)*cos(q6) - sin(q6)*cos(q4)*cos(q5), -sin(q5)*cos(q4), -0.303*sin(q5)*cos(q4) - 0.054],
[                           sin(q5)*cos(q6),                           -sin(q5)*sin(q6),          cos(q5),            0.303*cos(q5) + 1.5],
[-sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4),  sin(q4)*sin(q6)*cos(q5) - cos(q4)*cos(q6),  sin(q4)*sin(q5),          0.303*sin(q4)*sin(q5)],
[                                         0,                                          0,                0,                              1]])
```
According to the lecture of 'Euler Angles from a Rotation Matrix', I got `q4`, `q5`, `q6` using `atan2`.
```
theta4 = atan2(T3_G[2,2], -T3_G[0,2])
theta5 = atan2(sqrt(pow(T3_G[1,0], 2) + pow(T3_G[1,1],2)), T3_G[1,2])
theta6 = atan2(-T3_G[1,1], T3_G[1,0])
```
### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


I have already explained my implementation in detail in the previous section. Only one different thing from the guide from the lectures is using transpose matrix instead of using inverse operation when calculating transformation matrix from link 3 to gripper link. The correctness of `inv('LU')` is not good enough. The other code line are just same as the lecture's guide. 

In my opinion using DH convention is a little problematic. Since URDF does not use DH description, we need to set up additional DH table and effort to calculate some rotations. Even though DH convention is very convenient when I study robotics tradition, I need to know another representations for the kinematics.

Even though the implementation for the pick and place is working well (it finally completes the mission), robot arm sometimes acts acrobatic wasteful and ridiculous moves when locating the gripper to the target. These looks based on the over simplification of the inverse kinematics problem. I thought that additional frame-by-frame considerations are necessary for the optimized movements.
