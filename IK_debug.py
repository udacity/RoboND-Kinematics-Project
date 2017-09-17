from sympy import *
from time import time
from mpmath import radians
import tf

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

G_z = rotate_z(theta0)
G_y = rotate_y(theta1)
G_correction = simplify(G_z * G_y)
R_corr = G_correction.evalf(subs={theta0: pi, theta1: -pi/2})

#full transformation
T0_G = simplify(T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G)

T0_2 = simplify(T0_1 * T1_2)
T0_3 = simplify(T0_1 * T1_2 * T2_3)
T3_6 = simplify(T3_4 * T4_5* T5_6)

R0_3 = T0_3[0:3,0:3]
R0_3inv = R0_3.inv("LU")
R3_6_var = T3_6[0:3,0:3]
print(R3_6_var)

#print(T0_3)
#print(R)

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
    #print('start_time is: '+str(start_time))
    ########################################################################################
    ## 

    ## Insert IK code here!
    
    # Extract end-effector position and orientation from request
    # px,py,pz = end-effector position
    # roll, pitch, yaw = end-effector orientation
    px = req.poses[0].position.x
    py = req.poses[0].position.y
    pz = req.poses[0].position.z
    p = Matrix([[px],[py],[pz]])
    #print(str(px)+','+str(py)+','+str(pz))
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
        [req.poses[0].orientation.x, req.poses[0].orientation.y,
            req.poses[0].orientation.z, req.poses[0].orientation.w])
    #print(str(roll)+','+str(pitch)+','+str(yaw))

    Rrpy = rotate_z(yaw) * rotate_y(pitch) * rotate_x(roll) * R_corr
    #print(Rrpy)
    wc = p - 0.303*Rrpy[:,2]
    #print(wc)

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
    #print(R3_6)
    theta4 = atan2(R3_6[2,2],-R3_6[0,2])
    theta5_2 = atan2(sqrt(R3_6[0,2]**2 + R3_6[2,2]**2), R3_6[1,2])
    theta5 = atan2(R3_6[2,2], R3_6[1,2] * sin(theta4))
    theta6 = atan2(-R3_6[1,1],R3_6[1,0])
    #print(theta5)
    #print(theta5_2)
    ## 
    ########################################################################################
    
    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    ## (OPTIONAL) YOUR CODE HERE!

    angle1 = test_case[2][0]
    angle2 = test_case[2][1]
    angle3 = test_case[2][2]
    angle4 = test_case[2][3]
    angle5 = test_case[2][4]
    angle6 = test_case[2][5]

    #print('starting FK evaluation: '+str(time()))
    FK = T0_G.evalf(subs={q1: angle1, q2: angle2, q3: angle3, q4: angle4, q5: angle5, q6: angle6})
    #print('finished FK evaluation: '+str(time()))

    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = wc # <--- Load your calculated WC values in this array
    your_ee = [FK[0,3],FK[1,3],FK[2,3]] # <--- Load your calculated end effector value from your forward kinematics
    ########################################################################################
    #print(your_ee)
    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    #Find WC error
    if not(sum(your_wc)==3):
         wc_x_e = abs(your_wc[0]-test_case[1][0])
         wc_y_e = abs(your_wc[1]-test_case[1][1])
         wc_z_e = abs(your_wc[2]-test_case[1][2])
         wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
         print ("\nWrist error for x position is: %04.8f" % wc_x_e)
         print ("Wrist error for y position is: %04.8f" % wc_y_e)
         print ("Wrist error for z position is: %04.8f" % wc_z_e)
         print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # # Find theta errors
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
    # print (" ")

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
    #test_case_number = 0
    #while test_case_number not in range(1,4):
    #    test_case_number = input("pick a test case (1-3): ")
    #    test_case_number = int(test_case_number)

    test_code(test_cases[1])
    print("============================\n")
    test_code(test_cases[2])
    print("============================\n")
    test_code(test_cases[3])
