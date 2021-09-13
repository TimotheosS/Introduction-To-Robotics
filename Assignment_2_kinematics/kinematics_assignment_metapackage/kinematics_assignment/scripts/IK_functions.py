#! /usr/bin/env python3
import math
import numpy as np

def scara_IK(point):
    x = point[0]
    y = point[1]
    z = point[2]
    q = [0.0, 0.0, 0.0]

    l0 = 0.07
    l1 = 0.30
    l2 = 0.35
    
    #Fill in your IK solution here and return the three joint values in q

    x = x - l0
    enu = x ** 2 + y ** 2 - l1 ** 2 - l2 ** 2
    den = 2 * l1 * l2
    q[1] = math.acos(enu / den)

    enu = (l1 + l2 * math.cos(q[1])) * y - l2 * math.sin(q[1]) * x
    den = x ** 2 + y ** 2
    s1 = enu / den

    enu = (l1 + l2 * math.cos(q[1])) * x + l2 * math.sin(q[1]) * y
    c1 = enu / den

    q[0] = math.atan2(s1,c1)
    
    q[2] = z

    ##################

    return q


def kuka_IK(point, R, joint_positions):
    x = point[0]
    y = point[1]
    z = point[2]
    q = joint_positions #it must contain 7 elements

    L = 0.4
    M = 0.39
    a = math.pi / 2

    dbs = 0.311
    dwf = 0.078
    error = [1,1,1,1,1,1]

    z = np.array(([0],[0],[1]))
    z_trans = np.array(([0],[0],[0],[1]))

    """
    Fill in your IK solution here and return the seven joint values in q
    """

    while np.linalg.norm(error) > 0.0001:

        # Create Modified DH parameters
        DH =([ a, dbs, 0, q[0]],
            [ -a, 0  , 0, q[1]],
            [ -a, L  , 0, q[2]],
            [  a, 0  , 0, q[3]],
            [  a, M  , 0, q[4]],
            [ -a, 0  , 0, q[5]],
            [  0, dwf, 0, q[6]])

        q_new = q
	
        T0_1 = np.array(TF_Mat(DH[0][0],DH[0][1],DH[0][2],DH[0][3]))
        R1 = np.array(T0_1[0:3,0:3])
        z1 = z
        Pi1 = np.dot(T0_1, z_trans)

        T1_2 = TF_Mat(DH[1][0],DH[1][1],DH[1][2],DH[1][3])
        T0_2 = np.dot(np.array(T0_1),np.array(T1_2))
        R2 = np.array(T0_2[0:3,0:3])
        z2 = np.dot(R1, z)
        Pi2 = np.dot(T0_2, z_trans)

        T2_3 = TF_Mat(DH[2][0],DH[2][1],DH[2][2],DH[2][3])
        T0_3 = np.dot(T0_2,np.array(T2_3)) 
        R3 = np.array(T0_3[0:3,0:3])
        z3 = np.dot(R2, z)
        Pi3 = np.dot(T0_3, z_trans)

        T3_4 = TF_Mat(DH[3][0],DH[3][1],DH[3][2],DH[3][3])
        T0_4 = np.dot(T0_3,np.array(T3_4))
        R4 = np.array(T0_4[0:3,0:3])
        z4 = np.dot(R3, z)
        Pi4 = np.dot(T0_4, z_trans)

        T4_5 = TF_Mat(DH[4][0],DH[4][1],DH[4][2],DH[4][3])
        T0_5 = np.dot(T0_4,np.array(T4_5))
        R5 = np.array(T0_5[0:3,0:3])
        z5 = np.dot(R4, z)
        Pi5 = np.dot(T0_5, z_trans)

        T5_6 = TF_Mat(DH[5][0],DH[5][1],DH[5][2],DH[5][3])
        T0_6 = np.dot(T0_5,np.array(T5_6))
        R6 = np.array(T0_6[0:3,0:3])
        z6 = np.dot(R5, z)
        Pi6 = np.dot(T0_6, z_trans)

        T6_7 = TF_Mat(DH[6][0],DH[6][1],DH[6][2],DH[6][3])
        T0_7 = np.dot(T0_6,np.array(T6_7))
        R7 = np.array(T0_7[0:3,0:3])
        z7 = np.dot(R6, z)
        Pi7 = np.dot(T0_7, z_trans)

        P01 = np.transpose(np.array(np.cross(np.transpose(z1),np.transpose(Pi7[0:3] - Pi1[0:3]))))
        Pcol1 = np.concatenate((P01,z1), axis = 0)

        P02 = np.transpose(np.array(np.cross(np.transpose(z2),np.transpose(Pi7[0:3] - Pi2[0:3]))))
        Pcol2 = np.concatenate((P02,z2), axis = 0)

        P03 = np.transpose(np.array(np.cross(np.transpose(z3),np.transpose(Pi7[0:3] - Pi3[0:3]))))
        Pcol3 = np.concatenate((P03,z3), axis = 0)

        P04 = np.transpose(np.array(np.cross(np.transpose(z4),np.transpose(Pi7[0:3] - Pi4[0:3]))))
        Pcol4 = np.concatenate((P04,z4), axis = 0)

        P05 = np.transpose(np.array(np.cross(np.transpose(z5),np.transpose(Pi7[0:3] - Pi5[0:3]))))
        Pcol5 = np.concatenate((P05,z5), axis = 0)

        P06 = np.transpose(np.array(np.cross(np.transpose(z6),np.transpose(Pi7[0:3] - Pi6[0:3]))))
        Pcol6 = np.concatenate((P06,z6), axis = 0)

        P07 = np.transpose(np.array(np.cross(np.transpose(z7),np.transpose(Pi7[0:3] - Pi7[0:3]))))
        Pcol7 = np.concatenate((P07,z7), axis = 0)

        Jacob = np.concatenate((Pcol1,Pcol2,Pcol3,Pcol4,Pcol5,Pcol6,Pcol7), axis = 1)
        Jac_new =  np.linalg.pinv(Jacob)

        error_angle = 0.5*( np.cross(T0_7[[0,1,2],0], np.array([R[0][0], R[1][0], R[2][0]])) + np.cross(T0_7[[0,1,2],1], np.array([R[0][1], R[1][1], R[2][1]])) + np.cross(T0_7[[0,1,2],2], np.array([R[0][2], R[1][2], R[2][2]])) )
        error_position = T0_7[[0,1,2],3] - point
        error = np.array([error_position[0], error_position[1], error_position[2], error_angle[0], error_angle[1], error_angle[2]])

        q_new = q_new - np.dot(Jac_new, error)

        q = q_new
	# End of While Loop

    q = q_new
    return q
    

def TF_Mat(alpha, a, d, q):
        Trans = ([[            math.cos(q), -math.sin(q) * math.cos(alpha),  math.sin(q) * math.sin(alpha), d * math.cos(q)],
                  [            math.sin(q), math.cos(q)*math.cos(alpha)   , -math.cos(q) * math.sin(alpha), d * math.sin(q)],
                  [                 0     , math.sin(alpha)               ,  math.cos(alpha)              , a],
                  [                 0     , 0                             ,           0                   , 1]])
        return Trans
