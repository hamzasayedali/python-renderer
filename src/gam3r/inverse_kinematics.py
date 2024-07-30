from math import pi as PI
from math import cos as cos
from math import sin as sin
import numpy as np
from scipy.linalg import expm

num_joints = 2
L = [5,3]
joint_types = ["R","R"]
joint_axes = [[1,0,0],[1,0,0]]
up = [0,0,1]
theta0 = [PI/2,PI/2]
base_frame = [0,0,0]

# find the joint angles needed to position the end effector at a given point

# i want to try a work backwards approach, we know the point, draw a cloud of points where the joint of the last arm could be placed to reach the desired pos

# at 0 theta, end effector relative to joints is
pos_M = []
theta_M = [1,0,0]


#forward kinematic equation
def get_pos_from_theta(theta):

    # also construct the jacobian in this process

    J = np.zeros([6,num_joints])
    

    # start with the position of end effector with theta = 0
    pos_M = np.array([
        [1,0,0,0],
        [0,1,0,0],
        [0,0,1,0],
        [0,0,0,1]
    ])
    total_L = np.sum(L)
    pos = total_L * np.array(up)

    pos_M[:3,3] = pos

    #print(pos_M)
    

    # now generate the screw matrices for all the arms to apply to the pos_M
    screw_matrices = []
    partial_L_to_joint = 0
    for i in range(num_joints):
        ux = joint_axes[i][0]
        uy = joint_axes[i][1]
        uz = joint_axes[i][2]

        joint_to_base = -np.array(up)

        K = np.array([
            [0,-uz, uy],
            [uz,0,-ux],
            [-uy,ux,0]
        ])

        v_i = np.cross(joint_axes[i],joint_to_base) * partial_L_to_joint

        J[0:3,i] = joint_axes[i]
        J[3:6,i] = v_i
        
        S_i = np.array([
            [0,0,0,0],
            [0,0,0,0],
            [0,0,0,0],
            [0,0,0,0]
        ])

        S_i[0:3,0:3] = K
        S_i[0:3,3] = v_i
        #print(S_i)
        partial_L_to_joint += L[i]

        screw_matrices.append(S_i)

        #print(J)

        J_pinv = np.linalg.pinv(J)


    # now compute the matrix exponential of e^(S0*theta0)*e^(S1*theta1)*M

    matrix_exponential_product = pos_M

    #print(screw_matrices)

    for i in range(num_joints):
        matrix_exp_i = expm(screw_matrices[i]*theta[i])
        #print(matrix_exp_i)
        #print(matrix_exponential_product)
        matrix_exponential_product = np.matmul(matrix_exp_i,matrix_exponential_product)

    #print(matrix_exponential_product)
    return matrix_exponential_product, J_pinv


f_theta, J_pinv = get_pos_from_theta([0,0])
xd = np.array([0,5,4])

theta_initial = np.array([0,0])


error = xd - f_theta[0:3,3] 

print(error)

# adjust the first angle to get position closest to accurate, then do same for second

joint_step = 0.001
theta_guess = [0,0]

best_error = 10000
best_guess = theta_guess.copy()

for i in range(10):
    for i in range(num_joints):

        while True:
            theta_guess_1 = best_guess.copy()
            theta_guess_2 = best_guess.copy()
            theta_guess_1[i] = theta_guess_1[i]+joint_step
            theta_guess_2[i] = theta_guess_2[i]-joint_step

            f_theta_1, J_pinv = get_pos_from_theta(theta_guess_1)
            f_theta_2, J_pinv = get_pos_from_theta(theta_guess_2)

            error_1 = np.linalg.norm(xd - f_theta_1[0:3,3])
            error_2 = np.linalg.norm(xd - f_theta_2[0:3,3])

            if error_1 > best_error and error_2 > best_error:
                print(f"joint {i} best theta is {best_guess[i]}")
                break
                

            if error_1 < best_error:
                best_guess = theta_guess_1
                best_error = error_1

            if error_2 < best_error:
                best_guess = theta_guess_2
                best_error = error_2

            if best_error < 0.01:
                break

    print(best_guess)
    print(best_error)

        
            

        



theta_guess = [PI/2,PI/2]






















def matrix_exponential(S_i, theta):
    return S_i * theta
def get_theta_for_pos(pos):
    pass















