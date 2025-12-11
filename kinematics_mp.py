import numpy as np
from scipy.spatial.transform import Rotation
from copy import copy
import time

from numpy import sin
from numpy import cos

from numpy.linalg import inv
from numpy import transpose

import exponentials_mk5

import robot_constants as rc

pi = np.pi

RZ45 = np.array([
    [np.cos(pi/4.0), -np.sin(pi/4.0), 0.0],
    [np.sin(pi/4.0),  np.cos(pi/4.0), 0.0],
    [0.0,             0.0,            1.0],
    ], dtype=float)


RZn45 = np.array([
    [np.cos(-pi/4.0), -np.sin(-pi/4.0), 0.0],
    [np.sin(-pi/4.0),  np.cos(-pi/4.0), 0.0],
    [0.0,             0.0,            1.0],
    ], dtype=float)

RZn90 = np.array([
    [np.cos(-pi/2), -np.sin(-pi/2), 0],
    [np.sin(-pi/2),  np.cos(-pi/2), 0],
    [0,             0,            1],
    ], dtype=float)


np.set_printoptions(suppress=True)

def eepos_after_my_ee(ee_before, my_end_effector_homo):
    return  ee_before @ my_end_effector_homo

def spatial_frame_to_robot_frame(homo_spatial, homo_robot_base):
    homo_robot = homo_inverse(homo_robot_base) @ homo_spatial
    return homo_robot

def robot_frame_to_spatial_frame(homo_robot, homo_robot_base):
    homo_spatial = homo_robot_base @ homo_robot
    return homo_spatial

def to_homo_r4(r3):
    r4 = np.ones([4])
    r4[0:3] = r3
    return r4

def to_r3(r4):
    return r4[0:3]

def homo_inverse(homo):
    R = homo[0:3, 0:3]
    R_transpose = np.transpose(R)
    p = homo[0:3, 3]
    homo_inv = np.eye(4)
    homo_inv[0:3, 0:3] = R_transpose
    homo_inv[0:3, 3] = - R_transpose @ p

    return homo_inv

def WLNRT_with_ee_jacobian_inverse_kinematic(params, qmax, qmin, q_robot, ee_base_robot_frame_SE4, prev_dHdqi, ee_desired_pos):
    damp_lambda = 2.2
    delta_x_coef = 2

    q_robot_current = np.array(q_robot)
    W, curr_dHdqi = WLN_W_matrix(prev_dHdqi, qmax, qmin, q_robot_current)

    for i in range(len(prev_dHdqi)):
        prev_dHdqi[i] = curr_dHdqi[i]

    

    ee_current_homo = ee_base_robot_frame_SE4

    delta_x = (ee_desired_pos - ee_current_homo[0:3, 3]) * delta_x_coef

    # next step: add jacobian
    ja = get_analytic_jacobian(params, q_robot_current, ee_current_homo)

    W_inv = np.linalg.inv(W)

    delta_q = W_inv @ np.transpose(ja) @ np.linalg.inv(ja @ W_inv @ np.transpose(ja) + (damp_lambda**2) * np.eye(3)) @ delta_x
    
    q_robot_current += delta_q

    return q_robot_current, delta_x



def WLNRT_jacobian_inverse_kinematic_no_ee_pos_only(params, qmax, qmin, q, prev_dHdqi, ee_desired_pos):
    damp_lambda = 1
    delta_x_coef = 1

    q_curr = np.array(copy(q))

    W, curr_dHdqi = WLN_W_matrix(prev_dHdqi, qmax, qmin, q_curr)
    for i in range(len(prev_dHdqi)):
        prev_dHdqi[i] = curr_dHdqi[i]

    ee_current_homo = fkine_mk5(params, q_curr)

    real_ee_homo = eepos_after_my_ee(ee_current_homo, rc.my_end_effector_homo)

    # compute the vector between desired and current
    delta_x = (ee_desired_pos - real_ee_homo[0:3, 3]) * delta_x_coef
    
    ja = get_analytic_jacobian(params, q_curr, real_ee_homo)

    W_inv = np.linalg.inv(W)

    delta_q = W_inv @ np.transpose(ja) @ np.linalg.inv(ja @ W_inv @ np.transpose(ja) + (damp_lambda**2) * np.eye(3)) @ delta_x
    
    q_curr += delta_q

    return q_curr, delta_x

def WLN_W_matrix(prev_dHdqi, qmax, qmin, q):
    W = np.eye(12)
    curr_dHdqi = abs_dH_dqi(qmax, qmin, q)

    for i in range(12):
        if prev_dHdqi[i] <= curr_dHdqi[i]:
            W[i,i] = 1 + curr_dHdqi[i]

    return W, curr_dHdqi

def abs_dH_dqi(qmax, qmin, q):

    t0, t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11 = q
    t0max, t1max, t2max, t3max, t4max, t5max, t6max, t7max, t8max, t9max, t10max, t11max = qmax
    t0min, t1min, t2min, t3min, t4min, t5min, t6min, t7min, t8min, t9min, t10min, t11min = qmin

    out = np.array([abs((-1/4)*((-1)*t0+t0max)**(-1)*(t0+(-1)*t0min)**(-2)*( \
            t0max+(-1)*t0min)**2+(1/4)*((-1)*t0+t0max)**(-2)*(t0+(-1)* \
            t0min)**(-1)*(t0max+(-1)*t0min)**2),abs((-1/4)*((-1)*t1+t1max) \
            **(-1)*(t1+(-1)*t1min)**(-2)*(t1max+(-1)*t1min)**2+(1/4)*(( \
            -1)*t1+t1max)**(-2)*(t1+(-1)*t1min)**(-1)*(t1max+(-1)*t1min) \
            **2),abs((-1/4)*((-1)*t2+t2max)**(-1)*(t2+(-1)*t2min)**(-2)*( \
            t2max+(-1)*t2min)**2+(1/4)*((-1)*t2+t2max)**(-2)*(t2+(-1)* \
            t2min)**(-1)*(t2max+(-1)*t2min)**2),abs((-1/4)*((-1)*t3+t3max) \
            **(-1)*(t3+(-1)*t3min)**(-2)*(t3max+(-1)*t3min)**2+(1/4)*(( \
            -1)*t3+t3max)**(-2)*(t3+(-1)*t3min)**(-1)*(t3max+(-1)*t3min) \
            **2),abs((-1/4)*((-1)*t4+t4max)**(-1)*(t4+(-1)*t4min)**(-2)*( \
            t4max+(-1)*t4min)**2+(1/4)*((-1)*t4+t4max)**(-2)*(t4+(-1)* \
            t4min)**(-1)*(t4max+(-1)*t4min)**2),abs((-1/4)*((-1)*t5+t5max) \
            **(-1)*(t5+(-1)*t5min)**(-2)*(t5max+(-1)*t5min)**2+(1/4)*(( \
            -1)*t5+t5max)**(-2)*(t5+(-1)*t5min)**(-1)*(t5max+(-1)*t5min) \
            **2),abs((-1/4)*((-1)*t6+t6max)**(-1)*(t6+(-1)*t6min)**(-2)*( \
            t6max+(-1)*t6min)**2+(1/4)*((-1)*t6+t6max)**(-2)*(t6+(-1)* \
            t6min)**(-1)*(t6max+(-1)*t6min)**2),abs((-1/4)*((-1)*t7+t7max) \
            **(-1)*(t7+(-1)*t7min)**(-2)*(t7max+(-1)*t7min)**2+(1/4)*(( \
            -1)*t7+t7max)**(-2)*(t7+(-1)*t7min)**(-1)*(t7max+(-1)*t7min) \
            **2),abs((-1/4)*((-1)*t8+t8max)**(-1)*(t8+(-1)*t8min)**(-2)*( \
            t8max+(-1)*t8min)**2+(1/4)*((-1)*t8+t8max)**(-2)*(t8+(-1)* \
            t8min)**(-1)*(t8max+(-1)*t8min)**2),abs((-1/4)*((-1)*t9+t9max) \
            **(-1)*(t9+(-1)*t9min)**(-2)*(t9max+(-1)*t9min)**2+(1/4)*(( \
            -1)*t9+t9max)**(-2)*(t9+(-1)*t9min)**(-1)*(t9max+(-1)*t9min) \
            **2),abs((-1/4)*((-1)*t10+t10max)**(-1)*(t10+(-1)*t10min)**( \
            -2)*(t10max+(-1)*t10min)**2+(1/4)*((-1)*t10+t10max)**(-2)*( \
            t10+(-1)*t10min)**(-1)*(t10max+(-1)*t10min)**2),abs((-1/4)*(( \
            -1)*t11+t11max)**(-1)*(t11+(-1)*t11min)**(-2)*(t11max+(-1)* \
            t11min)**2+(1/4)*((-1)*t11+t11max)**(-2)*(t11+(-1)*t11min)**( \
            -1)*(t11max+(-1)*t11min)**2)])
    
    return out

def moore_penrose_inverse(ja, damper=0):
    mpi = transpose(ja) @  inv(ja @ transpose(ja) + damper * np.eye(3))
    return mpi

# add the opvar into this
def fkine_mk5_with_rod_ee(params, ee_SE4, q_robot, use_opvar=False):

    robot_ee_base = fkine_mk5(params, q_robot, use_opvar)
    
    ee_final = robot_ee_base @ ee_SE4

    return ee_final

def fkine_mk5_with_ee(params, ee_params, q_with_ee):
    ee1x, ee1y, ee1z, ee2x, ee2y, ee2z, gstx, gsty, gstz = ee_params

    robot_ee_base = fkine_mk5(params, q_with_ee[0:rc.num_joints])

    gst0 = np.eye(4)
    gst0[0:3, 3] = np.array([gstx, gsty, gstz])

    ee_12 = exponentials_mk5.get_ee_e12(np.array([ee1x,ee1y,ee1z]), np.array([ee2x,ee2y,ee2z]), q_with_ee[12:14])

    ee_final = robot_ee_base @ ee_12 @ gst0
    
    return robot_ee_base, ee_final

def get_rigid_body_center_of_mass(params, q):
    try:
        curr_SE4 = np.eye(4)
        
        center_of_mass_list = np.zeros((rc.nlinks * 2 + 1, 4, 4))

        for i in range(rc.nlinks):
            [JA1,JA2,UC1,UC2,AA1,AA2,AO1,AO2,LL,JD] = params[i]
            e1234 = exponentials_mk5.get_e1234(params[i, :], q[0+i*4:4+i*4])
            e12 = exponentials_mk5.get_e12(params[i, :], q[0+i*4:4+i*4])

            gst0 = np.array([
                [1,0,0,0],
                [0,1,0,0],
                [0,0,1,-1 * (UC1+AA1+LL+AA2+UC2)],
                [0,0,0,1]
            ], dtype=float)

            gs_c0_0 = [[1,0,0,0], [0,1,0,0], [0,0,1,-1*0.5*JD],[0,0,0,1]]
            gs_c1_0 = [[1,0,0,0], [0,1,0,0], [0,0,1,-1*(UC1+AA1+0.5*LL)],[0,0,0,1]]

            JD_shift = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,-JD],[0,0,0,1]], dtype=float)
            
            c0_SE4 = curr_SE4 @ gs_c0_0
            c1_SE4 = curr_SE4 @ JD_shift @ e12 @ gs_c1_0

            curr_SE4 = curr_SE4 @ JD_shift @ e1234 @ gst0

            center_of_mass_list[2*i, :, :] = c0_SE4
            center_of_mass_list[2*i+1, :, :] = c1_SE4

        # also put in the end-effector mass (matrix TBD)
        gs_last_0 = curr_SE4
        center_of_mass_list[-1, :, :] = gs_last_0
    except Exception as e:
        print("error: get_rigid_body_center_of_mass: ", e)
    
    return center_of_mass_list

def fkine_mk5(params, q, use_optimization=False):
    if len(q) != 12:
        print("Error in size of q: fkine_mk5 is made specifically for runze's 3-link arm")
        return
    
    out_homo = np.eye(4)
    
    if use_optimization is False:
        for i in range(3):
            [JA1,JA2,UC1,UC2,AA1,AA2,AO1,AO2,LL,JD] = params[i]
            e1234 = exponentials_mk5.get_e1234(params[i, :], q[0+i*4:4+i*4])
            gst0 = np.array([
                [1,0,0,0],
                [0,1,0,0],
                [0,0,1,-1 * (UC1+AA1+LL+AA2+UC2)],
                [0,0,0,1]
            ], dtype=float)

            JD_shift = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,-JD],[0,0,0,1]], dtype=float)

            out_homo = out_homo @ JD_shift @ e1234 @ gst0
    else:
        rotation_param = rc.opvar[21:27]
        
        for i in range(3):
            [JA1,JA2,UC1,UC2,AA1,AA2,AO1,AO2,LL,JD] = params[i]
            JD = rc.opvar[18+i]
            LL = rc.opvar_LL[i]

            theta1 = rotation_param[i*2]
            theta2 = rotation_param[i*2+1]
            RthetaSE4_u1 = np.eye(4)
            RthetaSE4_u1[0:3,0:3] = np.array([
                [np.cos(theta1), -np.sin(theta1), 0.0],
                [np.sin(theta1),  np.cos(theta1), 0.0],
                [0.0,             0.0,            1.0],
                ], dtype=float)
            RthetaSE4_u2 = np.eye(4)
            RthetaSE4_u2[0:3,0:3] = np.array([
                [np.cos(theta2), -np.sin(theta2), 0.0],
                [np.sin(theta2),  np.cos(theta2), 0.0],
                [0.0,             0.0,            1.0],
                ], dtype=float)
            
            e1234 = exponentials_mk5.get_e1234(params[i, :], q[0+i*4:4+i*4])
            gst0 = np.array([
                [1,0,0,0],
                [0,1,0,0],
                [0,0,1,-1 * (UC1+AA1+LL+AA2+UC2)],
                [0,0,0,1]
            ], dtype=float)

            JD_shift = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,-JD],[0,0,0,1]], dtype=float)

            out_homo = out_homo @ JD_shift @ e1234 @ gst0

    return out_homo

def fkine_ee_only(ee_params, q_ee):
    try:
        ee1x, ee1y, ee1z, ee2x, ee2y, ee2z, gstx, gsty, gstz = ee_params

        gst0 = np.eye(4)
        gst0[0:3, 3] = np.array([gstx, gsty, gstz])

        ee_12 = exponentials_mk5.get_ee_e12(np.array([ee1x,ee1y,ee1z]), np.array([ee2x,ee2y,ee2z]), q_ee)

        ee_base_to_tip = ee_12 @ gst0

    except Exception as e:
        print("fkine_ee_only: ", e)
    
    return ee_base_to_tip

def get_analytic_jacobian(params, q, ee_homo):
    js = get_spatial_jacobian(params, q)
    [xx,yy,zz] = ee_homo[0:3,3]

    js_to_ja = np.array([\
        [1,0,0,0,zz,-yy],\
        [0,1,0,-zz,0,xx],\
        [0,0,1,yy,-xx,0] \
    ])
    ja = js_to_ja @ js

    return ja

def get_spatial_jacobian(params, q):
    
    if len(q) != 12:
        print("Error in size of q: get_spatial_jacobian is made specifically for runze's 3-link arm")
        return
    
    [param_l0, param_l1, param_l2] = params
    nlinks = 3
    jacobian = np.zeros((6, nlinks*4))

    # now we calculate jacobian first 4 index
    # this is calculated using the first link, theta0-theta3
    
    g = np.eye(4) # initialize the Adjoint of product of exponentials as identity

    distance0 = 0
    [JA1,JA2,UC1,UC2,AA1,AA2,AO1,AO2,LL,JD] = param_l0
    distance1 = UC1 + AA1 + LL + AA2 + UC2 # lenghth of link0
    [JA1,JA2,UC1,UC2,AA1,AA2,AO1,AO2,LL,JD] = param_l1
    distance1 += JD # length between Link0 and Link1
    distance2 = distance1 + UC1 + AA1 + LL + AA2 + UC2 # length of link1
    [JA1,JA2,UC1,UC2,AA1,AA2,AO1,AO2,LL,JD] = param_l2
    distance2 += JD # length between link1 and link2

    # xi here is the twist, twistexp is the twist exponentials with twist and theta.
    xi_link0, twistexp_link0 = get_xi_and_twist_exp(param_l0, distance0,  q[0:4])
    xi_link1, twistexp_link1 = get_xi_and_twist_exp(param_l1, distance1,  q[4:8])
    xi_link2, twistexp_link2 = get_xi_and_twist_exp(param_l2, distance2,  q[8:12])

    xi_link012 = np.concatenate((xi_link0, xi_link1, xi_link2), axis=0)
    twistexp_link012 = np.concatenate((twistexp_link0, twistexp_link1, twistexp_link2), axis=0)

    # First column of jacobian is simply the twist of joint 0
    jacobian[:, 0] = xi_link0[0, :]
    # Initialize the g matrix with the first twist exponential e0
    g = g @ twistexp_link0[0, :, :]
    # now, find twist_prime for joint 1 to 11
    for i in range(1,12):
        # get the rigid adjoint of g, g = e0 @ e1 @ ... @ ei-1. product of exponentials
        rigid_adjoint_of_g = rigid_adjoint_of_homogeneous_matrix(g)
        # get xi_prime. which is Ad(g) @ xi
        xi_prime = rigid_adjoint_of_g @ xi_link012[i]
        # xi_prime is the ith column of the jacobian matrix
        jacobian[:, i] = xi_prime
        # update g. now g should be appended with twist exponential of the ith term.
        g = g @ twistexp_link012[i]

    return jacobian

def rigid_adjoint_of_homogeneous_matrix(g):
    gr = g[0:3, 0:3] # the R component. 3x3
    gp = g[0:3, 3]   # the p component. 3x1
    # gp_raised is the so(3) matrix representation of crossproduct of vector p
    gp_raised = np.array([[0, -gp[2], gp[1]],[gp[2], 0, -gp[0]],[-gp[1], gp[0], 0]])
    # assembling the rigid adjoint matrix. See MECHENG567 lecture notes for details
    rigid_adjoint_g = np.zeros((6,6))
    rigid_adjoint_g[0:3,0:3] = gr
    rigid_adjoint_g[3:6,3:6] = gr
    rigid_adjoint_g[0:3,3:6] = gp_raised @ gr
    return rigid_adjoint_g

def link_length(param):
    [JA1,JA2,UC1,UC2,AA1,AA2,AO1,AO2,LL,JD] = param
    return JD+UC1+AA1+LL+AA2+UC2

def get_xi_and_twist_exp(param, Distance, theta1234):
    [JA1,JA2,UC1,UC2,AA1,AA2,AO1,AO2,LL,JD] = param
    [theta1, theta2, theta3, theta4] = theta1234

    xi1 = np.array([0,(-1)*Distance+(-1)*UC1,0,1,0,0])
    xi2 = np.array([Distance+UC1,0,0,0,1,0])
    xi3 = np.array([2**(-1/2)*(AA1+AA2+Distance+LL+UC1),(-1)*2**(-1/2)*(AA1+AA2+ \
        Distance+LL+UC1),0,2**(-1/2),2**(-1/2),0])
    xi4 = np.array([2**(-1/2)*(AA1+AA2+Distance+LL+UC1),2**(-1/2)*(AA1+AA2+ \
        Distance+LL+UC1),0,(-1)*2**(-1/2),2**(-1/2),0])
    
    twistexp1 = np.array([[1,0,0,0],[0,cos(theta1),(-1)*sin(theta1),(-1)*(Distance+UC1)* \
        sin(theta1)],[0,sin(theta1),cos(theta1),(Distance+UC1)*((-1)+cos( \
        theta1))],[0,0,0,1]])
    
    twistexp2 = np.array([[cos(theta2),0,sin(theta2),(Distance+UC1)*sin(theta2)],[0,1,0,0],[( \
        -1)*sin(theta2),0,cos(theta2),(Distance+UC1)*((-1)+cos(theta2))],[ \
        0,0,0,1]])
    
    twistexp3 = np.array([[cos((1/2)*theta3)**2,sin((1/2)*theta3)**2,2**(-1/2)*sin( \
        theta3),2**(-1/2)*(AA1+AA2+Distance+LL+UC1)*sin(theta3)],[sin(( \
        1/2)*theta3)**2,cos((1/2)*theta3)**2,(-1)*2**(-1/2)*sin( \
        theta3),(-1)*2**(-1/2)*(AA1+AA2+Distance+LL+UC1)*sin(theta3)],[( \
        -1)*2**(-1/2)*sin(theta3),2**(-1/2)*sin(theta3),cos(theta3),( \
        AA1+AA2+Distance+LL+UC1)*((-1)+cos(theta3))],[0,0,0,1]])
    
    twistexp4 = np.array([[cos((1/2)*theta4)**2,(1/2)*((-1)+cos(theta4)),2**(-1/2)*sin( \
        theta4),2**(-1/2)*(AA1+AA2+Distance+LL+UC1)*sin(theta4)],[(1/2)*( \
        (-1)+cos(theta4)),cos((1/2)*theta4)**2,2**(-1/2)*sin(theta4), \
        2**(-1/2)*(AA1+AA2+Distance+LL+UC1)*sin(theta4)],[(-1)*2**(-1/2) \
        *sin(theta4),(-1)*2**(-1/2)*sin(theta4),cos(theta4),(AA1+AA2+ \
        Distance+LL+UC1)*((-1)+cos(theta4))],[0,0,0,1]])
        
    return np.array([xi1, xi2, xi3, xi4]), np.array([twistexp1, twistexp2, twistexp3, twistexp4])

def mocap_to_config_main(mp_rigid_body_homo_list, mp_q):

    q_out = get_configuration_from_mocap_mk8(mp_rigid_body_homo_list)
    for i in range(rc.num_joints):
        mp_q[i] = (float)(q_out[i])
    return

def get_configuration_from_mocap_mk7(mp_rigid_body_homo_list):
    try:
        pos_current = np.zeros((rc.num_rigid_bodies,3))

        for i in range(8):
            pos_current[i, 0:3] = copy(mp_rigid_body_homo_list[i][0:3,3])

        # z-axis in robot frame
        v_base = np.array([0,0,1])

        rot_base = copy(mp_rigid_body_homo_list[0][0:3,0:3])

        # the v_1 is the vector in spatial frame of the carbon fiber link of the first segment
        v_1 = pos_current[1,:] - pos_current[2,:]

        # if np.linalg.norm(v_1) < 1e-10:
        #     return
        
        # center of segement 1 ujoint 1

        ujoint_offset_matrix =  np.array([ 0.09469723,  0.0003719 , -0.00228688, -0.00047398,  0.04424405,  0.00010539,
  0.00367687,  0.00670824,  0.05418658, -0.04716496, -0.00490098, -0.00009136,
  0.00974918, -0.06272838,  0.00030576,  0.00491243,  0.00064445, -0.05291605,
  0.05546745, -0.00269808, -0.00003968, -0.00147893,  0.04047682,  0.00032019,
  0.00112746, -0.00061262,  0.05366455, -0.05227817,  0.00031681, -0.00092668,
 -0.00667265, -0.07534773, -0.0010758 ,  0.00263442, -0.00257358, -0.05225833,
  0.04902017, -0.00222058,  0.00064996,  0.0055149 ,  0.08310319, -0.00020003,
 -0.00258973, -0.00028038,  0.04914334, -0.06864349, -0.00302015, -0.00211766,
 -0.00965332, -0.0139427 ,  0.00002894,  0.00650485, -0.00318716, -0.05531878,], dtype=float)
        




        ujoint_offset_matrix = ujoint_offset_matrix.reshape(6,3,3)




        


        # first, use the forward kinematics to compute the ujoint centers.
        u_1_1 = pos_current[1,:] + ujoint_offset_matrix[0, :, :] @ (v_1 / np.linalg.norm(v_1))
        # center of segement 1 ujoint 2
        u_1_2 = pos_current[2,:] + ujoint_offset_matrix[1, :, :] @ (v_1 / np.linalg.norm(v_1))

        # the v_2 is the vector in spatial frame of the carbon fiber link of the second segment
        v_2 = pos_current[3,:] - pos_current[4,:]

        # compute ujoint centers for the second segment
        u_2_1 = pos_current[3,:] + ujoint_offset_matrix[2, :, :] @ (v_2 / np.linalg.norm(v_2))
        u_2_2 = pos_current[4,:] + ujoint_offset_matrix[3, :, :] @ (v_2 / np.linalg.norm(v_2))


        # the v_2 is the vector in spatial frame of the carbon fiber link of the second segment
        v_3 = pos_current[5,:] - pos_current[6,:]

        # compute ujoint centers for the second segment
        u_3_1 = pos_current[5,:] + ujoint_offset_matrix[4, :, :] @ (v_3 / np.linalg.norm(v_3))
        u_3_2 = pos_current[6,:] + ujoint_offset_matrix[5, :, :] @ (v_3 / np.linalg.norm(v_3))


        ee_rigid_body_center = pos_current[7,:]

        # print ("============")

        # print(u_1_1)
        # print(pos_current[1,:])
        # print(pos_current[2,:])
        # print(u_1_2)
        
        # print(u_2_1)
        # print(pos_current[3,:])
        # print(pos_current[4,:])
        # print(u_2_2)

        # print(u_3_1)
        # print(pos_current[5,:])
        # print(pos_current[6,:])
        # print(u_3_2)
        # print(ee_rigid_body_center)
        
        # ================================
        # Now that we have the position of the ujoint center, we can think the robot as:
        # ujoint_bracket --- carbon fiber link --- ujoint_bracket --- carbon fiber link --- ujoint_bracket --- carbon fiber link --- ujoint_bracket
        # where '---' represent a single ujoint (which are 2 revolute joints)

        
        # rotation matrix from v_1 to the base (z-axis of the robot base frame)

        RV1 = u_1_1 - u_1_2

        # skip if there's no data
        if np.linalg.norm(RV1) < 1e-10:
            return
        
        # RV1 is in spatial frame, we convert it into robot frame
        RV1 = np.transpose(rot_base) @ RV1 / np.linalg.norm(RV1)
        # R_u1 = Rotation.align_vectors(v_base, RV1)
        # get the euler angle in xyz order, use only the first two(xy), to get the ujoint angle
        # eul_u1 = R_u1[0].as_euler('xyz', degrees=True)

        # alternative method for calculating angles
        u1_theta1 = np.atan2(RV1[2], RV1[1]) - np.pi*0.5
        u1_theta2 = -1.0 * (np.atan2((RV1[2]**2 + RV1[1]**2)**0.5, RV1[0]) - np.pi/2)

        u1_theta1 = np.rad2deg(u1_theta1)
        u1_theta2 = np.rad2deg(u1_theta2)


        RV2 = u_1_2 - u_2_1
        RV2 = np.transpose(rot_base) @ RV2 / np.linalg.norm(RV2)
        # align RV1 to vbase (zaxis)
        R_trans_to_z = Rotation.align_vectors(v_base, RV1)
        # apply the same rotation to RV2 so that the relative position between RV2 and RV1 is preserved
        RV2_transformed = (R_trans_to_z[0].as_matrix()) @ RV2
        # rotate the frame by 45 degree according to our hardware design
        # rotate frame by +45 is rotate vector by -45
        RV2_transformed = RZn45 @ RV2_transformed
        # R_u2 = Rotation.align_vectors(v_base, RV2_transformed)
        # eul_u2 = R_u2[0].as_euler('xyz', degrees=True)


        # alternative method for calculating angles
        u2_theta1 = np.atan2(RV2_transformed[2], RV2_transformed[1]) - np.pi*0.5
        u2_theta2 = -1.0 * (np.atan2((RV2_transformed[2]**2 + RV2_transformed[1]**2)**0.5, RV2_transformed[0]) - np.pi/2)

        u2_theta1 = u2_theta1 * 180/np.pi
        u2_theta2 = u2_theta2 * 180/np.pi
        

        
        RV3 = u_2_1 - u_2_2
        RV3 = np.transpose(rot_base) @ RV3  / np.linalg.norm(RV3)
        R_trans_to_z = Rotation.align_vectors(v_base, RV2)
        RV3_transformed = (R_trans_to_z[0].as_matrix()) @ RV3
        # R_u3 = Rotation.align_vectors(v_base, RV3_transformed)
        # eul_u3 = R_u3[0].as_euler('xyz', degrees=True)


        # alternative method for calculating angles
        u3_theta1 = np.atan2(RV3_transformed[2], RV3_transformed[1]) - np.pi*0.5
        u3_theta2 = -1.0 * (np.atan2((RV3_transformed[2]**2 + RV3_transformed[1]**2)**0.5, RV3_transformed[0]) - np.pi/2)

        u3_theta1 = u3_theta1 * 180/np.pi
        u3_theta2 = u3_theta2 * 180/np.pi


        RV4 = u_2_2 - u_3_1
        RV4 = (np.transpose(rot_base) @ RV4) / np.linalg.norm(RV4)
        R_trans_to_z = Rotation.align_vectors(v_base, RV3)
        RV4_transformed = (R_trans_to_z[0].as_matrix()) @ RV4
        RV4_transformed = RZn45 @ RV4_transformed
        # R_u4 = Rotation.align_vectors(v_base, RV4_transformed)
        # rotate the frame by 45 degree according to our hardware design
        # eul_u4 = R_u4[0].as_euler('xyz', degrees=True)


        # alternative method for calculating angles
        u4_theta1 = np.atan2(RV4_transformed[2], RV4_transformed[1]) - np.pi*0.5
        u4_theta2 = -1.0 * (np.atan2((RV4_transformed[2]**2 + RV4_transformed[1]**2)**0.5, RV4_transformed[0]) - np.pi/2)

        u4_theta1 = u4_theta1 * 180/np.pi
        u4_theta2 = u4_theta2 * 180/np.pi


        RV5 = u_3_1 - u_3_2
        RV5 = np.transpose(rot_base) @ RV5 / np.linalg.norm(RV5)
        R_trans_to_z = Rotation.align_vectors(v_base, RV4)
        RV5_transformed = (R_trans_to_z[0].as_matrix()) @ RV5
        # R_u5 = Rotation.align_vectors(v_base, RV5_transformed)
        # eul_u5 = R_u5[0].as_euler('xyz', degrees=True)

        
        # alternative method for calculating angles
        u5_theta1 = np.atan2(RV5_transformed[2], RV5_transformed[1]) - np.pi*0.5
        u5_theta2 = -1.0 * (np.atan2((RV5_transformed[2]**2 + RV5_transformed[1]**2)**0.5, RV5_transformed[0]) - np.pi/2)

        u5_theta1 = u5_theta1 * 180/np.pi
        u5_theta2 = u5_theta2 * 180/np.pi


        RV6 = u_3_2 - ee_rigid_body_center
        RV6 = np.transpose(rot_base) @ RV6 / np.linalg.norm(RV6)
        R_trans_to_z = Rotation.align_vectors(v_base, RV5)
        RV6_transformed = (R_trans_to_z[0].as_matrix()) @ RV6
        RV6_transformed = RZn45 @ RV6_transformed
        # R_u6 = Rotation.align_vectors(v_base, RV6_transformed)
        # rotate the frame by 45 degree according to our hardware design
        # eul_u6 = R_u6[0].as_euler('xyz', degrees=True)
        
        # alternative method for calculating angles
        u6_theta1 = np.atan2(RV6_transformed[2], RV6_transformed[1]) - np.pi*0.5
        u6_theta2 = -1.0 * (np.atan2((RV6_transformed[2]**2 + RV6_transformed[1]**2)**0.5, RV6_transformed[0]) - np.pi/2)

        u6_theta1 = u6_theta1 * 180/np.pi
        u6_theta2 = u6_theta2 * 180/np.pi



        q = np.array([u1_theta1, u1_theta2, u2_theta1, u2_theta2, u3_theta1, u3_theta2, u4_theta1, u4_theta2, u5_theta1, u5_theta2, u6_theta1, u6_theta2])

        formatted = [f"{x:.2f}" for x in np.radians(q)]
        print(formatted)

        # q = np.array(q)
        
        q = np.radians(q)
    except Exception as e:
        print("get_config_mk7:", e)
    return q

def get_configuration_from_mocap(mp_rigid_body_homo_list):

    pos_current = np.zeros((rc.num_rigid_bodies,3))

    for i in range(6):
        pos_current[i, 0:3] = copy(mp_rigid_body_homo_list[i][0:3,3])

    # getting theta0, theta1

    # v_base = rot_current[0, :, 2]
    v_base = np.array([0,0,1])

    rot_base = copy(mp_rigid_body_homo_list[0][0:3,0:3])
    
    v_1 = pos_current[0,:] - pos_current[1,:]
    v_1 = np.transpose(rot_base) @ v_1

    if np.linalg.norm(v_1) < 1e-10:
        return
    
    r_01= Rotation.align_vectors(v_1, v_base)
    eul_01 = r_01[0].as_euler('xyz', degrees=True)

    # print(eul_01[0:2])

    # getting theta2, theta3
    v_2 = pos_current[1,:] - pos_current[2,:]
    v_2 = np.transpose(rot_base) @ v_2

    r_12 = Rotation.align_vectors(v_2, v_1)

    r_12m = r_12[0].as_matrix() @ RZ45

    r_12m = Rotation.from_matrix(r_12m)
    eul_12 = r_12m.as_euler('xyz', degrees=True)

    # print(eul_12[0:2])

    # getting theta4, theta 5
    v_3 = pos_current[2,:] - pos_current[3,:]
    v_3 = np.transpose(rot_base) @ v_3
    r_23 = Rotation.align_vectors(v_3, v_2)
    r_23m = r_23[0]
    eul_23 = r_23m.as_euler('xyz', degrees=True)

    # print(eul_23[0:2])

    # getting theta6, theta7
    v_4 = pos_current[3,:] - pos_current[4,:]
    v_4 = np.transpose(rot_base) @ v_4
    r_34 = Rotation.align_vectors(v_4, v_3)
    r_34m = r_34[0].as_matrix() @ RZ45
    r_34m = Rotation.from_matrix(r_34m)
    eul_34 = r_34m.as_euler('xyz', degrees=True)

    # print(eul_34[0:2])

    # getting theta8, theta9
    v_5 = pos_current[4,:] - pos_current[5,:]
    v_5 = np.transpose(rot_base) @ v_5
    r_45 = Rotation.align_vectors(v_5, v_4)
    r_45m = r_45[0]
    eul_45 = r_45m.as_euler('xyz', degrees=True)

    # print(eul_45[0:2])

    # getting theta10, theta 11
    # v_6 = pos_current[5,:] - pos_current[6,:]
    v_6 = mp_rigid_body_homo_list[5][0:3,2]
    
    v_6 = np.transpose(rot_base) @ v_6
    r_56 = Rotation.align_vectors(v_6, v_5)
    r_56m = r_56[0].as_matrix() @ RZ45
    r_56m = Rotation.from_matrix(r_56m)
    eul_56 = r_56m.as_euler('xyz', degrees=True)

    # print(eul_56[0:2])
    q = [eul_01[0], eul_01[1], eul_12[0], eul_12[1], eul_23[0], eul_23[1], eul_34[0], eul_34[1], eul_45[0], eul_45[1], eul_56[0], eul_56[1]]

    # formatted = [f"{x:.2f}" for x in q]
    # print(formatted)

    q = np.array(q)
    
    q = np.radians(q)
    return q




def get_configuration_from_mocap_mk8(mp_rigid_body_homo_list):
    try:
        pos_current = np.zeros((rc.num_rigid_bodies,3))

        for i in range(8):
            pos_current[i, 0:3] = copy(mp_rigid_body_homo_list[i][0:3,3])

        # z-axis in robot frame
        v_base = np.array([0,0,1])

        rot_base = copy(mp_rigid_body_homo_list[0][0:3,0:3])

        # the v_1 is the vector in spatial frame of the carbon fiber link of the first segment
        v_1 = pos_current[1,:] - pos_current[2,:]

        # if np.linalg.norm(v_1) < 1e-10:
        #     return
        
        # center of segement 1 ujoint 1


        # first, use the forward kinematics to compute the ujoint centers.
        u_1_1 = pos_current[0,:]
        # center of segement 1 ujoint 2
        u_1_2 = pos_current[1,:]

        # compute ujoint centers for the second segment
        u_2_1 = pos_current[2,:]
        u_2_2 = pos_current[3,:]

        # the v_2 is the vector in spatial frame of the carbon fiber link of the second segment

        # compute ujoint centers for the second segment
        u_3_1 = pos_current[4,:]
        u_3_2 = pos_current[5,:]

        ee_rigid_body_center = pos_current[6,:]

        RV1 = u_1_1 - u_1_2

        # skip if there's no data
        if np.linalg.norm(RV1) < 1e-10:
            return
        
        # RV1 is in spatial frame, we convert it into robot frame
        RV1 = np.transpose(rot_base) @ RV1 / np.linalg.norm(RV1)
        # R_u1 = Rotation.align_vectors(v_base, RV1)
        # get the euler angle in xyz order, use only the first two(xy), to get the ujoint angle
        # eul_u1 = R_u1[0].as_euler('xyz', degrees=True)

        # alternative method for calculating angles
        u1_theta1 = np.atan2(RV1[2], RV1[1]) - np.pi*0.5
        u1_theta2 = -1.0 * (np.atan2((RV1[2]**2 + RV1[1]**2)**0.5, RV1[0]) - np.pi/2)

        u1_theta1 = np.rad2deg(u1_theta1)
        u1_theta2 = np.rad2deg(u1_theta2)


        RV2 = u_1_2 - u_2_1
        RV2 = np.transpose(rot_base) @ RV2 / np.linalg.norm(RV2)
        # align RV1 to vbase (zaxis)
        R_trans_to_z = Rotation.align_vectors(v_base, RV1)
        # apply the same rotation to RV2 so that the relative position between RV2 and RV1 is preserved
        RV2_transformed = (R_trans_to_z[0].as_matrix()) @ RV2
        # rotate the frame by 45 degree according to our hardware design
        # rotate frame by +45 is rotate vector by -45
        RV2_transformed = RZn45 @ RV2_transformed
        # R_u2 = Rotation.align_vectors(v_base, RV2_transformed)
        # eul_u2 = R_u2[0].as_euler('xyz', degrees=True)


        # alternative method for calculating angles
        u2_theta1 = np.atan2(RV2_transformed[2], RV2_transformed[1]) - np.pi*0.5
        u2_theta2 = -1.0 * (np.atan2((RV2_transformed[2]**2 + RV2_transformed[1]**2)**0.5, RV2_transformed[0]) - np.pi/2)

        u2_theta1 = u2_theta1 * 180/np.pi
        u2_theta2 = u2_theta2 * 180/np.pi
        

        
        RV3 = u_2_1 - u_2_2
        RV3 = np.transpose(rot_base) @ RV3  / np.linalg.norm(RV3)
        R_trans_to_z = Rotation.align_vectors(v_base, RV2)
        RV3_transformed = (R_trans_to_z[0].as_matrix()) @ RV3
        # R_u3 = Rotation.align_vectors(v_base, RV3_transformed)
        # eul_u3 = R_u3[0].as_euler('xyz', degrees=True)


        # alternative method for calculating angles
        u3_theta1 = np.atan2(RV3_transformed[2], RV3_transformed[1]) - np.pi*0.5
        u3_theta2 = -1.0 * (np.atan2((RV3_transformed[2]**2 + RV3_transformed[1]**2)**0.5, RV3_transformed[0]) - np.pi/2)

        u3_theta1 = u3_theta1 * 180/np.pi
        u3_theta2 = u3_theta2 * 180/np.pi


        RV4 = u_2_2 - u_3_1
        RV4 = (np.transpose(rot_base) @ RV4) / np.linalg.norm(RV4)
        R_trans_to_z = Rotation.align_vectors(v_base, RV3)
        RV4_transformed = (R_trans_to_z[0].as_matrix()) @ RV4
        RV4_transformed = RZn45 @ RV4_transformed
        # R_u4 = Rotation.align_vectors(v_base, RV4_transformed)
        # rotate the frame by 45 degree according to our hardware design
        # eul_u4 = R_u4[0].as_euler('xyz', degrees=True)


        # alternative method for calculating angles
        u4_theta1 = np.atan2(RV4_transformed[2], RV4_transformed[1]) - np.pi*0.5
        u4_theta2 = -1.0 * (np.atan2((RV4_transformed[2]**2 + RV4_transformed[1]**2)**0.5, RV4_transformed[0]) - np.pi/2)

        u4_theta1 = u4_theta1 * 180/np.pi
        u4_theta2 = u4_theta2 * 180/np.pi


        RV5 = u_3_1 - u_3_2
        RV5 = np.transpose(rot_base) @ RV5 / np.linalg.norm(RV5)
        R_trans_to_z = Rotation.align_vectors(v_base, RV4)
        RV5_transformed = (R_trans_to_z[0].as_matrix()) @ RV5
        # R_u5 = Rotation.align_vectors(v_base, RV5_transformed)
        # eul_u5 = R_u5[0].as_euler('xyz', degrees=True)

        
        # alternative method for calculating angles
        u5_theta1 = np.atan2(RV5_transformed[2], RV5_transformed[1]) - np.pi*0.5
        u5_theta2 = -1.0 * (np.atan2((RV5_transformed[2]**2 + RV5_transformed[1]**2)**0.5, RV5_transformed[0]) - np.pi/2)

        u5_theta1 = u5_theta1 * 180/np.pi
        u5_theta2 = u5_theta2 * 180/np.pi

        # with the rod rigid body
        RV6 = u_3_2 - ee_rigid_body_center
        RV6 = np.transpose(rot_base) @ RV6 / np.linalg.norm(RV6)
        R_trans_to_z = Rotation.align_vectors(v_base, RV5)
        RV6_transformed = (R_trans_to_z[0].as_matrix()) @ RV6
        RV6_transformed = RZn45 @ RV6_transformed
        # R_u6 = Rotation.align_vectors(v_base, RV6_transformed)
        # rotate the frame by 45 degree according to our hardware design
        # eul_u6 = R_u6[0].as_euler('xyz', degrees=True)
        
        # alternative method for calculating angles
        u6_theta1 = np.atan2(RV6_transformed[2], RV6_transformed[1]) - np.pi*0.5
        u6_theta2 = -1.0 * (np.atan2((RV6_transformed[2]**2 + RV6_transformed[1]**2)**0.5, RV6_transformed[0]) - np.pi/2)

        u6_theta1 = u6_theta1 * 180/np.pi
        u6_theta2 = u6_theta2 * 180/np.pi



        # ============= using the last joint SE4 only =============
        # # without the rod rigid body, just use the sixth plate
        # RV6 = mp_rigid_body_homo_list[5][0:3,2]
        # RV6 = np.transpose(rot_base) @ RV6 / np.linalg.norm(RV6)
        # R_trans_to_z = Rotation.align_vectors(v_base, RV5)
        # RV6_transformed = (R_trans_to_z[0].as_matrix()) @ RV6
        # RV6_transformed = RZn45 @ RV6_transformed

        # u6_theta1 = np.atan2(RV6_transformed[2], RV6_transformed[1]) - np.pi*0.5
        # u6_theta2 = -1.0 * (np.atan2((RV6_transformed[2]**2 + RV6_transformed[1]**2)**0.5, RV6_transformed[0]) - np.pi/2)

        # u6_theta1 = u6_theta1 * 180/np.pi
        # u6_theta2 = u6_theta2 * 180/np.pi
        # ===========================================================

        q = np.array([u1_theta1, u1_theta2, u2_theta1, u2_theta2, u3_theta1, u3_theta2, u4_theta1, u4_theta2, u5_theta1, u5_theta2, u6_theta1, u6_theta2])

        formatted = [f"{x:.2f}" for x in np.radians(q)]
        # print(formatted)

        # print(np.linalg.norm(pos_current[0,:] - pos_current[1,:]))
        # print(np.linalg.norm(pos_current[1,:] - pos_current[2,:]))
        # print(np.linalg.norm(pos_current[2,:] - pos_current[3,:]))
        # print(np.linalg.norm(pos_current[3,:] - pos_current[4,:]))
        # print(np.linalg.norm(pos_current[4,:] - pos_current[5,:]))
        # print(np.linalg.norm(pos_current[5,:] - pos_current[6,:]))

        # q = np.array(q)
        
        q = np.radians(q)
    except Exception as e:
        print("get_config_mk7:", e)
    return q

def get_configuration_from_mocap(mp_rigid_body_homo_list):

    pos_current = np.zeros((rc.num_rigid_bodies,3))

    for i in range(6):
        pos_current[i, 0:3] = copy(mp_rigid_body_homo_list[i][0:3,3])

    # getting theta0, theta1

    # v_base = rot_current[0, :, 2]
    v_base = np.array([0,0,1])

    rot_base = copy(mp_rigid_body_homo_list[0][0:3,0:3])
    
    v_1 = pos_current[0,:] - pos_current[1,:]
    v_1 = np.transpose(rot_base) @ v_1

    if np.linalg.norm(v_1) < 1e-10:
        return
    
    r_01= Rotation.align_vectors(v_1, v_base)
    eul_01 = r_01[0].as_euler('xyz', degrees=True)

    # print(eul_01[0:2])

    # getting theta2, theta3
    v_2 = pos_current[1,:] - pos_current[2,:]
    v_2 = np.transpose(rot_base) @ v_2

    r_12 = Rotation.align_vectors(v_2, v_1)

    r_12m = r_12[0].as_matrix() @ RZ45

    r_12m = Rotation.from_matrix(r_12m)
    eul_12 = r_12m.as_euler('xyz', degrees=True)

    # print(eul_12[0:2])

    # getting theta4, theta 5
    v_3 = pos_current[2,:] - pos_current[3,:]
    v_3 = np.transpose(rot_base) @ v_3
    r_23 = Rotation.align_vectors(v_3, v_2)
    r_23m = r_23[0]
    eul_23 = r_23m.as_euler('xyz', degrees=True)

    # print(eul_23[0:2])

    # getting theta6, theta7
    v_4 = pos_current[3,:] - pos_current[4,:]
    v_4 = np.transpose(rot_base) @ v_4
    r_34 = Rotation.align_vectors(v_4, v_3)
    r_34m = r_34[0].as_matrix() @ RZ45
    r_34m = Rotation.from_matrix(r_34m)
    eul_34 = r_34m.as_euler('xyz', degrees=True)

    # print(eul_34[0:2])

    # getting theta8, theta9
    v_5 = pos_current[4,:] - pos_current[5,:]
    v_5 = np.transpose(rot_base) @ v_5
    r_45 = Rotation.align_vectors(v_5, v_4)
    r_45m = r_45[0]
    eul_45 = r_45m.as_euler('xyz', degrees=True)

    # print(eul_45[0:2])

    # getting theta10, theta 11
    # v_6 = pos_current[5,:] - pos_current[6,:]
    v_6 = mp_rigid_body_homo_list[5][0:3,2]
    
    v_6 = np.transpose(rot_base) @ v_6
    r_56 = Rotation.align_vectors(v_6, v_5)
    r_56m = r_56[0].as_matrix() @ RZ45
    r_56m = Rotation.from_matrix(r_56m)
    eul_56 = r_56m.as_euler('xyz', degrees=True)

    # print(eul_56[0:2])
    q = [eul_01[0], eul_01[1], eul_12[0], eul_12[1], eul_23[0], eul_23[1], eul_34[0], eul_34[1], eul_45[0], eul_45[1], eul_56[0], eul_56[1]]

    # formatted = [f"{x:.2f}" for x in q]
    # print(formatted)

    q = np.array(q)
    
    q = np.radians(q)
    return q

# a small simulator test for this module
if __name__ == '__main__':
    pass
    