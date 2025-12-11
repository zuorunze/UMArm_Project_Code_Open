import numpy as np
from copy import copy
import time
import kinematics_mp
import robot_constants as rc
from scipy.spatial.transform import Rotation


def inverse_kinematics_with_static_ee_main(mp_q_robot_current, mp_q_robot_desired, mp_homo_desired, mp_rigid_body_homo_list, mp_control_flags):
    q_robot_current = mp_q_robot_current
    rigid_body_homo_list = mp_rigid_body_homo_list
    control_flags = mp_control_flags

    prev_dHdqi = np.zeros([12],dtype=float)
    robot_homo_base = rigid_body_homo_list[0][:,:]
    q_robot_desired_current = np.zeros([rc.num_joints], dtype=float)

    for i in range(len(q_robot_desired_current)):
        q_robot_desired_current[i] = q_robot_current[i]

    while True:
        try:
            # rc.print_formatted(mp_homo_desired[0][0:3, 3])


            if control_flags[rc.jacobian_control_realtime_flag_index] == 1:
                robot_homo_base = rigid_body_homo_list[0][:,:]
                ee_tip_SE4 = kinematics_mp.fkine_mk5_with_rod_ee(rc.params, rc.my_end_effector_homo, q_robot_desired_current)

                jacobian_ikine_one_step(mp_homo_desired, robot_homo_base, q_robot_desired_current, ee_tip_SE4, prev_dHdqi, q_robot_desired_current)

                for i in range(len(q_robot_desired_current)):
                    mp_q_robot_desired[i] = q_robot_desired_current[i]
            
            else:
                prev_dHdqi = np.zeros([12],dtype=float)
                time.sleep(0.1)

        except Exception as e:
            print("ee_static: ", e)
    return


def end_effector_orientation_ikine(desired_robot_frame_SE4, ee_base_robot_frame_SE4, q_ee_desired_out):
    try:
        q_out = ee_module_desired_orientation_to_q(ee_base_robot_frame_SE4, desired_robot_frame_SE4)
        
        for i in range(len(q_out)):
            q_ee_desired_out[i] =  q_out[i]
    except Exception as e:

        print("end_effector_orientation_ikine: ", e)
    return
    
def jacobian_ikine_one_step(homo_desired, robot_homo_base, q_robot, ee_base_robot_frame_SE4, prev_dHdqi, q_desired_out):
    try:
        desired_pos = copy(homo_desired[0][0:3, 3])

        spatial_frame_r4_pos = kinematics_mp.to_homo_r4(desired_pos)
        robot_frame_r4_pos = kinematics_mp.spatial_frame_to_robot_frame(spatial_frame_r4_pos, robot_homo_base)
        robot_frame_r3_pos = kinematics_mp.to_r3(robot_frame_r4_pos)        
        q_desired_result, errorvec = kinematics_mp.WLNRT_with_ee_jacobian_inverse_kinematic(rc.params, rc.q_max_global, rc.q_min_global, q_robot , ee_base_robot_frame_SE4, prev_dHdqi, robot_frame_r3_pos)
        
        # add hard stop for joint over limits
        for i in range(len(q_desired_result)):
            if q_desired_result[i] > rc.q_max_global[i] - 0.01:
                q_desired_result[i] = rc.q_max_global[i] - 0.01
            if q_desired_result[i] < rc.q_min_global[i] + 0.01:
                q_desired_result[i] = rc.q_min_global[i] + 0.01

        # assign value to the output q_desired.
        for i in range(rc.num_joints):
            q_desired_out[i] = q_desired_result[i]

    except Exception as e:
        # Code to handle the exception
        print("jacobian_ikine_realtime:", e)
    return

def ee_module_desired_orientation_to_q(base_frame_SE4, desired_SE4):
    try:
        np_base_frame_SE4 = np.array(base_frame_SE4)
        np_desired_SE4 = np.array(desired_SE4)

        rot_base_SO3 = np_base_frame_SE4[0:3, 0:3]
        rot_desired_SO3 = np_desired_SE4[0:3, 0:3]

        rot_final = rot_desired_SO3 @ np.transpose(rot_base_SO3)
        
        z_final = rot_final[0:3, 2]

        q1 = np.pi/2 - np.atan2(z_final[2], z_final[0])
        q2 = - np.pi/2 + np.atan2(np.sqrt(z_final[0]**2 + z_final[2]**2), z_final[1])

        return np.array([q1, q2])
    
    except Exception as e:
        print("ee_module_desired_orientation_to_q: ", e)

    

