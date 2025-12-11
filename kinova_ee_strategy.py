import numpy as np
from numpy import pi, cos, sin, tan
from scipy.spatial.transform import Rotation as R

import kinematics_mp
import robot_constants as rc

import time

def mp_kinova_ee_move_to_target_spatial(mp_kinova_ee_kinova_frame, mp_kinova_move_spatial, mp_rigid_body_homo_list, mp_control_flags):
    try:
        while True:
            if mp_control_flags[rc.kinova_move_to_target] > 0:
                
                if mp_control_flags[rc.kinova_move_spatial] == 0 and mp_control_flags[rc.kinova_move_delta] == 0:
                    # compute the target SE4
                    # in future this target should be given, and this function should be wrapped into some helper function.
                    tool_frame_mount = mp_rigid_body_homo_list[5]

                    shift_z_from_tool_frame = np.array([
                        [1,0,0,0],
                        [0,1,0,0],
                        [0,0,1,-0.028+rc.my_end_effector_homo[2][3]],
                        [0,0,0,1]
                    ], dtype=float)

                    R_tool_frame_to_sensor_y180 = np.array([
                        [cos(pi), 0, sin(pi),   0],
                        [0,       1,    0,      0],
                        [-sin(pi), 0, cos(pi),  0],
                        [0,0,0,1]
                    ], dtype=float)

                    R_tool_frame_to_sensor_zn90 = np.array([
                        [cos(-0.5*pi), -sin(-0.5*pi), 0, 0],
                        [sin(-0.5*pi), cos(-0.5*pi), 0 , 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]
                    ])

                    sensor_frame =  tool_frame_mount @ shift_z_from_tool_frame @ R_tool_frame_to_sensor_y180 @ R_tool_frame_to_sensor_zn90


                    # kinova_move_to_target == 1: pushing
                    # kinova_move_to_target == 2: pulling
                    kinova_mocap_target_SE4 = np.eye(4)

                    if mp_control_flags[rc.kinova_move_to_target] == 1:
                        R_x90 = np.array([
                            [1, 0, 0, 0],
                            [0, cos(pi/2), -sin(pi/2), 0],
                            [0, sin(pi/2),  cos(pi/2), 0],
                            [0, 0, 0, 1]
                        ])

                        R_z180 = np.array([
                            [cos(pi), -sin(pi), 0, 0],
                            [sin(pi), cos(pi), 0 , 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]
                        ])

                        T_z10cm = np.array([
                            [1,0,0,0],
                            [0,1,0,0],
                            [0,0,1,-0.1],
                            [0,0,0,1]
                        ])

                        T_y2cm = np.array([
                            [1,0,0,0],
                            [0,1,0,-0.02],
                            [0,0,1,0],
                            [0,0,0,1]
                        ])

                        kinova_mocap_target_SE4 = sensor_frame @ R_x90 @ R_z180 @ T_z10cm @ T_y2cm


                    elif mp_control_flags[rc.kinova_move_to_target] == 2:
                        # start with sensor frame
                        # 1. translate +z by about 10cm
                        # 2. translate -y by about 2cm
                        # 3. Rotate by z for +pi
                        # 4. Rotate by new x for + pi/2

                        T_z10cm = np.array([
                            [1,0,0,0],
                            [0,1,0,0],
                            [0,0,1,0.075],
                            [0,0,0,1]
                        ])

                        T_yn2cm = np.array([
                            [1,0,0,0],
                            [0,1,0,-0.02],
                            [0,0,1,0],
                            [0,0,0,1]
                        ])

                        R_z180 = np.array([
                            [cos(pi), -sin(pi), 0, 0],
                            [sin(pi), cos(pi), 0 , 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]
                        ])

                        R_x90 = np.array([
                            [1, 0, 0, 0],
                            [0, cos(pi/2), -sin(pi/2), 0],
                            [0, sin(pi/2),  cos(pi/2), 0],
                            [0, 0, 0, 1]
                        ])

                        kinova_mocap_target_SE4 = sensor_frame @ T_z10cm @ T_yn2cm @ R_z180 @ R_x90
                    

                    elif mp_control_flags[rc.kinova_move_to_target] == 3:
                        # pulling from below
                        # 1. z +12cm
                        # 2. y +1cm
                        # 3. x +pi/2
                        T_z12cm = np.array([
                            [1,0,0,0],
                            [0,1,0,0],
                            [0,0,1,0.09],
                            [0,0,0,1]
                        ])

                        T_y2cm = np.array([
                            [1,0,0,0],
                            [0,1,0,0.02],
                            [0,0,1,0],
                            [0,0,0,1]
                        ])


                        R_x90 = np.array([
                            [1, 0, 0, 0],
                            [0, cos(pi/2), -sin(pi/2), 0],
                            [0, sin(pi/2),  cos(pi/2), 0],
                            [0, 0, 0, 1]
                        ])

                        kinova_mocap_target_SE4 = sensor_frame @ T_z12cm @ T_y2cm @ R_x90

                    elif mp_control_flags[rc.kinova_move_to_target] == 4:
                        # start with sensor frame
                        # 1. translate +z by about 10cm
                        # 2. translate -y by about 2cm
                        # 3. Rotate by z for +pi
                        # 4. Rotate by new x for + pi/2

                        T_z10cm = np.array([
                            [1,0,0,0],
                            [0,1,0,0],
                            [0,0,1,0.11],
                            [0,0,0,1]
                        ])

                        T_yn2cm = np.array([
                            [1,0,0,0],
                            [0,1,0,-0.09],
                            [0,0,1,0],
                            [0,0,0,1]
                        ])

                        R_z180 = np.array([
                            [cos(pi), -sin(pi), 0, 0],
                            [sin(pi), cos(pi), 0 , 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]
                        ])

                        R_x90 = np.array([
                            [1, 0, 0, 0],
                            [0, cos(pi/2), -sin(pi/2), 0],
                            [0, sin(pi/2),  cos(pi/2), 0],
                            [0, 0, 0, 1]
                        ])

                        kinova_mocap_target_SE4 = sensor_frame @ T_z10cm @ T_yn2cm @ R_z180 @ R_x90
                    

                    elif mp_control_flags[rc.kinova_move_to_target] == 5:
                        R_x90 = np.array([
                            [1, 0, 0, 0],
                            [0, cos(pi/2), -sin(pi/2), 0],
                            [0, sin(pi/2),  cos(pi/2), 0],
                            [0, 0, 0, 1]
                        ])

                        R_z180 = np.array([
                            [cos(pi), -sin(pi), 0, 0],
                            [sin(pi), cos(pi), 0 , 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]
                        ])

                        T_z20cm = np.array([
                            [1,0,0,0.03],
                            [0,1,0,-0.005],
                            [0,0,1,-0.25],
                            [0,0,0,1]
                        ])


                        R_ytheta = np.array([
                            [cos(-0.1172665) ,  0 ,  sin(-0.1172665) , 0],
                            [  0    ,  1 ,    0    , 0],
                            [-sin(-0.1172665),  0 ,  cos(-0.1172665) , 0],
                            [0,0,0,1]
                        ])

                        kinova_mocap_target_SE4 = sensor_frame @ R_x90 @ R_z180 @ T_z20cm @ R_ytheta

                    
                    elif mp_control_flags[rc.kinova_move_to_target] == 6:
                        # start with sensor frame
                        # 1. translate +z by about 10cm
                        # 2. translate -y by about 2cm
                        # 3. Rotate by z for +pi
                        # 4. Rotate by new x for + pi/2

                        T_zn20cm = np.array([
                            [1,0,0,0.04],
                            [0,1,0,0],
                            [0,0,1,-0.24],
                            [0,0,0,1]
                        ])

                        R_xn90 = np.array([
                            [1, 0, 0, 0],
                            [0, cos(-pi/2), -sin(-pi/2), 0],
                            [0, sin(-pi/2),  cos(-pi/2), 0],
                            [0, 0, 0, 1]
                        ])

                        R_y90 = np.array([
                            [cos(pi/2) ,  0 ,  sin(pi/2) , 0],
                            [  0    ,  1 ,    0    , 0],
                            [-sin(pi/2),  0 ,  cos(pi/2) , 0],
                            [0,0,0,1]
                        ])


                        R_ytheta = np.array([
                            [cos(-0.1172665) ,  0 ,  sin(-0.1172665) , 0],
                            [  0    ,  1 ,    0    , 0],
                            [-sin(-0.1172665),  0 ,  cos(-0.1172665) , 0],
                            [0,0,0,1]
                        ])

                        kinova_mocap_target_SE4 = sensor_frame @ R_xn90 @ R_y90 @ T_zn20cm @ R_ytheta


                    print("moving to UMARM......")

                    # get current mocap frame
                    kinova_mocap_curr_SE4 = mp_rigid_body_homo_list[8]

                    # get the transformation from current to the set point kinova_mocap_hold_SE4
                    diff_curr_hold_SE4 = kinova_mocap_target_SE4 @ kinematics_mp.homo_inverse(kinova_mocap_curr_SE4)

                    # set this difference as the target
                    mp_kinova_move_spatial[0] = diff_curr_hold_SE4

                    print(diff_curr_hold_SE4)

                    # send the activate signal
                    mp_control_flags[rc.kinova_move_spatial] = 1
                else:

                    # print("mp_kinova_ee_move_to_target_spatial: Kinova arm busy")
                    pass
                

                # reset the flag
                mp_control_flags[rc.kinova_move_to_target] = 0

            else:
                time.sleep(0.1)

    except Exception as e:
        print("mp_kinova_ee_strategy: ", e)