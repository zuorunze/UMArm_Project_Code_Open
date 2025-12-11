import robot_constants as rc
import numpy as np
import time
import os

import kinematics_mp

theta_test = 45
R_x = np.array([
    [1, 0, 0],
    [0, np.cos(theta_test), -np.sin(theta_test)],
    [0, np.sin(theta_test), np.cos(theta_test)]
])


def random_sampled_ikine_test(mp_q_robot_desired, mp_homo_desired, mp_control_flags):
    # first, get the set of points to test
    filepath = "range_of_motion_test.npy"
    arr = np.load(filepath)

    reshaped_arr = np.reshape(arr, (int(len(arr)/4), 4))

    print("random_sampled_ikine_test, data shape:", reshaped_arr.shape)

    # filter out the success point
    success_pts = []
    failed_pts = []

    for i in range(len(reshaped_arr)):
        if reshaped_arr[i,3] <= 0.002:
            success_pts.append(reshaped_arr[i,0:3])
        else:
            failed_pts.append(reshaped_arr[i,0:3])

    success_pts = np.array(success_pts)
    failed_pts = np.array(failed_pts)

    test_size = 10

    test_iterations = 10

    robot_base_spatial_SE4 = np.eye(4)

    rand_sample_index_array = np.random.randint(0, len(success_pts) - 1, size=test_size)

    result_array = np.zeros((test_iterations, test_size, 4), dtype=float)

    # wait for the user command
    
    while True:
        if mp_control_flags[rc.jacobian_control_realtime_flag_index] == 1:
            for itr in range(test_iterations):
                for idx, ridx in enumerate(rand_sample_index_array):
                    print("testting: ", itr, idx)
                    # acquire the test point
                    test_pt = success_pts[ridx]
                    
                    # move the actuator to the test point
                    new_SE4 = np.eye(4)
                    new_SE4[0:3,3] = test_pt
                    
                    mp_homo_desired[0] = new_SE4

                    # wait for the ikine to converge
                    # take the current q
                    q_curr = np.zeros(len(mp_q_robot_desired), dtype=float)
                    q_prev = np.zeros(len(mp_q_robot_desired), dtype=float)

                    while True:
                        # update the current and previous q
                        for i in range(len(q_curr)):
                            q_prev[i] = q_curr[i]

                        time.sleep(0.02)

                        for i in range(len(q_curr)):
                            q_curr[i] = mp_q_robot_desired[i]
                        
                        # if the current q is not changed too much in 0.01s
                        if np.linalg.norm(q_curr[i] - q_prev[i]) < 1e-5:
                            # stop waiting for q, move on to next point
                            break
                    
                    # do the forward kinematics:
                    ee_tip_SE4_robot_frame = kinematics_mp.fkine_mk5_with_rod_ee(rc.params, rc.my_end_effector_homo, q_curr)
                    ee_tip_SE4 = kinematics_mp.robot_frame_to_spatial_frame(ee_tip_SE4_robot_frame, robot_base_spatial_SE4)

                    ikine_error = np.linalg.norm(test_pt - ee_tip_SE4[0:3,3])
                    
                    result_array[itr, idx, 3] = ikine_error
                    result_array[itr, idx, 0:3] = test_pt

            np.save("./random_ikine_test_1.npy", result_array)
                
            while True:
                print("random_sampled_ikine_test ended!")
                print(result_array)
                time.sleep(4)
        else:
            time.sleep(0.1)

        pass
    pass

def range_of_motion_test(mp_q_robot_desired, mp_homo_desired, mp_control_flags):
    try:
        # robot_base
        robot_base_spatial_SE4 = np.eye(4)

        start_point = np.array([-0.8,-0,  +0.2], dtype=float)
        end_point =   np.array([ 0.8, 0,  -1.0], dtype=float)

        x_num = 60
        y_num = 1
        z_num = 60

        x_pts = np.linspace(start_point[0], end_point[0], num=x_num, endpoint=True)
        y_pts = np.linspace(start_point[1], end_point[1], num=y_num, endpoint=True)
        z_pts = np.linspace(start_point[2], end_point[2], num=z_num, endpoint=True)

        target_SE4 = np.eye(4)

        count = 0
        

        while True:
            if mp_control_flags[rc.jacobian_control_realtime_flag_index] == 1:
                filepath = "range_of_motion_test_4.npy"
                np.save(filepath, np.array([], dtype=float))

                for x_pt in x_pts:
                    for y_pt in y_pts:
                        for z_pt in z_pts:
                                print(count)
                                count+=1
                                target_SE4[0:3,3] = np.array([x_pt, y_pt, z_pt], dtype=float)
                                # update the desired homo
                                mp_homo_desired[0] = target_SE4

                                # take the current q
                                q_curr = np.zeros(len(mp_q_robot_desired), dtype=float)
                                q_prev = np.zeros(len(mp_q_robot_desired), dtype=float)
                                while True:
                                    # update the current and previous q
                                    for i in range(len(q_curr)):
                                        q_prev[i] = q_curr[i]

                                    time.sleep(0.01)

                                    for i in range(len(q_curr)):
                                        q_curr[i] = mp_q_robot_desired[i]
                                    
                                    # if the current q is not changed too much in 0.01s
                                    if np.linalg.norm(q_curr[i] - q_prev[i]) < 1e-4:
                                        # stop waiting for q, move on to next point
                                        break
                                    

                                # do the forward kinematics:
                                ee_tip_SE4_robot_frame = kinematics_mp.fkine_mk5_with_rod_ee(rc.params, rc.my_end_effector_homo, q_curr)
                                ee_tip_SE4 = kinematics_mp.robot_frame_to_spatial_frame(ee_tip_SE4_robot_frame, robot_base_spatial_SE4)
                                
                                ikine_error = ((x_pt - ee_tip_SE4[0,3])**2 + (y_pt - ee_tip_SE4[1,3])**2 + (z_pt - ee_tip_SE4[2,3])**2) ** 0.5

                                datavec = np.array([x_pt, y_pt, z_pt, ikine_error], dtype=float)
                                print(datavec)
                                
                                # save the data

                                arr = np.load(filepath)
                                stacked_arr = np.concatenate((arr, datavec), axis=0)
                                np.save(filepath, stacked_arr)


                        #         if ikine_error >= success_cutoff:
                        #             continue

                        #         if z_pt >= z_max:
                        #             z_max = z_pt
                        #             x_at_zmax = x_pt
                        #             y_at_zmax = y_pt
                                
                        #         if z_pt <= z_min:
                        #             z_min = z_pt
                        #             x_at_zmin = x_pt
                        #             y_at_zmin = y_pt

                                
                        # # print the error
                        # if z_max > -100 and z_min <100:
                        #     datavec = np.array([x_at_zmax, y_at_zmax, z_max, x_at_zmin, y_at_zmin, z_min], dtype=float)
                        #     print(datavec)
                            
                        #     # save the data

                        #     arr = np.load(filepath)
                        #     stacked_arr = np.concatenate((arr, datavec), axis=0)
                        #     np.save(filepath, stacked_arr)

                #===================
                print("ee_strategy_quit")
                while True:
                    print("ee_strategy_finished")
                    time.sleep(2)
            else:
                time.sleep(0.1)




    except Exception as e:
        print("range_of_motion_test", e)
    
    return


def force_transmission_demo(mp_rigid_body_homo_list, mp_homo_desired, mp_control_flags):
    try:
        ee_pos_0 = np.array([0.79621, -0.17788, 0.61149], dtype=float)

        ee_pos_1 = np.array([0.43799, -0.14382, 0.71414], dtype=float)

        ee_pos_2 = np.array([0.24228, -0.13822, 1.02561], dtype=float)
        
        while True:
            if mp_control_flags[rc.force_trans_push] == 1:
                new_SE4 = np.eye(4)
                new_SE4[0:3,3] = mp_rigid_body_homo_list[8][0:3,3]
                mp_homo_desired[0] = new_SE4

            elif mp_control_flags[rc.force_trans_testpoint] == 0:
                new_SE4 = np.eye(4)
                new_SE4[0:3,3] = ee_pos_0
                mp_homo_desired[0] = new_SE4

            elif mp_control_flags[rc.force_trans_testpoint] == 1:
                new_SE4 = np.eye(4)
                new_SE4[0:3,3] = ee_pos_1
                mp_homo_desired[0] = new_SE4

            elif mp_control_flags[rc.force_trans_testpoint] == 2:
                new_SE4 = np.eye(4)
                new_SE4[0:3,3] = ee_pos_2
                mp_homo_desired[0] = new_SE4

            else:
                time.sleep(0.1)
    except Exception as e:
        print("force_transmission_demo: ", e)

    


def generate_an_N_pointed_star(center_pt, r_in, r_out, Np, Ns_edge, z_in, z_out):
    # first, figure out the vertices.
    traj  = np.zeros((Ns_edge * 2 * Np, 3), dtype=float)
    verts = np.zeros((2*Np, 3), dtype=float)
    theta_pp = np.pi / float(Np)

    for m in range(Np):
        verts[2*m,:] = center_pt + np.array([r_out * np.cos((2.0 * float(m)) * theta_pp),
                                             r_out * np.sin((2.0 * float(m)) * theta_pp),
                                             z_out])
        verts[2*m+1, :] = center_pt + np.array([r_in * np.cos((2.0 * float(m) + 1.0) * theta_pp),
                                             r_in * np.sin((2.0 * float(m) + 1.0) * theta_pp),
                                             z_in])
        
    
    # then, place the samplint points along each edge

    for e in range(2*Np):
        start_point = verts[e,:]

        if e == 2*Np - 1:
            end_point = verts[0, :]
        else:
            end_point   = verts[e+1, :]
        
        for i in range(Ns_edge):
            new_pt = start_point + (end_point - start_point) * float(i) / float(Ns_edge)
            traj[e*Ns_edge + i, :] = new_pt
    
    return traj


def generate_a_circular_trajectory(center_pt, n_sample, r, z_offset):
    traj = np.zeros((n_sample, 3), dtype=float)
                
    for i in range(n_sample):
        x_offset = r * np.sin(2.0 * np.pi * float(i) / float(n_sample))
        y_offset = r * np.cos(2.0 * np.pi * float(i) / float(n_sample))
        traj[i, :] = center_pt + np.array([x_offset, y_offset, z_offset])
    
    return traj

def ee_strategy_drawing_circle(mp_rigid_body_homo_list, mp_homo_desired, mp_control_flags):
    try:
        n_sample = 1000
        r = 0.25
        z_offset = 0.06
        
        first_point_wait_time  = 10.0
        # waypoint_interval = 0.06
        waypoint_interval = 0.03

        while True:
            # if user gives the activation command:
            if mp_control_flags[rc.jacobian_control_realtime_flag_index] == 1:
                # first, take the current position
                initial_pos = mp_rigid_body_homo_list[6][0:3,3]
                # then, construct a circle around the initial pos. the trajectory is stored as a vector of (n_sample, 3)
                
                
                # traj = generate_a_circular_trajectory(initial_pos, n_sample, r, z_offset)
                traj = generate_an_N_pointed_star(center_pt=initial_pos,
                                                  r_in=0.1,
                                                  r_out=0.3,
                                                  Np=5,
                                                  Ns_edge=200,
                                                  z_in=0.01,
                                                  z_out=0.075,)


                # start to actuate the arm using this trajectory

                traj_ptr = 0
                new_SE4 = np.eye(4)

                
                # start the recorder
                mp_control_flags[rc.recorder_start] = 1


                while True:
                    

                    new_SE4[0:3,3] = traj[traj_ptr, :]
                    mp_homo_desired[0] = new_SE4

                    if traj_ptr == 0:
                        time.sleep(first_point_wait_time)
                    else:
                        time.sleep(waypoint_interval)

                    traj_ptr += 1
                    if traj_ptr >= len(traj):
                        traj_ptr = 0

            else:
                time.sleep(0.1)
        
    except Exception as e:
        print("draw circle: ", e)
    pass

def ee_strategy_ikine_step_response(mp_homo_desired, mp_control_flags):
    try:
        pt_0 = np.array([0.792, -0.170, 0.623])
        pt_1 = np.array([0.494, -0.041, 0.724])
        
        while True:
            if mp_control_flags[rc.jacobian_control_realtime_flag_index] == 1:
                start_time = time.time()
                interval = 5
                new_se4 = np.eye(4)
                new_se4[0:3,3] = pt_0

                while True:
                    if time.time() - start_time > 4 * interval:
                        mp_control_flags[rc.recorder_start] = 0
                        time.sleep(1)
                        print("test end!")
                    if time.time() - start_time > 2 * interval:
                        new_se4[0:3,3] = pt_1
                    elif time.time() - start_time > interval:
                        mp_control_flags[rc.recorder_start] = 1

                    mp_homo_desired[0] = new_se4
            else:
                new_se4 = np.eye(4)
                new_se4[0:3,3] = pt_0
                mp_homo_desired[0] = new_se4
                time.sleep(0.1)
                pass
    except Exception as e:
        print("ikine step response:" , e)



def ee_strategy_waypoint_tracking(mp_homo_desired, mp_control_flags):
    try:
        pt_0 = np.array([0.792, -0.170, 0.623])

        pt_1 = np.array([0.504, -0.041, 0.724])
        pt_2 = np.array([0.760, 0.144, 0.724])
        pt_3 = np.array([1.048, 0.015, 0.724])
        pt_4 = np.array([1.080, -0.300, 0.724])
        pt_5 = np.array([0.824, -0.484, 0.724])
        pt_6 = np.array([0.536, -0.355, 0.724])

        while True:
            if mp_control_flags[rc.jacobian_control_realtime_flag_index] == 1:
                
                start_time = time.time()
                interval = 10
                new_se4 = np.eye(4)
                
                while True:
                    if time.time() - start_time > 9 * interval:
                        # print("recorder stopped!")
                        mp_control_flags[rc.recorder_start] = 0
                    elif  time.time() - start_time > 8 * interval:
                        new_se4[0:3,3] = pt_0
                    elif time.time() - start_time > 7 * interval:
                        new_se4[0:3,3] = pt_5
                    elif time.time() - start_time > 6 * interval:
                        new_se4[0:3,3] = pt_2
                    elif time.time() - start_time > 5 * interval:
                        new_se4[0:3,3] = pt_4
                    elif time.time() - start_time > 4 * interval:
                        new_se4[0:3,3] = pt_6
                    elif time.time() - start_time > 3 * interval:
                        new_se4[0:3,3] = pt_3
                    elif time.time() - start_time > 2 * interval:
                        new_se4[0:3,3] = pt_1
                    elif time.time() - start_time > 1 * interval:
                        mp_control_flags[rc.recorder_start] = 1
                        # print("recorder_start!")
                        new_se4[0:3,3] = pt_0
                    elif time.time() - start_time >= 0:
                        new_se4[0:3,3] = pt_0


                    mp_homo_desired[0] = new_se4

            else:
                new_se4 = np.eye(4)
                new_se4[0:3,3] = pt_0
                mp_homo_desired[0] = new_se4
                time.sleep(0.1)
                pass
    except Exception as e:
        print("waypoint: ", e)
    return

def ee_strategy_wand_follow(mp_rigid_body_homo_list, mp_homo_desired, mp_control_flags):
    try:
        while True:
            if mp_control_flags[rc.jacobian_control_realtime_flag_index] == 1:
                wand_SE4 = mp_rigid_body_homo_list[7]

                mp_homo_desired[0] = wand_SE4

                time.sleep(1.0/120.0)
            else:
                time.sleep(0.1)

            
    except Exception as e:
        print("ee_strategy_wand_follow, ", e)

    return

def ee_strategy_random_p(mp_p_actuator, mp_control_flags):
    while True:
        try:
            if mp_control_flags[rc.actuate_random_p] == 1:
                # reset the flag to 0
                mp_control_flags[rc.actuate_random_p] = 0
                np.random.seed()
                test_p = np.random.uniform(0.0, 30.0, size=24)
                test_p = np.array(
                [ 5.18880361, 39.278431  ,  5.18777754, 31.43925185, 28.7674453 , 34.83536451,
                          24.42533352, 27.86426097, 34.57118717, 30.73791426, 34.56815866, 39.94255312,
                          32.50199647, 27.51036644, 39.73402987, 35.71415296, 10.96068679, 30.3314796 ,
                          10.9595931 , 39.93188716, 12.84384248, 28.39068009, 15.91776308, 37.38815273,]
                ) * 0.7


                test_p = np.array(
                   [9.03226783 ,17.22665022 , 8.99089491 ,39.94021757 ,17.90808501 ,17.43650379,
                    39.72365517, 38.71354833,  3.97454328, 39.88639566,  3.82836639, 13.3405645,
                    37.58907286, 30.00882113, 11.87287283,  9.1792207 , 13.15835325, 39.7643463,
                    13.12341939, 15.9301697 , 25.21266847, 15.62694477, 10.3406453 , 6.35636986,],
                    dtype=float
                ) * 0.7


                test_p = np.array(
                   [39.97146713,  1.23229865, 39.93009421,  5.55929623,  8.93413595, 12.22841974,
                    20.34880431, 27.51842564, 36.27101139, 19.61666148, 36.1248345 ,  7.02258395,
                    38.36154559, 13.21814786, 12.12167756,  3.94564703, 25.29419718, 39.51566385,
                    25.25926332, 15.82901636, 21.62540488,  5.86054244,  8.86937618,  2.38381692,],dtype=float
                ) * 0.7

                for i in range(len(test_p)):
                    mp_p_actuator[i] = test_p[i]
                
                print("new random p:", test_p)

            elif mp_control_flags[rc.actuate_random_p] == 2:
                # reset the flag to 0
                mp_control_flags[rc.actuate_random_p] = 0

                test_p = np.ones(24,dtype=float) * 5

                for i in range(len(test_p)):
                    mp_p_actuator[i] = test_p[i]

                print("all 5 psi:", test_p)
                
            else:
                time.sleep(0.1)


        except Exception as e:
            print("ee_strategy_random_p: ", e)



    return


def ee_strategy_q_step_response(mp_q_robot_desired, mp_control_flags):

    q_target_0 = np.ones([12], dtype=float) * 0.000001
    q_target_01 = np.ones([12], dtype=float) * 0.1
    q_target_n01 = np.ones([12], dtype=float) * -0.1
    

    q_target = q_target_0
    
    while True:
        try:
            if mp_control_flags[rc.q_override] == 1:
                
                if mp_control_flags[rc.q_control_idx] == 0:
                    q_target = q_target_0
                    
                elif mp_control_flags[rc.q_control_idx] == 1:
                    q_target = q_target_01
                    q_target[0] = 0.06
                    q_target[1] = 0.06
                    q_target[2] = 0.08
                    q_target[3] = 0.08

                    q_target[4] = 0.12
                    q_target[5] = 0.12
                    q_target[6] = 0.14
                    q_target[7] = 0.14
                elif mp_control_flags[rc.q_control_idx] == 2:
                    q_target = q_target_n01


                elif mp_control_flags[rc.q_control_idx] == 1000:

                    mp_control_flags[rc.recorder_start] = 1

                    # full test
                    
                    q_target = np.ones([12], dtype=float) * (-0.00001)

                    time_interval = 8
                    start_time = time.time()

                    while True:
                        if time.time() - start_time > 10 * time_interval:
                            mp_control_flags[rc.recorder_start] = 0
                            print("test end!")

                        elif time.time() - start_time > 9 * time_interval:
                            q_target = np.ones([12], dtype=float) * (0.00001)
                            
                        elif time.time() - start_time > 8 * time_interval:
                            q_target = np.ones([12], dtype=float) * (0.3)
                            q_target[0] = 0.25
                            q_target[1] = 0.25

                        elif time.time() - start_time > 7 * time_interval:
                            q_target = np.ones([12], dtype=float) * (-0.00001)
                        
                        elif time.time() - start_time > 6 * time_interval:
                            q_target = np.ones([12], dtype=float) * (-0.3)
                            q_target[0] = -0.25
                            q_target[1] = -0.25
                        
                        elif time.time() - start_time > 5 * time_interval:
                            q_target = np.ones([12], dtype=float) * (-0.00001)
                        
                        elif time.time() - start_time > 4 * time_interval:
                            q_target = np.ones([12], dtype=float) * (0.1)
                        
                        elif time.time() - start_time > 3 * time_interval:
                            q_target = np.ones([12], dtype=float) * (-0.00001)

                        elif time.time() - start_time > 2 * time_interval:
                            q_target = np.ones([12], dtype=float) * (-0.1)

                        elif time.time() - start_time > 1 * time_interval:
                            q_target = np.ones([12], dtype=float) * (-0.00001)
                            

                        for i in range(rc.num_joints):
                            mp_q_robot_desired[i] = q_target[i]
                    
                    
                for i in range(rc.num_joints):
                    mp_q_robot_desired[i] = q_target[i]

            else:
                time.sleep(0.1)
        except Exception as e:
            print("ee_strategy_q_control: ", e)
