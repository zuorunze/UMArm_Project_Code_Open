import multiprocessing as mp
import time
import numpy as np
import re

import mocap_natnet_receiver_routine
import visualization_ee_mp
import joint_pid_mp
import end_effector_strategy_mp
import inverse_kinematic_ee_mp
import data_processing_mp
import dryrun
import mp_compliance
import ni_daq_mx
import direct_p_control
import kinova_main
import kinova_ee_strategy

import robot_constants as rc

# have a console run on the main thread:
def console():
    quit_all = False
    try:
        while not quit_all:
            try:
                inchars = input('Enter command or (\'h\' for list of commands)\n')
                if len(inchars)>0:
                    c1 = inchars[0].lower()
                    args = re.findall(r'[-+]?\d+\.?\d*', inchars)
                    args = [float(num) if '.' in num else int(num) for num in args]
                    # shutdown
                    if c1 == 'q':
                        mp_control_flags[rc.recorder_start] = 0
                        time.sleep(0.2)

                        quit_all = True
                        pass

                    if c1 == 's':
                        mp_control_flags[rc.joint_control_flag_index] = 0 # disable joint control
                        mp_control_flags[rc.jacobian_control_realtime_flag_index] = 0 # disable inverse kinematics
                        mp_control_flags[rc.actuator_enable] = 0
                        mp_control_flags[rc.q_override] = 0
                        mp_control_flags[rc.recorder_start] = 0

                        # data recorder stop
                        mp_control_flags[rc.recorder_start] = 0
                        print("recorder stopped")

                        # stop direct control
                        mp_control_flags[rc.direct_p_control] = 0


                    if c1 == 'w':
                        if args[0] == 1:
                            mp_control_flags[rc.jacobian_control_realtime_flag_index] = 1
                            
                            mp_control_flags[rc.joint_control_flag_index] = 1
                            mp_control_flags[rc.actuator_enable] = 1

                            mp_control_flags[rc.q_override] = 0

                    if c1 == 't':
                        print("applied realtime tune: before - ", mp_control_flags[rc.realtime_tune], "now: ", args[0])
                        mp_control_flags[rc.realtime_tune] = args[0]
                    
                    if c1 == 'e':
                        if args[0] == 1:
                            mp_control_flags[rc.actuate_init_p] = 1
                        if args[0] == 2:
                            pact_demo = np.array(
                                [1, 25, 1, 25,  25, 25, 25, 25,  1, 25, 1, 25,  25, 25, 25, 25, 1, 20, 1, 20,  20, 20, 20, 20,]
                            )
                            # create a random pressure array, pressure in range [5,25]
                            prandom = np.random.uniform(5, 25, size=rc.num_actuators)
                            for i in range(rc.num_actuators):
                                mp_p_actuator[i] = prandom[i]
                            mp_control_flags[rc.direct_p_control] = 1
                    

                    
                    if c1 == 'p':
                        if args[0] == 0:
                            mp_control_flags[rc.q_control_idx] = 0
                        elif args[0] == 1:
                            mp_control_flags[rc.q_control_idx] = 1
                        elif args[0] == 2:
                            mp_control_flags[rc.q_control_idx] = 2
                        elif args[0] == 1000:
                            mp_control_flags[rc.q_control_idx] = 1000
                        elif args[0] == 9999:
                            # starting up step control
                            mp_control_flags[rc.jacobian_control_realtime_flag_index] = 0
                            mp_control_flags[rc.q_override] = 1
                            mp_control_flags[rc.joint_control_flag_index] = 1
                            mp_control_flags[rc.actuator_enable] = 1
                            mp_control_flags[rc.q_control_idx] = 0

                    if c1 == 'm': # manual control of cartesian movements
                        
                        # put the arguments into the multiprocess data variable
                        new_delta = np.array([args[0], args[1], args[2], args[3], args[4], args[5]])
                        mp_kinova_move_delta[0] = new_delta

                        # activate the kinova flags
                        mp_control_flags[rc.kinova_move_delta] = 1
                        mp_control_flags[rc.kinova_move_spatial] = 0
                        mp_control_flags[rc.kinova_enable] = 1

                    if c1=='x':
                        if args[0] == 0:
                            mp_control_flags[rc.kinova_move_delta] = 0
                            mp_control_flags[rc.kinova_move_spatial] = 0
                            mp_control_flags[rc.kinova_move_home] = 1
                            mp_control_flags[rc.kinova_enable] = 1
                        if args[0] == 1:
                            retract_x = np.eye(4)
                            retract_x[0:3,3] = mp_rigid_body_homo_list[8][0:3,0:3] @ np.array([0,0,-0.13])
                            
                            mp_kinova_move_spatial[0] = retract_x
                            mp_control_flags[rc.kinova_move_delta] = 0
                            mp_control_flags[rc.kinova_move_spatial] = 1
                            mp_control_flags[rc.kinova_enable] = 1

                        if args[0] == 2:
                            extend_x = np.eye(4)
                            extend_x[0:3,3] = mp_rigid_body_homo_list[8][0:3,0:3] @ np.array([0,0,0.13])

                            mp_kinova_move_spatial[0] = extend_x
                            mp_control_flags[rc.kinova_move_delta] = 0
                            mp_control_flags[rc.kinova_move_spatial] = 1
                            mp_control_flags[rc.kinova_enable] = 1
                        
                        if args[0] == 3:

                            if args[1] == 1:
                                mp_control_flags[rc.kinova_move_to_target] = 1

                            if args[1] == 2:
                                mp_control_flags[rc.kinova_move_to_target] = 2
                                
                            if args[1] == 3:
                                mp_control_flags[rc.kinova_move_to_target] = 3

                            if args[1] == 4:
                                mp_control_flags[rc.kinova_move_to_target] = 4
                            
                            if args[1] == 5:
                                mp_control_flags[rc.kinova_move_to_target] = 5
                            
                            if args[1] == 6:
                                mp_control_flags[rc.kinova_move_to_target] = 6

                            
                            # align the kinova with the force sensor
                            mp_control_flags[rc.kinova_move_delta] = 0
                            mp_control_flags[rc.kinova_move_spatial] = 0
                            mp_control_flags[rc.kinova_enable] = 1
                        

                        if args[0] == 4: 
                            mp_control_flags[rc.jacobian_control_realtime_flag_index] = 1
                            mp_control_flags[rc.joint_control_flag_index] = 1
                            mp_control_flags[rc.actuator_enable] = 1
                            mp_control_flags[rc.q_override] = 0

                            # move the UMARM to loop through the testing position
                            if args[1] == 0:
                                mp_control_flags[rc.force_trans_testpoint] = 0
                                
                            elif args[1] == 1:
                                mp_control_flags[rc.force_trans_testpoint] = 1
                                
                            elif args[1] == 2:
                                mp_control_flags[rc.force_trans_testpoint] = 2
                            
                        if args[0] == 5:
                            if args[1] == 0:
                                mp_control_flags[rc.recorder_start] = 0
                                pass
                            elif args[1] == 1:
                                mp_control_flags[rc.recorder_start] = 1
                                pass


                    if c1 == 'r':
                        if args[0] == 1:
                            # generate a new configuration
                            mp_control_flags[rc.recorder_start] = 1
                            print("recorder started")
                        if args[0] == 0:
                            # generate a new configuration
                            mp_control_flags[rc.recorder_start] = 1
                            print("recorder stopped")
                pass
            except Exception as e:
                print("invalid command, ", e)
    except Exception as e:
        print("console: ", e)
    

if __name__ == "__main__":

    global mp_control_flags

    gvar_manager = mp.Manager()

    # motion capture rigid body centers acquired from the motion capture
    mp_rigid_body_homo_list = gvar_manager.list()   # managed list: can be manipulated in all processes!

    # current robot configutration acquired from the motion capture
    mp_q_robot = gvar_manager.list()
    
    # desired robot configuration to achieve some tool-frame target
    # updated by the inverse kinematics process
    mp_q_robot_desired = gvar_manager.list()

    # current robot actuator pressure
    mp_p_actuator = gvar_manager.list()
    mp_p_record   = gvar_manager.list()

    # control flags:
    # 0: enable joint control: joint pid & serial output
    # 1: enable jacobian inverse kinematic control - realtime
    mp_control_flags = gvar_manager.list()
    
    # desired homogeneous matrix for the end effector frame.
    # updated by the strategy planning process.
    mp_homo_desired = gvar_manager.list()

    # current robot jacobian matrix updated by the motion capture process
    mp_jacobian_current = gvar_manager.list()
    # jacobian should be size 3 , (4*nlinks)
    mp_jacobian_current.append(np.zeros((3, rc.num_joints), dtype=float))

    # current task space compliance at the end-effector
    mp_task_compliance = gvar_manager.list()
    mp_task_compliance.append(np.zeros((3,3), dtype=float))

    # current configuration space compliance
    mp_config_compliance = gvar_manager.list()
    mp_config_compliance.append(np.zeros((rc.num_joints,rc.num_joints), dtype=float))

    # ft sensor
    mp_ft = gvar_manager.list()

    # kinova
    mp_kinova_move_spatial = gvar_manager.list()
    mp_kinova_move_delta = gvar_manager.list()
    mp_kinova_ee_kinova_frame = gvar_manager.list()
    mp_kinova_ee_mocap = gvar_manager.list()

    
    mp_kinova_move_spatial.append(np.eye(4))
    mp_kinova_move_delta.append(np.zeros(6, dtype=float))
    mp_kinova_ee_kinova_frame.append(np.eye(4))
    mp_kinova_ee_mocap.append(np.eye(4))
    
    

    ### some initializations
    rigid_body_id_list = range(rc.num_rigid_bodies)

    

    for id in range(rc.num_control_flags):
        mp_control_flags.append(0)

    mp_control_flags[rc.realtime_tune] = 1

    for id in rigid_body_id_list:
        mp_rigid_body_homo_list.append(np.eye(4))

    for joint_id in range(rc.num_joints):
        mp_q_robot.append(0.0)
        mp_q_robot_desired.append(0.0)


    for act_id in range(rc.num_actuators):
        mp_p_actuator.append(5.0)
        mp_p_record.append(12345.00)

    for id in range(2):
        mp_homo_desired.append(np.eye(4))

    for ft_idx in range(6):
        mp_ft.append(0.0)

    ### configuration flags::

    # dry run: no mocap, robot q is updated by another process.
    is_dryrun = False
    # use_q_desired: instead of using the actual q from the motion capture,
    # we use the desired q generated by the inverse kinematics process as the current q.
    # that is, we assume that the (current q) = (desired q).
    use_q_desired = False
    # this will disable the pid & serial comm process.
    is_disable_joint_ctrl = False

    is_direct_p_control = True

    use_kinova_arm = False

    use_mocap = True

    use_force_torque = True

    if use_mocap:
        p_mocap_receive = mp.Process(target=mocap_natnet_receiver_routine.mocap_routine_main, args=(mp_rigid_body_homo_list, mp_q_robot, mp_jacobian_current, mp_control_flags))

    # visualizer process
    # To visualize real system, put mp_q_robot for second arg.
    # To visualize simu system, put mp_q_robot_desired for the second arg.
    if use_q_desired is False:
        p_visualize = mp.Process(target=visualization_ee_mp.visualization_ee_main, args=(mp_rigid_body_homo_list, mp_q_robot, mp_homo_desired, mp_task_compliance, mp_ft, mp_kinova_ee_kinova_frame))
    else:
        p_visualize = mp.Process(target=visualization_ee_mp.visualization_ee_main, args=(mp_rigid_body_homo_list, mp_q_robot_desired, mp_homo_desired, mp_task_compliance, mp_ft, mp_kinova_ee_kinova_frame))

    
    # inverse kinematic process.
    # for real system, put mp_q_robot for first arg.
    # for simu system, put mp_q_robot_desired for the first arg.
    if use_q_desired is False:
        p_inverse_kinematics_with_ee = mp.Process(target=inverse_kinematic_ee_mp.inverse_kinematics_with_static_ee_main, args=(mp_q_robot, mp_q_robot_desired, mp_homo_desired, mp_rigid_body_homo_list, mp_control_flags))
    else:
        p_inverse_kinematics_with_ee = mp.Process(target=inverse_kinematic_ee_mp.inverse_kinematics_with_static_ee_main, args=(mp_q_robot_desired, mp_q_robot_desired, mp_homo_desired, mp_rigid_body_homo_list, mp_control_flags))
    
    # process for pid and serial command sender
    if is_direct_p_control is False:
        p_joint_control = mp.Process(target=joint_pid_mp.joint_control_main, args=(mp_q_robot, mp_q_robot_desired, mp_p_record, mp_control_flags))
    else:
        p_joint_control = mp.Process(target=direct_p_control.mp_direct_p_control, args=(mp_p_actuator, mp_p_record, mp_control_flags))


    if use_kinova_arm is True:
        p_kinova = mp.Process(target=kinova_main.mp_kinova, args=(mp_kinova_ee_kinova_frame, mp_kinova_move_spatial, mp_kinova_move_delta, mp_rigid_body_homo_list, mp_control_flags))
        p_kinova_ee_strategy = mp.Process(target=kinova_ee_strategy.mp_kinova_ee_move_to_target_spatial, args=(mp_kinova_ee_kinova_frame, mp_kinova_move_spatial, mp_rigid_body_homo_list, mp_control_flags))
    # process for high-level planning stuff


    p_ee_strategy = mp.Process(target=end_effector_strategy_mp.ee_strategy_q_step_response, args=(mp_q_robot_desired, mp_control_flags))
    # p_ee_strategy = mp.Process(target=end_effector_strategy_mp.ee_strategy_wand_follow, args=(mp_rigid_body_homo_list, mp_homo_desired, mp_control_flags))
    # p_ee_strategy = mp.Process(target=end_effector_strategy_mp.ee_strategy_waypoint_tracking, args=(mp_homo_desired, mp_control_flags))
    # p_ee_strategy = mp.Process(target=end_effector_strategy_mp.ee_strategy_ikine_step_response, args=(mp_homo_desired, mp_control_flags))
    # p_ee_strategy = mp.Process(target=end_effector_strategy_mp.ee_strategy_drawing_circle, args=(mp_rigid_body_homo_list, mp_homo_desired, mp_control_flags))
    # p_ee_strategy = mp.Process(target=end_effector_strategy_mp.force_transmission_demo, args=(mp_rigid_body_homo_list, mp_homo_desired, mp_control_flags))

    # p_ee_strategy = mp.Process(target=end_effector_strategy_mp.range_of_motion_test, args=(mp_q_robot_desired, mp_homo_desired, mp_control_flags))
    # p_ee_strategy = mp.Process(target=end_effector_strategy_mp.random_sampled_ikine_test, args=(mp_q_robot_desired, mp_homo_desired, mp_control_flags))

    

    # process for recording data log
    p_data_record = mp.Process(target=data_processing_mp.data_processing_mp_main, args=(mp_rigid_body_homo_list, mp_q_robot, mp_q_robot_desired, mp_homo_desired, mp_p_record, mp_ft, mp_control_flags))

    # process for forward compliance computation
    p_forward_compliance = mp.Process(target=mp_compliance.mp_compliance_main, args=(mp_q_robot, mp_p_record, mp_jacobian_current, mp_task_compliance, mp_control_flags))

    if use_force_torque:
        # process for reading and processing the force torque sensor
        p_force_torque = mp.Process(target=ni_daq_mx.ni_daq_mx_subroutine, args=(mp_ft,))

    ### start those processes
    if use_mocap:
        p_mocap_receive.start()

    p_visualize.start()
    p_inverse_kinematics_with_ee.start()
    p_data_record.start()
    p_ee_strategy.start()
    p_forward_compliance.start()

    if use_force_torque:
        p_force_torque.start()

    if use_kinova_arm is True:
        p_kinova.start()
        p_kinova_ee_strategy.start()

    if is_disable_joint_ctrl is False:
        p_joint_control.start()

    # in the main thread (this thread) run the console app
    console()

    # if the console returns, terminate every process
    if use_mocap:
        p_mocap_receive.terminate()

    p_visualize.terminate()
    p_inverse_kinematics_with_ee.terminate()
    p_data_record.terminate()
    p_ee_strategy.terminate()
    p_forward_compliance.terminate()
    if use_force_torque:
        p_force_torque.terminate()

    if use_kinova_arm is True:
        p_kinova.terminate()
        p_kinova_ee_strategy.terminate()

    if is_disable_joint_ctrl is False:
        p_joint_control.terminate()

    # all the process join the main process
    if use_mocap:
        p_mocap_receive.join()

    p_visualize.join()
    p_inverse_kinematics_with_ee.join()
    p_data_record.join()
    p_ee_strategy.join()
    p_forward_compliance.join()
    if use_force_torque:
        p_force_torque.join()

    if use_kinova_arm is True:
        p_kinova.join()
        p_kinova_ee_strategy.join()

    if is_disable_joint_ctrl is False:
        p_joint_control.join()
    
    # shut down all the shared variables.
    gvar_manager.shutdown()