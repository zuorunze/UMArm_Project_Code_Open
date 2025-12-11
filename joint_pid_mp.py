import numpy as np
from simple_pid import PID
import concurrent.futures
from copy import copy
import serial
import time

import robot_constants as rc




def joint_control_main(mp_q_robot, mp_q_robot_desired, mp_p_actuator, mp_control_flags):
    global pid_freq
    global serial_freq
    global kp_global
    global ki_global
    global kd_global
    global min_psi_base_global

    global min_psi_global
    global p_desired_global
    global q_desired_global
    global q_desired_command_global
    global q_current_global
    global actuator_ratio_list_global
    global pid_controllers_global

    global joint_control_enable

    global rs485_interface

    global actuator_pressure_output

    pid_freq = 500.0
    serial_freq = 85.0


    # =========================================
    # best parameters

    # kp_global = np.array([1.2, 1.2, 1.4, 1.4,   1.5, 1.5, 1.5, 1.5,   1.8, 1.8, 1.8, 1.8], dtype=float) * 0.9
    # ki_global = np.array([1.2, 1.2, 1.4, 1.4,   1.5, 1.5, 1.5, 1.5,   1.8, 1.8, 1.8, 1.8], dtype=float) * 5
    # kd_global = np.array([1.2, 1.2, 1.4, 1.4,   1.5, 1.5, 1.5, 1.5,   1.8, 1.8, 1.8, 1.8], dtype=float) * 0.05

    # min_psi_base_global = np.array([20, 20, 18, 18,    16,16,12,12,   5,5,3,3],dtype=float)

    # min_psi_base_global = np.ones([rc.num_joints], dtype=float) * 2


    # =========================================

    detune_param = 1
    # detune_param = 1
    # detune_param = 0.8
    # detune_param = 0.666666
    # detune_param = 0.333333

    kp_global = np.array([1.1, 1.1, 1.1, 1.1,   1.3, 1.3, 1.3, 1.3,   2.0, 2.0, 2.0, 2.0], dtype=float) * 0.1 * detune_param  # 0.06
    ki_global = np.array([1.5, 1.5, 1.5, 1.5,   1.5, 1.5, 1.5, 1.5,   2.0, 2.0, 2.0, 2.0], dtype=float) * 2.0 * detune_param  # 1.8
    kd_global = np.array([1.4, 1.4, 1.4, 1.4,   1.4, 1.4, 1.0, 1.0,   1.0, 1.0, 1.0, 1.0], dtype=float) * 0.003 * detune_param # 0.0046


    # min_psi_base_global = np.array([17, 17, 16, 16,    13,13,11,11,   6,6,4,4],dtype=float)

    # min_psi_base_global = np.ones(12, dtype=float) * 14.0
    # min_psi_base_global = np.ones(12, dtype=float) * 8.0

    min_psi_base_global = np.array([15, 15, 15, 15,    13,13,9,9,   5,5,3,3],dtype=float)
    # min_psi_base_global = np.array([12, 12, 12, 12,    10,10,7,7,   5,5,3,3],dtype=float)

    # min_psi_base_global = np.array([6, 6, 6, 6,    4,4,4,4,   3,3,3,3],dtype=float)

    # min_psi_base_global = np.array([8, 8, 8, 8,    11,11,8,8,   4,4,4,4],dtype=float)

    # min_psi_base_global = np.array([2.1, 2.1, 2.1, 2.1,    2.1,2.1,2.1,2.1,   2.1,2.1,2.1,2.1],dtype=float)

    # =========================================
    # =========================================


    # detune_coef = 0.6

    # kp_global = np.array([1.2, 1.2, 1.4, 1.4,   1.5, 1.5, 1.5, 1.5,   1.8, 1.8, 1.8, 1.8], dtype=float) * 0.9 * detune_coef
    # ki_global = np.array([1.2, 1.2, 1.4, 1.4,   1.5, 1.5, 1.5, 1.5,   1.8, 1.8, 1.8, 1.8], dtype=float) * 4.5 * detune_coef
    # kd_global = np.ones([12]) * 0.05 * detune_coef

    # min_psi_base_global = np.ones([rc.num_joints], dtype=float) * 1

    # =========================================


    min_psi_global = np.ones([rc.num_joints], dtype=float)
    for i in range(rc.num_joints):
        min_psi_global[i] = min_psi_base_global[i]



    p_desired_global = {key: 0 for key in rc.actuator_address_list}

    q_desired_global = np.ones([12], dtype=float) * 0.00001
    q_desired_command_global = None
    q_current_global = None
    joint_control_enable = None


    rs485_interface = None

    actuator_ratio_list_global = np.array([[1,1], [1,1], [1,1], [1,1], [1,1], [1,1], [1,1], [1,1], [1,1], [1,1], [1,1], [1,1]], dtype=float)

    pid_controllers_global = []


    try:
        joint_control_init(pid_controllers_global)

        # receive the cross-process variables
        q_current_global = mp_q_robot
        
        q_desired_command_global = mp_q_robot_desired
        # q_desired_command_global = np.ones([12]) * -0.00001

        joint_control_enable = mp_control_flags

        actuator_pressure_output = mp_p_actuator

        joint_control_pool = concurrent.futures.ThreadPoolExecutor(max_workers=2)

        joint_control_pool.submit(pid_thread_main)
        joint_control_pool.submit(serial_thread_main)

        joint_control_pool.shutdown(wait=True, cancel_futures=True)

    except Exception as e:
        print("pid_main: ", e)
    return


def pid_thread_main():
    try:
        pid_cleared = False

        while True:
                
            if joint_control_enable[rc.joint_control_flag_index] == 1:
                if pid_cleared is False:
                    for i in range(rc.num_joints):
                        q_desired_global[i] = q_current_global[i]
                    pid_controllers_global = []
                    joint_control_init(pid_controllers_global)
                    pid_cleared = True
                else:
                    # implement speed limit control!
                    for i in range(rc.num_joints):
                        
                        if q_desired_global[i] > q_desired_command_global[i]: # decrease q
                            new_q = q_desired_global[i] - rc.joint_speed_limit[i] * (1/pid_freq)
                            if new_q < q_desired_command_global[i]:
                                q_desired_global[i] = q_desired_command_global[i]
                            else:
                                q_desired_global[i] = new_q
                        else: # increase q
                            new_q = q_desired_global[i] + rc.joint_speed_limit[i] * (1/pid_freq)
                            if new_q > q_desired_command_global[i]:
                                q_desired_global[i] = q_desired_command_global[i]
                            else:
                                q_desired_global[i] = new_q

                    tune_param = joint_control_enable[rc.realtime_tune]
                    pid_update(tune_param)
                    update_desired_pressure_using_ratio_list()
            else:
                pid_cleared = False
                time.sleep(0.2)

            time.sleep(1.0/pid_freq)
    except Exception as e:
        print("pid_thread_main: ", e)


def serial_thread_main():
    try:
        while initialize_serial(rc.COM_port,rc.BAUD_RATE) is False:
            time.sleep(1)
            print("trying serial connection again......")
            
    except Exception as e:
        print("serial connection:", e)

    serial_stopped = True
    while True:
        try:
            if joint_control_enable[rc.gripper_state_update] == 1:
                gripper_state_control()
                joint_control_enable[rc.gripper_state_update] = 0
            

            if joint_control_enable[rc.actuate_init_p] == 1:
                joint_control_enable[rc.actuate_init_p] = 0

                for i in range(rc.num_joints):

                    act_id_index = rc.q_to_actuator_list_index[i]
                    act_id = rc.actuator_address_list[act_id_index]
                   
                    p_desired_global[act_id[0]] = min_psi_global[i]
                    p_desired_global[act_id[1]] = min_psi_global[i]

                send_desired_p_to_arm()



            if joint_control_enable[rc.joint_control_flag_index] == 1:
                serial_stopped = False
                if joint_control_enable[rc.actuator_enable] == 1:
                    send_desired_p_to_arm()
            else:
                if serial_stopped == False:
                    arm_stop()
                    serial_stopped = True

            
                
            

            time.sleep(1.0/serial_freq)
        except Exception as e:
            print("serial_thread_main:", e)




def pid_reset():
    for controllers in pid_controllers_global:
        controllers.setpoint = 0
        controllers._integral = 0
        controllers._last_time = time.time()
        controllers._last_output = None
        controllers._last_input = None
        controllers._last_error = None
    return

def joint_control_init(pid_controllers):
    global min_psi_global
    global actuator_ratio_list_global
    
    for idx in range(rc.num_joints):
        pid_controllers.append(PID(1.0, 0.1, 0.05, setpoint= 0.0, output_limits=(-1.0,1.0), sample_time=1.0/120.0))
    
    min_psi_global = np.ones([rc.num_joints], dtype=float)
    for i in range(rc.num_joints):
        min_psi_global[i] = min_psi_base_global[i]

    actuator_ratio_list_global = np.array([[1,1], [1,1], [1,1], [1,1], [1,1], [1,1], [1,1], [1,1], [1,1], [1,1], [1,1], [1,1]], dtype=float)

    L0_coef = 1.0
    L1_coef = 1.0
    L2_coef = 1.0

    for idx in [0,1,2,3]:
        pid_controllers[idx].Kd = kd_global[idx] * L0_coef
        pid_controllers[idx].Ki = ki_global[idx] * L0_coef
        pid_controllers[idx].Kp = kp_global[idx] * L0_coef
        pid_controllers[idx].sample_time = 1.0/pid_freq
        

    for idx in [4,5,6,7]:
        pid_controllers[idx].Kd = kd_global[idx] * L1_coef
        pid_controllers[idx].Ki = ki_global[idx] * L1_coef
        pid_controllers[idx].Kp = kp_global[idx] * L1_coef
        pid_controllers[idx].sample_time = 1.0/pid_freq
        

    for idx in [8,9,10,11]:
        pid_controllers[idx].Kd = kd_global[idx] * L2_coef
        pid_controllers[idx].Ki = ki_global[idx] * L2_coef
        pid_controllers[idx].Kp = kp_global[idx] * L2_coef
        pid_controllers[idx].sample_time = 1.0/pid_freq
        
    return


def map_value(value, old_min, old_max, new_min, new_max):
    return (value - old_min) * (new_max - new_min) / (old_max - old_min) + new_min


def pid_to_pressure_mapping_typeA(pid_out, init_pressure, max_pressure, min_pressure):
    # mapping 1:
    if pid_out > 0:
        p_act0 = map_value(pid_out, 0.0, 1.0, init_pressure, max_pressure)
        p_act1 = map_value(pid_out, 0.0, 1.0, init_pressure, min_pressure)
    elif pid_out < 0:
        p_act0 = map_value(pid_out, -1.0, 0.0, min_pressure, init_pressure)
        p_act1 = map_value(pid_out, -1.0, 0.0, max_pressure, init_pressure)
    else:
        p_act0 = init_pressure
        p_act1 = init_pressure
    
    return p_act0, p_act1


def pid_to_pressure_mapping_typeB(pid_out, init_pressure, max_pressure, min_pressure):
    Cp = (max_pressure - init_pressure) / (max_pressure - min_pressure)
    
    if pid_out >= 0 and pid_out < Cp:
        p_act0 = init_pressure + pid_out * (max_pressure - min_pressure)
        p_act1 = init_pressure
    elif pid_out >= Cp:
        p_act0 = max_pressure
        p_act1 = init_pressure - (pid_out - Cp) * (max_pressure - min_pressure)
    elif pid_out < 0 and pid_out >= (-Cp):
        p_act0 = init_pressure
        p_act1 = init_pressure - pid_out * (max_pressure - min_pressure)
    elif pid_out < -Cp:
        p_act0 = init_pressure + (pid_out + Cp) * (max_pressure - min_pressure)
        p_act1 = max_pressure


    return p_act0, p_act1


def pid_to_pressure_mapping_typeC(pid_out, init_pressure, max_pressure, min_pressure):
    k1 = (max_pressure - init_pressure) / (0.5* (max_pressure - min_pressure))
    k2 = (min_pressure - init_pressure) / (0.5* (min_pressure - max_pressure))

    if pid_out >= 0:
        p_act0 = (pid_out - 0.5 * (pid_out ** 2)) * (max_pressure - min_pressure) * k1 + init_pressure
        p_act1 = k2 * (min_pressure - max_pressure) * (pid_out**2) * 0.5 + init_pressure
    if pid_out < 0:
        p_act0 = k2 * (min_pressure - max_pressure) * (pid_out**2) * 0.5 + init_pressure
        p_act1 = (-pid_out - 0.5 * (pid_out ** 2)) * (max_pressure - min_pressure) * k1 + init_pressure

    return p_act0, p_act1

def pid_update(tune_param): 
    try:
        q_curr = np.array(copy(q_current_global))
        for i in range(len(q_curr)):
            
            this_pid = pid_controllers_global[i]
            this_pid.setpoint = q_desired_global[i]

            # rc.print_formatted(q_desired_global)

            if i in [0,1]:
                pid_out = this_pid(q_curr[i]) * 21
            elif i in [2,3]:
                pid_out = this_pid(q_curr[i]) * 18
            elif i in [4,5]:
                pid_out = this_pid(q_curr[i]) * 6
            else:
                pid_out = this_pid(q_curr[i]) * 6

            if pid_out > 1:
                pid_out = 1
            elif pid_out < -1:
                pid_out = -1

            # print("{:.2f}".format(pid_out), end=", ")

            theta_error = abs(q_desired_global[i] - q_curr[i])

            if theta_error >= 0.03:
                adaptive_gain = 1.5
            else:
                adaptive_gain = 1

            adaptive_gain = 1

            
            this_pid.Kp = kp_global[i] * adaptive_gain * tune_param
            this_pid.Ki = ki_global[i] * adaptive_gain * tune_param
            this_pid.Kd = kd_global[i] * adaptive_gain * tune_param
            

            init_pressure = min_psi_global[i]
            max_pressure = 50
            min_pressure = 1
            
            # pid out: 0: both init_pressure
            # pid out: 1: act0 large, act1 small
            # pid out: -1: act0 small, act1 large
            
            # mapping 1:
            p_act0, p_act1 = pid_to_pressure_mapping_typeA(pid_out, init_pressure, max_pressure, min_pressure)
            # p_act0, p_act1 = pid_to_pressure_mapping_typeB(pid_out, init_pressure, max_pressure, min_pressure)
            # p_act0, p_act1 = pid_to_pressure_mapping_typeC(pid_out, init_pressure, max_pressure, min_pressure)
            
            actuator_ratio_list_global[i,0] = p_act0
            actuator_ratio_list_global[i,1] = p_act1

            


    except Exception as e:
        # Code to handle the exception
        print("An error occurred in pid_update:", e)

    return


def update_desired_pressure_using_ratio_list():
    for i in range(rc.num_joints):

        act_ratio = actuator_ratio_list_global[i]
        act_id_index = rc.q_to_actuator_list_index[i]
        act_id = rc.actuator_address_list[act_id_index]
        
        # # version 1
        # if act_ratio[1] == 1:
        #     p_desired_global[act_id[1]] = min_psi_global[i]
        #     p_desired_global[act_id[0]] = min_psi_global[i] * act_ratio[0]
        # elif act_ratio[1] == 0:
        #     p_desired_global[act_id[0]] = min_psi_global[i]
        #     p_desired_global[act_id[1]] = min_psi_global[i] * act_ratio[0]

        # Version 2
        p_desired_global[act_id[0]] = act_ratio[0]
        p_desired_global[act_id[1]] = act_ratio[1]
    return


    
def initialize_serial(com,baud):
    global rs485_interface
    try:
        print("--Connecting to {}".format(com))
        rs485_interface = serial.Serial(port=com, baudrate=baud, timeout=.1)
    except:
        print("--Connection failed")
        return False
    else:
        print("--Connected")
    return True

def serial_handler():
    pass

def arm_stop():
    str_to_be_sent = "bb0 9999"
    utf8str = str_to_be_sent.encode('utf-8') + b'\n'
    rs485_interface.write(utf8str)

    time.sleep(0.2)
    str_to_be_sent = "ex0 9999"
    utf8str = str_to_be_sent.encode('utf-8') + b'\n'
    rs485_interface.write(utf8str)

    time.sleep(3)
    str_to_be_sent = "ss0 9999"
    utf8str = str_to_be_sent.encode('utf-8') + b'\n'
    rs485_interface.write(utf8str)


def gripper_state_control():
    if joint_control_enable[rc.gripper_state] == 0:
        open_angle = 70
        str_to_be_sent = "gg{:.2f} {}".format(open_angle, int(3))
        utf8str = str_to_be_sent.encode('utf-8') + b'\n'
        rs485_interface.write(utf8str)
        
    else:
        close_angle = 125
        str_to_be_sent = "gg{:.2f} {}".format(close_angle, int(3))
        utf8str = str_to_be_sent.encode('utf-8') + b'\n'
        rs485_interface.write(utf8str)
        pass


def send_desired_p_to_arm():
    # update the actuator pressure to the arm
    for i, act_addr in enumerate(rc.actuator_address_list):
        act_pressure = p_desired_global[act_addr]
        # also update the actuator pressure to the mp variable
        actuator_pressure_output[i] = p_desired_global[act_addr] * 6894.76

        str_to_be_sent = "bb{:.2f} {}".format(act_pressure, int(act_addr))
        utf8str = str_to_be_sent.encode('utf-8') + b'\n'
        rs485_interface.write(utf8str)

    
    return