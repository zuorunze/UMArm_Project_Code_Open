import numpy as np


COM_port = 'COM37'
BAUD_RATE = 115200

nlinks = 3
num_rigid_bodies = 9
num_joints = 4*nlinks
num_actuators = 8*nlinks

# Robot parameter
def param_builder(JA1,JA2,UC1,UC2,AA1,AA2,AO1,AO2,LL,JD):
    return [JA1,JA2,UC1,UC2,AA1,AA2,AO1,AO2,LL,JD]

def print_formatted(np_array):
    formatted = [f"{x:.5f}" for x in np_array]
    print(formatted)

param_link0 = param_builder(
    JA1 = 0.1, JA2 = 0.05,
    UC1 = 0, UC2 = 0,
    AA1 = 0.0285, AA2 = 0.0285,
    AO1 = 0.03, AO2 = 0.03,
    LL = 0.161868,
    JD = 0
)

param_link1 = param_builder(
    JA1 = 0.05, JA2 = 0.05,
    UC1 = 0, UC2 = 0,
    AA1 = 0.0285, AA2 = 0.0285,
    AO1 = 0.03, AO2 = 0.03,
    LL = 0.13754,
    JD = 0.047871
)

param_link2 = param_builder(
    JA1 = 0.05, JA2 = 0.05,
    UC1 = 0, UC2 = 0,
    AA1 = 0.0285, AA2 = 0.0285,
    AO1 = 0.03, AO2 = 0.03,
    LL = 0.127111,
    JD = 0.047382
)

# # for the trajectory tracing experiment, and the ikine step response
# my_end_effector_homo = [
#     [1,0,0,0],
#     [0,1,0,0],
#     [0,0,1,-0.1402],
#     [0,0,0,1]
# ]


# this is the force gauge rod ee
my_end_effector_homo = [
    [1,0,0,0],
    [0,1,0,0],
    [0,0,1,-0.1403855087774544],
    [0,0,0,1]
]

kinova_ee_frame_to_mocap_SE4 = np.array([
    [1,0,0,0],
    [0,1,0,0],
    [0,0,1,-0.005],
    [0,0,0,1]
],dtype=float)


kinova_tooltip_frame_to_mocap_SE4 = np.array([
    [1,0,0,0],
    [0,1,0,0],
    [0,0,1,0.01],
    [0,0,0,1]
],dtype=float)



d_mocap_to_ujoint_center = 0.0520

d_seg2_mocap_to_ujoint_center = 0.051

# in kg
rigid_body_mass_list = np.array([0.24, 0.02, 0.48, 0.02, 0.48, 0.02, 0.24], dtype=float) * 9.81


Bfi_seg1 = np.array([
    0.163, 0.163, 0.163, 0.163, 0.163, 0.163, 0.163, 0.163
],dtype=float) * 0.8

Nfi_seg1 = np.array([
    0.86, 0.86, 0.86, 0.86, 0.86, 0.86, 0.86, 0.86
],dtype=float) * 0.8

LBase_seg1 = np.array([
    0.217, 0.217, 0.217, 0.217, 0.217, 0.217, 0.217, 0.217
],dtype=float) * 0.8

LActOffset_seg1 = np.array([
    0.083, 0.083, 0.083, 0.083, 0.083, 0.083, 0.083, 0.083
],dtype=float) * 0.8

Bfi_seg2 = np.array([
    0.165, 0.165, 0.165, 0.165, 0.165, 0.165, 0.165, 0.165
],dtype=float) * 0.8

Nfi_seg2 = np.array([
    1.01, 1.01, 1.01, 1.01, 1.01, 1.01, 1.01, 1.01
],dtype=float) * 0.8

LBase_seg2 = np.array([
    0.193, 0.193, 0.193, 0.193, 0.193, 0.193, 0.193, 0.193
],dtype=float) * 0.8

LActOffset_seg2 = np.array([
    0.072, 0.072, 0.072, 0.072, 0.072, 0.072, 0.072, 0.072
],dtype=float) * 0.8

Bfi_seg3 = np.array([
    0.151, 0.151, 0.151, 0.151, 0.151, 0.151, 0.151, 0.151
],dtype=float) * 0.8

Nfi_seg3 = np.array([
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0
],dtype=float) * 0.8

LBase_seg3 = np.array([
    0.184, 0.184, 0.184, 0.184, 0.184, 0.184, 0.184, 0.184
],dtype=float) * 0.8

LActOffset_seg3 = np.array([
    0.065, 0.065, 0.065, 0.065, 0.065, 0.065, 0.065, 0.065
],dtype=float) * 0.8




Nfis = np.array([Nfi_seg1, Nfi_seg2, Nfi_seg3], dtype=float)
Bfis = np.array([Bfi_seg1, Bfi_seg2, Bfi_seg3], dtype=float)
LBases = np.array([LBase_seg1, LBase_seg2, LBase_seg3], dtype=float)
LActOffsets = np.array([LActOffset_seg1, LActOffset_seg2, LActOffset_seg3], dtype=float)

params = np.array([param_link0, param_link1, param_link2])

def ee_param_builder(ee1x, ee1y, ee1z, ee2x, ee2y, ee2z, gstx, gsty, gstz):
    return np.array([ee1x, ee1y, ee1z, ee2x, ee2y, ee2z, gstx, gsty, gstz], dtype=float)

ee_param = ee_param_builder(0, 0, -0.060, 0, 0, -0.078, 0, 0.097, -0.155)

# q_max_global = np.array([0.13, 0.13, 0.13, 0.13,   0.3, 0.3, 0.3, 0.3,  0.3, 0.3, 0.3, 0.3,], dtype=float)
# q_min_global = np.array([0.13, 0.13, 0.13, 0.13,   0.3, 0.3, 0.3, 0.3,  0.3, 0.3, 0.3, 0.3,], dtype=float) * -1.0


q_max_global = np.ones(12, dtype=float) * 0.4
q_min_global = np.ones(12, dtype=float) * (-0.4)

joint_speed_limit = np.array([0.2, 0.2, 
                              0.2, 0.2, 
                              0.4, 0.4,
                              0.4, 0.4,
                              0.7, 0.7,
                              0.7, 0.7,], dtype=float)

# q_max_global = np.array([0.23081567, 0.2663568, 0.32195642, 0.32573937, 0.30805444, 0.25519955, 0.25618608, 0.23053591, 0.23518803, 0.25221936, 0.26026512, 0.25185422],dtype=float)
# q_min_global = np.array([0.23853478, 0.2644964, 0.37439225, 0.32590717, 0.18242545, 0.26423334, 0.27326905, 0.23736844, 0.25051164, 0.30119105, 0.28901177, 0.24635142],dtype=float) * -1


# q_max_global = np.array([0.5, 0.01, 0.32195642, 0.1,  0.30805444, 0.25519955, 0.25618608, 0.23053591, 0.23518803, 0.25221936, 0.26026512, 0.25185422],dtype=float)
# q_min_global = np.array([0.2, 0.5, 0.37439225, -0.6, 0.18242545, 0.26423334, 0.27326905, 0.23736844, 0.25051164, 0.30119105, 0.28901177, 0.24635142],dtype=float) * -1


# q_max_global = np.ones([12])
# q_min_global = np.ones([12]) * -1


actuator_address_list = np.array(
    [74,73,69,61, 55,72,51,68,
     54,62,63,52, 64,60,66,58,
     57,65,67,53, 12,56,11,59], dtype=int)


q_to_actuator_list_index = np.array(
    [[3+8*0,1+8*0],[0+8*0,2+8*0],[7+8*0,5+8*0],[4+8*0,6+8*0],
     [3+8*1,1+8*1],[0+8*1,2+8*1],[7+8*1,5+8*1],[4+8*1,6+8*1],
     [3+8*2,1+8*2],[0+8*2,2+8*2],[7+8*2,5+8*2],[4+8*2,6+8*2],
     [3+8*3,1+8*3],[0+8*3,2+8*3],[7+8*3,5+8*3],[4+8*3,6+8*3]], dtype=int)

wand_rigid_body_id = 7


num_control_flags = 50

joint_control_flag_index = 0
jacobian_control_realtime_flag_index = 1
ee_strategy_flag_index = 2
ee_enable = 3
actuator_enable = 4
gripper_state = 5
pick_up_state = 6
gripper_state_update = 7
gripper_lift_override = 8
wearable_demo = 9
wearable_demo_place = 10
wearable_demo_angle = 11

q_override = 12
q_control_idx = 13

recorder_start = 14
recorder_new_data = 15

compliance_new_data = 16

direct_p_control = 17

actuate_random_p = 18

actuate_init_p = 19

realtime_tune = 20


kinova_enable = 21
kinova_move_delta = 22
kinova_move_spatial = 23
kinova_move_toolframe = 24
kinova_move_home = 25

kinova_move_to_target = 26

force_trans_testpoint = 27
force_trans_push = 28

# load calibration
opvar = None
opvar_LL = None