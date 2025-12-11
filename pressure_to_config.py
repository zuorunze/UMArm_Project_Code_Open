import numpy as np
import robot_constants as rc
import exponentials_mk5
from numpy import pi
from numpy import sin, cos, tan
import matplotlib.pyplot as plt
import print_link_mk5 as p5
import time

def multiprocess_get_pressure_from_config(mp_rbse4, mp_q, mp_pressure):
    min_kpa = 10000
    antagonistic_pair_id = [(3,1),(4,2),(7,5),(8,6),  (11,9),(12,10),(15,13),(16,14),  (19,17),(20,18),(23,21),(24,22)]
    
    
    while True:
        out_pressure = np.zeros(24, dtype=float)
        robot_base = mp_rbse4[0]
        ret_array = v2_compute_const_and_coef_for_3_segment_robot(rc.params, mp_q, robot_base)

        # print(ret_array)
        
        for i, const_and_coef in enumerate(ret_array):
            [const, coef] = const_and_coef
            pair_id = antagonistic_pair_id[i]

            if const < 0:
                out_pressure[pair_id[0]-1] = min_kpa
                out_pressure[pair_id[1]-1] = (min_kpa - const)/coef
                
            elif const >= 0:
                out_pressure[pair_id[0]-1] = const + min_kpa * coef
                out_pressure[pair_id[1]-1] = min_kpa
        
        
        for i in range(rc.num_actuators):
            mp_pressure[i] = out_pressure[i]


def estimate_config_with_pressure():


    return


def v2_compute_const_and_coef_for_3_segment_robot(params, target_theta, robot_base_SE4):
    ujoint_center_SE4list, rigid_body_center_of_mass_SE4list = fkine_pressure_to_config(params, target_theta, robot_base_SE4)

    

    weights = rc.rigid_body_mass_list
    ujc_list = ujoint_center_SE4list[:, 0:3, 3]
    center_gravity_list = rigid_body_center_of_mass_SE4list[:, 0:3, 3]

    seg1_param = rc.params[0, :]
    seg1_Nfi = rc.Nfis[0, :]
    seg1_Bfi = rc.Bfis[0, :]
    seg1_theta = target_theta[0:4]


    seg1_segBaseRU1 = ujoint_center_SE4list[0, 0:3, 0:3]
    seg1_segBasePU1 = ujoint_center_SE4list[0, 0:3, 3]
    seg1_segBaseRU2 = ujoint_center_SE4list[1, 0:3, 0:3]
    seg1_segBasePU2 = ujoint_center_SE4list[1, 0:3, 3]

    seg1_LActOffsetList = rc.LActOffsets[0,:]
    seg1_LBaseList = rc.LBases[0,:]
    
    p3p1const, p3p1coef, p4p2const, p4p2coef, p7p5const, p7p5coef, p8p6const, p8p6coef = v2_SEG_1_compute_pressure_const_and_coef(seg1_param, seg1_Nfi, seg1_Bfi, seg1_theta, seg1_segBaseRU1, seg1_segBasePU1, seg1_segBaseRU2, seg1_segBasePU2, weights, ujc_list, center_gravity_list, seg1_LActOffsetList, seg1_LBaseList)
    

    seg2_param = rc.params[1,:]
    seg2_Nfi = rc.Nfis[1,:]
    seg2_Bfi = rc.Bfis[1,:]
    seg2_theta = target_theta[4:8]


    seg2_segBaseRU1 = ujoint_center_SE4list[2, 0:3, 0:3]
    seg2_segBasePU1 = ujoint_center_SE4list[2, 0:3, 3]
    seg2_segBaseRU2 = ujoint_center_SE4list[3, 0:3, 0:3]
    seg2_segBasePU2 = ujoint_center_SE4list[3, 0:3, 3]

    seg2_LActOffsetList = rc.LActOffsets[1,:]
    seg2_LBaseList = rc.LBases[1,:]
    
    p11p9const, p11p9coef, p12p10const, p12p10coef, p15p13const, p15p13coef, p14p12const, p14p12coef = v2_SEG_2_compute_pressure_const_and_coef(seg2_param, seg2_Nfi, seg2_Bfi, seg2_theta, seg2_segBaseRU1, seg2_segBasePU1, seg2_segBaseRU2, seg2_segBasePU2, weights, ujc_list, center_gravity_list, seg2_LActOffsetList, seg2_LBaseList)
    
    seg3_param = rc.params[2,:]
    seg3_Nfi = rc.Nfis[2,:]
    seg3_Bfi = rc.Bfis[2,:]
    seg3_theta = target_theta[8:12]

    
    seg3_segBaseRU1 = ujoint_center_SE4list[4, 0:3, 0:3]
    seg3_segBasePU1 = ujoint_center_SE4list[4, 0:3, 3]
    seg3_segBaseRU2 = ujoint_center_SE4list[5, 0:3, 0:3]
    seg3_segBasePU2 = ujoint_center_SE4list[5, 0:3, 3]

    seg3_LActOffsetList = rc.LActOffsets[2,:]
    seg3_LBaseList = rc.LBases[2,:]

    p19p17const, p19p17coef, p20p18const, p20p18coef, p23p21const, p23p21coef, p24p22const, p24p22coef = v2_SEG_3_compute_pressure_const_and_coef(seg3_param, seg3_Nfi, seg3_Bfi, seg3_theta, seg3_segBaseRU1, seg3_segBasePU1, seg3_segBaseRU2, seg3_segBasePU2, weights, ujc_list, center_gravity_list, seg3_LActOffsetList, seg3_LBaseList)
    

    ret_array = np.array([[p3p1const, p3p1coef], [p4p2const, p4p2coef], [p7p5const, p7p5coef], [p8p6const, p8p6coef], 
                          [p11p9const, p11p9coef], [p12p10const, p12p10coef], [p15p13const, p15p13coef], [p14p12const, p14p12coef], 
                          [p19p17const, p19p17coef], [p20p18const, p20p18coef], [p23p21const, p23p21coef], [p24p22const, p24p22coef]]
                          , dtype=float)
    
    return ret_array


def fkine_pressure_to_config(params, theta, robot_base_SE4):
    try:
        # First, do a forward kinematics and acquire the needed coordinates:
        # 1. ujoint centers * 6
        # 2. center of mass * 7
        # 3. segment base * 6
        curr_SE4 = robot_base_SE4

        ujoint_center_SE4list             = np.zeros((rc.nlinks*2, 4, 4), dtype=float)
        rigid_body_center_of_mass_SE4list = np.zeros((rc.nlinks*2 + 1, 4, 4), dtype=float)
        
        for i in range(rc.nlinks):
            # first, forward kinematics to get the segment base
            [JA1,JA2,UC1,UC2,AA1,AA2,AO1,AO2,LL,JD] = params[i]
            e1234 = exponentials_mk5.get_e1234(params[i, :], theta[0+i*4:4+i*4])
            e12 = exponentials_mk5.get_e12(params[i, :], theta[0+i*4:4+i*4])

            gst0 = np.array([
                [1,0,0,0],
                [0,1,0,0],
                [0,0,1,-1 * (UC1+AA1+LL+AA2+UC2)],
                [0,0,0,1]
            ], dtype=float)

            gs_c0_0 = [[1,0,0,0], [0,1,0,0], [0,0,1,-1*0.5*JD],[0,0,0,1]]
            gs_c1_0 = [[1,0,0,0], [0,1,0,0], [0,0,1,-1*(UC1+AA1+0.5*LL)],[0,0,0,1]]

            JD_shift = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,-JD],[0,0,0,1]], dtype=float)
            
            # after applying the joint distance shift, this is the first ujoint outer rim
            # the ujoint center can also be acquired.
            ujoint_center_SE4list[2*i, :, :] = curr_SE4 @ JD_shift

            # now get the rigid bodies center of mass
            c0_SE4 = curr_SE4 @ gs_c0_0
            c1_SE4 = curr_SE4 @ JD_shift @ e12 @ gs_c1_0

            rigid_body_center_of_mass_SE4list[i*2, :, :]   = c0_SE4
            rigid_body_center_of_mass_SE4list[i*2+1, :, :] = c1_SE4

            # move on to the next segment
            curr_SE4 = curr_SE4 @ JD_shift @ e1234 @ gst0
            ujoint_center_SE4list[2*i + 1, :, :] = curr_SE4
        
        # put the end-effector into the rigid body center of mass list as well
        gs_last_0 = curr_SE4
        rigid_body_center_of_mass_SE4list[-1, :, :] = gs_last_0
    except Exception as e:
        print("error in fkine_pressure_to_config: ", e)

    return ujoint_center_SE4list, rigid_body_center_of_mass_SE4list


def sec(n):
    return 1.0/np.cos(n)



def v2_SEG_1_compute_pressure_const_and_coef(param, Nfi, Bfi, theta, segBaseRU1, segBasePU1, segBaseRU2, segBasePU2, weights, ujcList, centerGravityList, LActOffsetList, LBaseList):


    [JA1,JA2,UC1,UC2,AA1,AA2,AO1,AO2,LL,JD] = param
    [Nf1, Nf2, Nf3, Nf4, Nf5, Nf6, Nf7, Nf8] = Nfi
    [Bf1, Bf2, Bf3, Bf4, Bf5, Bf6, Bf7, Bf8] = Bfi
    [theta1, theta2, theta3, theta4] = theta

    [[u1r11, u1r12, u1r13], [u1r21, u1r22, u1r23], [u1r31, u1r32, u1r33]] = segBaseRU1
    [u1sbpx, u1sbpy, u1sbpz] = segBasePU1

    [[u2r11, u2r12, u2r13], [u2r21, u2r22, u2r23], [u2r31, u2r32, u2r33]] = segBaseRU2
    [u2sbpx, u2sbpy, u2sbpz] = segBasePU2

    [weightRB1, weightRB2, weightRB3,weightRB4,weightRB5,weightRB6, weightRB7] = weights
    [ujc1x, ujc1y, ujc1z] = ujcList[0] 
    [ujc2x, ujc2y, ujc2z] = ujcList[1] 
    [ujc3x, ujc3y, ujc3z] = ujcList[2] 
    [ujc4x, ujc4y, ujc4z] = ujcList[3] 
    [ujc5x, ujc5y, ujc5z] = ujcList[4] 
    [ujc6x, ujc6y, ujc6z] = ujcList[5]

    [cg1x, cg1y, cg1z] = centerGravityList[0]
    [cg2x, cg2y, cg2z] = centerGravityList[1]
    [cg3x, cg3y, cg3z] = centerGravityList[2]
    [cg4x, cg4y, cg4z] = centerGravityList[3]
    [cg5x, cg5y, cg5z] = centerGravityList[4]
    [cg6x, cg6y, cg6z] = centerGravityList[5]
    [cg7x, cg7y, cg7z] = centerGravityList[6]

    [LActOffset1, LActOffset2, LActOffset3, LActOffset4, LActOffset5, LActOffset6, LActOffset7, LActOffset8] = LActOffsetList
    [LBase1, LBase2, LBase3, LBase4, LBase5, LBase6, LBase7, LBase8] = LBaseList

    p3p1const=(-4)*JA1**(-1)*Nf3**2*pi*(((-1)*cg2x*u1r13*u1r31+(-1)* \
    cg2y*u1r23*u1r31+cg2x*u1r11*u1r33+cg2y*u1r21*u1r33+u1r13* \
    u1r31*ujc1x+(-1)*u1r11*u1r33*ujc1x+u1r23*u1r31*ujc1y+(-1)* \
    u1r21*u1r33*ujc1y)*weightRB2+((-1)*cg3x*u1r13*u1r31+(-1)* \
    cg3y*u1r23*u1r31+cg3x*u1r11*u1r33+cg3y*u1r21*u1r33+u1r13* \
    u1r31*ujc1x+(-1)*u1r11*u1r33*ujc1x+u1r23*u1r31*ujc1y+(-1)* \
    u1r21*u1r33*ujc1y)*weightRB3+((-1)*cg4x*u1r13*u1r31+(-1)* \
    cg4y*u1r23*u1r31+cg4x*u1r11*u1r33+cg4y*u1r21*u1r33+u1r13* \
    u1r31*ujc1x+(-1)*u1r11*u1r33*ujc1x+u1r23*u1r31*ujc1y+(-1)* \
    u1r21*u1r33*ujc1y)*weightRB4+((-1)*cg5x*u1r13*u1r31+(-1)* \
    cg5y*u1r23*u1r31+cg5x*u1r11*u1r33+cg5y*u1r21*u1r33+u1r13* \
    u1r31*ujc1x+(-1)*u1r11*u1r33*ujc1x+u1r23*u1r31*ujc1y+(-1)* \
    u1r21*u1r33*ujc1y)*weightRB5+((-1)*cg6x*u1r13*u1r31+(-1)* \
    cg6y*u1r23*u1r31+cg6x*u1r11*u1r33+cg6y*u1r21*u1r33+u1r13* \
    u1r31*ujc1x+(-1)*u1r11*u1r33*ujc1x+u1r23*u1r31*ujc1y+(-1)* \
    u1r21*u1r33*ujc1y)*weightRB6+((-1)*cg7x*u1r13*u1r31+(-1)* \
    cg7y*u1r23*u1r31+cg7x*u1r11*u1r33+cg7y*u1r21*u1r33+u1r13* \
    u1r31*ujc1x+(-1)*u1r11*u1r33*ujc1x+u1r23*u1r31*ujc1y+(-1)* \
    u1r21*u1r33*ujc1y)*weightRB7)*sec(theta1)*(AA1*cos(theta2)+ \
    AO1*sin(theta2))**(-1)*(AA1**2+AO1**2+JA1**2+(-2)*AO1*JA1* \
    cos(theta2)+2*AA1*JA1*sin(theta2))**(1/2)*(Bf3**2+(-3)*(( \
    AA1**2+(AO1+(-1)*JA1)**2)**(1/2)+LActOffset3+(-1)*LBase3+(-1)*( \
    AA1**2+AO1**2+JA1**2+(-2)*AO1*JA1*cos(theta2)+2*AA1*JA1*sin( \
    theta2))**(1/2))**2)**(-1)






    p3p1coef=(-1)*Nf1**(-2)*Nf3**2*(AA1*cos(theta2)+(-1)*AO1*sin(theta2)) \
    *(AA1*cos(theta2)+AO1*sin(theta2))**(-1)*(AA1**2+AO1**2+ \
    JA1**2+(-2)*AO1*JA1*cos(theta2)+(-2)*AA1*JA1*sin(theta2))**( \
    -1/2)*(AA1**2+AO1**2+JA1**2+(-2)*AO1*JA1*cos(theta2)+2*AA1* \
    JA1*sin(theta2))**(1/2)*((-6)*AA1**2+(-6)*AO1**2+Bf1**2+6* \
    AO1*JA1+(-6)*JA1**2+(-6)*(AA1**2+(AO1+(-1)*JA1)**2)**(1/2)* \
    LActOffset1+(-3)*LActOffset1**2+6*(AA1**2+(AO1+(-1)*JA1)**2)**( \
    1/2)*LBase1+6*LActOffset1*LBase1+(-3)*LBase1**2+6*AO1*JA1* \
    cos(theta2)+6*AA1*JA1*sin(theta2)+6*LActOffset1*(AA1**2+ \
    AO1**2+JA1**2+(-2)*AO1*JA1*cos(theta2)+(-2)*AA1*JA1*sin( \
    theta2))**(1/2)+(-6)*LBase1*(AA1**2+AO1**2+JA1**2+(-2)*AO1* \
    JA1*cos(theta2)+(-2)*AA1*JA1*sin(theta2))**(1/2)+6*((AA1**2+( \
    AO1+(-1)*JA1)**2)*(AA1**2+AO1**2+JA1**2+(-2)*AO1*JA1*cos( \
    theta2)+(-2)*AA1*JA1*sin(theta2)))**(1/2))*(6*AA1**2+6* \
    AO1**2+(-1)*Bf3**2+(-6)*AO1*JA1+6*JA1**2+6*(AA1**2+(AO1+(-1) \
    *JA1)**2)**(1/2)*LActOffset3+3*LActOffset3**2+(-6)*(AA1**2+( \
    AO1+(-1)*JA1)**2)**(1/2)*LBase3+(-6)*LActOffset3*LBase3+3* \
    LBase3**2+(-6)*AO1*JA1*cos(theta2)+6*AA1*JA1*sin(theta2)+( \
    -6)*LActOffset3*(AA1**2+AO1**2+JA1**2+(-2)*AO1*JA1*cos( \
    theta2)+2*AA1*JA1*sin(theta2))**(1/2)+6*LBase3*(AA1**2+ \
    AO1**2+JA1**2+(-2)*AO1*JA1*cos(theta2)+2*AA1*JA1*sin(theta2) \
    )**(1/2)+(-6)*((AA1**2+(AO1+(-1)*JA1)**2)*(AA1**2+AO1**2+ \
    JA1**2+(-2)*AO1*JA1*cos(theta2)+2*AA1*JA1*sin(theta2)))**( \
    1/2))**(-1)






    p4p2const=4*JA1**(-1)*Nf4**2*pi*((cg2x*u1r13*u1r32+cg2y*u1r23*u1r32+ \
    (-1)*cg2x*u1r12*u1r33+(-1)*cg2y*u1r22*u1r33+(-1)*u1r13* \
    u1r32*ujc1x+u1r12*u1r33*ujc1x+(-1)*u1r23*u1r32*ujc1y+u1r22* \
    u1r33*ujc1y)*weightRB2+(cg3x*u1r13*u1r32+cg3y*u1r23*u1r32+( \
    -1)*cg3x*u1r12*u1r33+(-1)*cg3y*u1r22*u1r33+(-1)*u1r13* \
    u1r32*ujc1x+u1r12*u1r33*ujc1x+(-1)*u1r23*u1r32*ujc1y+u1r22* \
    u1r33*ujc1y)*weightRB3+(cg4x*u1r13*u1r32+cg4y*u1r23*u1r32+( \
    -1)*cg4x*u1r12*u1r33+(-1)*cg4y*u1r22*u1r33+(-1)*u1r13* \
    u1r32*ujc1x+u1r12*u1r33*ujc1x+(-1)*u1r23*u1r32*ujc1y+u1r22* \
    u1r33*ujc1y)*weightRB4+(cg5x*u1r13*u1r32+cg5y*u1r23*u1r32+( \
    -1)*cg5x*u1r12*u1r33+(-1)*cg5y*u1r22*u1r33+(-1)*u1r13* \
    u1r32*ujc1x+u1r12*u1r33*ujc1x+(-1)*u1r23*u1r32*ujc1y+u1r22* \
    u1r33*ujc1y)*weightRB5+(cg6x*u1r13*u1r32+cg6y*u1r23*u1r32+( \
    -1)*cg6x*u1r12*u1r33+(-1)*cg6y*u1r22*u1r33+(-1)*u1r13* \
    u1r32*ujc1x+u1r12*u1r33*ujc1x+(-1)*u1r23*u1r32*ujc1y+u1r22* \
    u1r33*ujc1y)*weightRB6+(cg7x*u1r13*u1r32+cg7y*u1r23*u1r32+( \
    -1)*cg7x*u1r12*u1r33+(-1)*cg7y*u1r22*u1r33+(-1)*u1r13* \
    u1r32*ujc1x+u1r12*u1r33*ujc1x+(-1)*u1r23*u1r32*ujc1y+u1r22* \
    u1r33*ujc1y)*weightRB7)*(AA1*cos(theta1)*cos(theta2)+(-1)* \
    AO1*sin(theta1))**(-1)*(AA1**2+AO1**2+JA1**2+(-2)*AO1*JA1* \
    cos(theta1)+(-2)*AA1*JA1*cos(theta2)*sin(theta1))**(1/2)*( \
    Bf4**2+(-3)*((AA1**2+(AO1+(-1)*JA1)**2)**(1/2)+LActOffset4+(-1) \
    *LBase4+(-1)*(AA1**2+AO1**2+JA1**2+(-2)*AO1*JA1*cos(theta1)+( \
    -2)*AA1*JA1*cos(theta2)*sin(theta1))**(1/2))**2)**(-1)






    p4p2coef=Nf2**(-2)*Nf4**2*(AA1*cos(theta1)*cos(theta2)+(-1)*AO1*sin( \
    theta1))**(-1)*(AA1*cos(theta1)*cos(theta2)+AO1*sin(theta1))* \
    (AA1**2+AO1**2+JA1**2+(-2)*AO1*JA1*cos(theta1)+(-2)*AA1*JA1* \
    cos(theta2)*sin(theta1))**(1/2)*(AA1**2+AO1**2+JA1**2+(-2)* \
    AO1*JA1*cos(theta1)+2*AA1*JA1*cos(theta2)*sin(theta1))**( \
    -1/2)*(6*AA1**2+6*AO1**2+(-1)*Bf4**2+(-6)*AO1*JA1+6*JA1**2+ \
    6*(AA1**2+(AO1+(-1)*JA1)**2)**(1/2)*LActOffset4+3* \
    LActOffset4**2+(-6)*(AA1**2+(AO1+(-1)*JA1)**2)**(1/2)*LBase4+( \
    -6)*LActOffset4*LBase4+3*LBase4**2+(-6)*AO1*JA1*cos(theta1)+ \
    (-6)*AA1*JA1*cos(theta2)*sin(theta1)+(-6)*LActOffset4*( \
    AA1**2+AO1**2+JA1**2+(-2)*AO1*JA1*cos(theta1)+(-2)*AA1*JA1* \
    cos(theta2)*sin(theta1))**(1/2)+6*LBase4*(AA1**2+AO1**2+JA1**2+ \
    (-2)*AO1*JA1*cos(theta1)+(-2)*AA1*JA1*cos(theta2)*sin( \
    theta1))**(1/2)+(-6)*((AA1**2+(AO1+(-1)*JA1)**2)*(AA1**2+ \
    AO1**2+JA1**2+(-2)*AO1*JA1*cos(theta1)+(-2)*AA1*JA1*cos( \
    theta2)*sin(theta1)))**(1/2))**(-1)*(6*AA1**2+6*AO1**2+(-1)* \
    Bf2**2+(-6)*AO1*JA1+6*JA1**2+6*(AA1**2+(AO1+(-1)*JA1)**2)**( \
    1/2)*LActOffset2+3*LActOffset2**2+(-6)*(AA1**2+(AO1+(-1)*JA1) \
    **2)**(1/2)*LBase2+(-6)*LActOffset2*LBase2+3*LBase2**2+(-6)* \
    AO1*JA1*cos(theta1)+6*AA1*JA1*cos(theta2)*sin(theta1)+(-6)* \
    LActOffset2*(AA1**2+AO1**2+JA1**2+(-2)*AO1*JA1*cos(theta1)+2* \
    AA1*JA1*cos(theta2)*sin(theta1))**(1/2)+6*LBase2*(AA1**2+ \
    AO1**2+JA1**2+(-2)*AO1*JA1*cos(theta1)+2*AA1*JA1*cos(theta2) \
    *sin(theta1))**(1/2)+(-6)*((AA1**2+(AO1+(-1)*JA1)**2)*(AA1**2+ \
    AO1**2+JA1**2+(-2)*AO1*JA1*cos(theta1)+2*AA1*JA1*cos(theta2) \
    *sin(theta1)))**(1/2))






    p7p5const=(-2)*2**(1/2)*JA2**(-1)*Nf7**2*pi*(cg3x*(u2r13*(u2r31+ \
    u2r32)+(-1)*(u2r11+u2r12)*u2r33)*weightRB3+cg3y*(u2r23*( \
    u2r31+u2r32)+(-1)*(u2r21+u2r22)*u2r33)*weightRB3+(-1)*u2r13* \
    u2r31*ujc2x*weightRB3+(-1)*u2r13*u2r32*ujc2x*weightRB3+ \
    u2r11*u2r33*ujc2x*weightRB3+u2r12*u2r33*ujc2x*weightRB3+(-1) \
    *u2r23*u2r31*ujc2y*weightRB3+(-1)*u2r23*u2r32*ujc2y* \
    weightRB3+u2r21*u2r33*ujc2y*weightRB3+u2r22*u2r33*ujc2y* \
    weightRB3+cg4x*u2r13*u2r31*weightRB4+cg4y*u2r23*u2r31* \
    weightRB4+cg4x*u2r13*u2r32*weightRB4+cg4y*u2r23*u2r32* \
    weightRB4+(-1)*cg4x*u2r11*u2r33*weightRB4+(-1)*cg4x*u2r12* \
    u2r33*weightRB4+(-1)*cg4y*u2r21*u2r33*weightRB4+(-1)*cg4y* \
    u2r22*u2r33*weightRB4+(-1)*u2r13*u2r31*ujc2x*weightRB4+(-1) \
    *u2r13*u2r32*ujc2x*weightRB4+u2r11*u2r33*ujc2x*weightRB4+ \
    u2r12*u2r33*ujc2x*weightRB4+(-1)*u2r23*u2r31*ujc2y* \
    weightRB4+(-1)*u2r23*u2r32*ujc2y*weightRB4+u2r21*u2r33* \
    ujc2y*weightRB4+u2r22*u2r33*ujc2y*weightRB4+cg5x*u2r13* \
    u2r31*weightRB5+cg5y*u2r23*u2r31*weightRB5+cg5x*u2r13* \
    u2r32*weightRB5+cg5y*u2r23*u2r32*weightRB5+(-1)*cg5x*u2r11* \
    u2r33*weightRB5+(-1)*cg5x*u2r12*u2r33*weightRB5+(-1)*cg5y* \
    u2r21*u2r33*weightRB5+(-1)*cg5y*u2r22*u2r33*weightRB5+(-1)* \
    u2r13*u2r31*ujc2x*weightRB5+(-1)*u2r13*u2r32*ujc2x* \
    weightRB5+u2r11*u2r33*ujc2x*weightRB5+u2r12*u2r33*ujc2x* \
    weightRB5+(-1)*u2r23*u2r31*ujc2y*weightRB5+(-1)*u2r23* \
    u2r32*ujc2y*weightRB5+u2r21*u2r33*ujc2y*weightRB5+u2r22* \
    u2r33*ujc2y*weightRB5+cg6x*u2r13*u2r31*weightRB6+cg6y* \
    u2r23*u2r31*weightRB6+cg6x*u2r13*u2r32*weightRB6+cg6y* \
    u2r23*u2r32*weightRB6+(-1)*cg6x*u2r11*u2r33*weightRB6+(-1)* \
    cg6x*u2r12*u2r33*weightRB6+(-1)*cg6y*u2r21*u2r33*weightRB6+ \
    (-1)*cg6y*u2r22*u2r33*weightRB6+(-1)*u2r13*u2r31*ujc2x* \
    weightRB6+(-1)*u2r13*u2r32*ujc2x*weightRB6+u2r11*u2r33* \
    ujc2x*weightRB6+u2r12*u2r33*ujc2x*weightRB6+(-1)*u2r23* \
    u2r31*ujc2y*weightRB6+(-1)*u2r23*u2r32*ujc2y*weightRB6+ \
    u2r21*u2r33*ujc2y*weightRB6+u2r22*u2r33*ujc2y*weightRB6+ \
    cg7x*u2r13*u2r31*weightRB7+cg7y*u2r23*u2r31*weightRB7+cg7x* \
    u2r13*u2r32*weightRB7+cg7y*u2r23*u2r32*weightRB7+(-1)*cg7x* \
    u2r11*u2r33*weightRB7+(-1)*cg7x*u2r12*u2r33*weightRB7+(-1)* \
    cg7y*u2r21*u2r33*weightRB7+(-1)*cg7y*u2r22*u2r33*weightRB7+ \
    (-1)*u2r13*u2r31*ujc2x*weightRB7+(-1)*u2r13*u2r32*ujc2x* \
    weightRB7+u2r11*u2r33*ujc2x*weightRB7+u2r12*u2r33*ujc2x* \
    weightRB7+(-1)*u2r23*u2r31*ujc2y*weightRB7+(-1)*u2r23* \
    u2r32*ujc2y*weightRB7+u2r21*u2r33*ujc2y*weightRB7+u2r22* \
    u2r33*ujc2y*weightRB7)*sec(theta4)*(AA2**2+AO2**2+JA2**2+(-2) \
    *AO2*JA2*cos(theta4)+2*AA2*JA2*cos(theta3)*sin(theta4))**( \
    1/2)*(6*AA2**2+6*AO2**2+(-1)*Bf7**2+(-6)*AO2*JA2+6*JA2**2+ \
    6*(AA2**2+(AO2+(-1)*JA2)**2)**(1/2)*LActOffset7+3* \
    LActOffset7**2+(-6)*(AA2**2+(AO2+(-1)*JA2)**2)**(1/2)*LBase7+( \
    -6)*LActOffset7*LBase7+3*LBase7**2+(-6)*AO2*JA2*cos(theta4)+ \
    6*AA2*JA2*cos(theta3)*sin(theta4)+(-6)*LActOffset7*(AA2**2+ \
    AO2**2+JA2**2+(-2)*AO2*JA2*cos(theta4)+2*AA2*JA2*cos(theta3) \
    *sin(theta4))**(1/2)+6*LBase7*(AA2**2+AO2**2+JA2**2+(-2)*AO2* \
    JA2*cos(theta4)+2*AA2*JA2*cos(theta3)*sin(theta4))**(1/2)+( \
    -6)*((AA2**2+(AO2+(-1)*JA2)**2)*(AA2**2+AO2**2+JA2**2+(-2)* \
    AO2*JA2*cos(theta4)+2*AA2*JA2*cos(theta3)*sin(theta4)))**( \
    1/2))**(-1)*(AA2*cos(theta3)+AO2*tan(theta4))**(-1)






    p7p5coef=(-1)*Nf5**(-2)*Nf7**2*(AA2**2+AO2**2+JA2**2+(-2)*AO2*JA2* \
    cos(theta4)+(-2)*AA2*JA2*cos(theta3)*sin(theta4))**(-1/2)*( \
    AA2**2+AO2**2+JA2**2+(-2)*AO2*JA2*cos(theta4)+2*AA2*JA2*cos( \
    theta3)*sin(theta4))**(1/2)*((-6)*AA2**2+(-6)*AO2**2+Bf5**2+ \
    6*AO2*JA2+(-6)*JA2**2+(-6)*(AA2**2+(AO2+(-1)*JA2)**2)**(1/2) \
    *LActOffset5+(-3)*LActOffset5**2+6*(AA2**2+(AO2+(-1)*JA2)**2) \
    **(1/2)*LBase5+6*LActOffset5*LBase5+(-3)*LBase5**2+6*AO2* \
    JA2*cos(theta4)+6*AA2*JA2*cos(theta3)*sin(theta4)+6* \
    LActOffset5*(AA2**2+AO2**2+JA2**2+(-2)*AO2*JA2*cos(theta4)+( \
    -2)*AA2*JA2*cos(theta3)*sin(theta4))**(1/2)+(-6)*LBase5*( \
    AA2**2+AO2**2+JA2**2+(-2)*AO2*JA2*cos(theta4)+(-2)*AA2*JA2* \
    cos(theta3)*sin(theta4))**(1/2)+6*((AA2**2+(AO2+(-1)*JA2)**2)* \
    (AA2**2+AO2**2+JA2**2+(-2)*AO2*JA2*cos(theta4)+(-2)*AA2*JA2* \
    cos(theta3)*sin(theta4)))**(1/2))*(6*AA2**2+6*AO2**2+(-1)* \
    Bf7**2+(-6)*AO2*JA2+6*JA2**2+6*(AA2**2+(AO2+(-1)*JA2)**2)**( \
    1/2)*LActOffset7+3*LActOffset7**2+(-6)*(AA2**2+(AO2+(-1)*JA2) \
    **2)**(1/2)*LBase7+(-6)*LActOffset7*LBase7+3*LBase7**2+(-6)* \
    AO2*JA2*cos(theta4)+6*AA2*JA2*cos(theta3)*sin(theta4)+(-6)* \
    LActOffset7*(AA2**2+AO2**2+JA2**2+(-2)*AO2*JA2*cos(theta4)+2* \
    AA2*JA2*cos(theta3)*sin(theta4))**(1/2)+6*LBase7*(AA2**2+ \
    AO2**2+JA2**2+(-2)*AO2*JA2*cos(theta4)+2*AA2*JA2*cos(theta3) \
    *sin(theta4))**(1/2)+(-6)*((AA2**2+(AO2+(-1)*JA2)**2)*(AA2**2+ \
    AO2**2+JA2**2+(-2)*AO2*JA2*cos(theta4)+2*AA2*JA2*cos(theta3) \
    *sin(theta4)))**(1/2))**(-1)*(AA2*cos(theta3)+(-1)*AO2*tan( \
    theta4))*(AA2*cos(theta3)+AO2*tan(theta4))**(-1)






    p8p6const=2*2**(1/2)*JA2**(-1)*Nf8**2*pi*(cg3x*(u2r13*(u2r31+(-1)* \
    u2r32)+((-1)*u2r11+u2r12)*u2r33)*weightRB3+cg3y*(u2r23*u2r31+ \
    (-1)*u2r23*u2r32+(-1)*u2r21*u2r33+u2r22*u2r33)*weightRB3+( \
    -1)*u2r13*u2r31*ujc2x*weightRB3+u2r13*u2r32*ujc2x* \
    weightRB3+u2r11*u2r33*ujc2x*weightRB3+(-1)*u2r12*u2r33* \
    ujc2x*weightRB3+(-1)*u2r23*u2r31*ujc2y*weightRB3+u2r23* \
    u2r32*ujc2y*weightRB3+u2r21*u2r33*ujc2y*weightRB3+(-1)* \
    u2r22*u2r33*ujc2y*weightRB3+cg4x*u2r13*u2r31*weightRB4+ \
    cg4y*u2r23*u2r31*weightRB4+(-1)*cg4x*u2r13*u2r32*weightRB4+ \
    (-1)*cg4y*u2r23*u2r32*weightRB4+(-1)*cg4x*u2r11*u2r33* \
    weightRB4+cg4x*u2r12*u2r33*weightRB4+(-1)*cg4y*u2r21*u2r33* \
    weightRB4+cg4y*u2r22*u2r33*weightRB4+(-1)*u2r13*u2r31* \
    ujc2x*weightRB4+u2r13*u2r32*ujc2x*weightRB4+u2r11*u2r33* \
    ujc2x*weightRB4+(-1)*u2r12*u2r33*ujc2x*weightRB4+(-1)* \
    u2r23*u2r31*ujc2y*weightRB4+u2r23*u2r32*ujc2y*weightRB4+ \
    u2r21*u2r33*ujc2y*weightRB4+(-1)*u2r22*u2r33*ujc2y* \
    weightRB4+cg5x*u2r13*u2r31*weightRB5+cg5y*u2r23*u2r31* \
    weightRB5+(-1)*cg5x*u2r13*u2r32*weightRB5+(-1)*cg5y*u2r23* \
    u2r32*weightRB5+(-1)*cg5x*u2r11*u2r33*weightRB5+cg5x*u2r12* \
    u2r33*weightRB5+(-1)*cg5y*u2r21*u2r33*weightRB5+cg5y*u2r22* \
    u2r33*weightRB5+(-1)*u2r13*u2r31*ujc2x*weightRB5+u2r13* \
    u2r32*ujc2x*weightRB5+u2r11*u2r33*ujc2x*weightRB5+(-1)* \
    u2r12*u2r33*ujc2x*weightRB5+(-1)*u2r23*u2r31*ujc2y* \
    weightRB5+u2r23*u2r32*ujc2y*weightRB5+u2r21*u2r33*ujc2y* \
    weightRB5+(-1)*u2r22*u2r33*ujc2y*weightRB5+cg6x*u2r13* \
    u2r31*weightRB6+cg6y*u2r23*u2r31*weightRB6+(-1)*cg6x*u2r13* \
    u2r32*weightRB6+(-1)*cg6y*u2r23*u2r32*weightRB6+(-1)*cg6x* \
    u2r11*u2r33*weightRB6+cg6x*u2r12*u2r33*weightRB6+(-1)*cg6y* \
    u2r21*u2r33*weightRB6+cg6y*u2r22*u2r33*weightRB6+(-1)* \
    u2r13*u2r31*ujc2x*weightRB6+u2r13*u2r32*ujc2x*weightRB6+ \
    u2r11*u2r33*ujc2x*weightRB6+(-1)*u2r12*u2r33*ujc2x* \
    weightRB6+(-1)*u2r23*u2r31*ujc2y*weightRB6+u2r23*u2r32* \
    ujc2y*weightRB6+u2r21*u2r33*ujc2y*weightRB6+(-1)*u2r22* \
    u2r33*ujc2y*weightRB6+cg7x*u2r13*u2r31*weightRB7+cg7y* \
    u2r23*u2r31*weightRB7+(-1)*cg7x*u2r13*u2r32*weightRB7+(-1)* \
    cg7y*u2r23*u2r32*weightRB7+(-1)*cg7x*u2r11*u2r33*weightRB7+ \
    cg7x*u2r12*u2r33*weightRB7+(-1)*cg7y*u2r21*u2r33*weightRB7+ \
    cg7y*u2r22*u2r33*weightRB7+(-1)*u2r13*u2r31*ujc2x* \
    weightRB7+u2r13*u2r32*ujc2x*weightRB7+u2r11*u2r33*ujc2x* \
    weightRB7+(-1)*u2r12*u2r33*ujc2x*weightRB7+(-1)*u2r23* \
    u2r31*ujc2y*weightRB7+u2r23*u2r32*ujc2y*weightRB7+u2r21* \
    u2r33*ujc2y*weightRB7+(-1)*u2r22*u2r33*ujc2y*weightRB7)* \
    sec(theta4)*(AA2*cos(theta3)+(-1)*AO2*sin(theta3))**(-1)*( \
    AA2**2+AO2**2+JA2**2+(-2)*AO2*JA2*cos(theta3)+(-2)*AA2*JA2* \
    sin(theta3))**(1/2)*(6*AA2**2+6*AO2**2+(-1)*Bf8**2+(-6)*AO2* \
    JA2+6*JA2**2+6*(AA2**2+(AO2+(-1)*JA2)**2)**(1/2)*LActOffset8+ \
    3*LActOffset8**2+(-6)*(AA2**2+(AO2+(-1)*JA2)**2)**(1/2)* \
    LBase8+(-6)*LActOffset8*LBase8+3*LBase8**2+(-6)*AO2*JA2*cos( \
    theta3)+(-6)*AA2*JA2*sin(theta3)+(-6)*LActOffset8*(AA2**2+ \
    AO2**2+JA2**2+(-2)*AO2*JA2*cos(theta3)+(-2)*AA2*JA2*sin( \
    theta3))**(1/2)+6*LBase8*(AA2**2+AO2**2+JA2**2+(-2)*AO2*JA2* \
    cos(theta3)+(-2)*AA2*JA2*sin(theta3))**(1/2)+(-6)*((AA2**2+( \
    AO2+(-1)*JA2)**2)*(AA2**2+AO2**2+JA2**2+(-2)*AO2*JA2*cos( \
    theta3)+(-2)*AA2*JA2*sin(theta3)))**(1/2))**(-1)






    p8p6coef=Nf6**(-2)*Nf8**2*(AA2*cos(theta3)+(-1)*AO2*sin(theta3))**(-1) \
    *(AA2*cos(theta3)+AO2*sin(theta3))*(AA2**2+AO2**2+JA2**2+(-2) \
    *AO2*JA2*cos(theta3)+(-2)*AA2*JA2*sin(theta3))**(1/2)*( \
    AA2**2+AO2**2+JA2**2+(-2)*AO2*JA2*cos(theta3)+2*AA2*JA2*sin( \
    theta3))**(-1/2)*(6*AA2**2+6*AO2**2+(-1)*Bf8**2+(-6)*AO2* \
    JA2+6*JA2**2+6*(AA2**2+(AO2+(-1)*JA2)**2)**(1/2)*LActOffset8+ \
    3*LActOffset8**2+(-6)*(AA2**2+(AO2+(-1)*JA2)**2)**(1/2)* \
    LBase8+(-6)*LActOffset8*LBase8+3*LBase8**2+(-6)*AO2*JA2*cos( \
    theta3)+(-6)*AA2*JA2*sin(theta3)+(-6)*LActOffset8*(AA2**2+ \
    AO2**2+JA2**2+(-2)*AO2*JA2*cos(theta3)+(-2)*AA2*JA2*sin( \
    theta3))**(1/2)+6*LBase8*(AA2**2+AO2**2+JA2**2+(-2)*AO2*JA2* \
    cos(theta3)+(-2)*AA2*JA2*sin(theta3))**(1/2)+(-6)*((AA2**2+( \
    AO2+(-1)*JA2)**2)*(AA2**2+AO2**2+JA2**2+(-2)*AO2*JA2*cos( \
    theta3)+(-2)*AA2*JA2*sin(theta3)))**(1/2))**(-1)*(6*AA2**2+ \
    6*AO2**2+(-1)*Bf6**2+(-6)*AO2*JA2+6*JA2**2+6*(AA2**2+(AO2+( \
    -1)*JA2)**2)**(1/2)*LActOffset6+3*LActOffset6**2+(-6)*(AA2**2+ \
    (AO2+(-1)*JA2)**2)**(1/2)*LBase6+(-6)*LActOffset6*LBase6+3* \
    LBase6**2+(-6)*AO2*JA2*cos(theta3)+6*AA2*JA2*sin(theta3)+( \
    -6)*LActOffset6*(AA2**2+AO2**2+JA2**2+(-2)*AO2*JA2*cos( \
    theta3)+2*AA2*JA2*sin(theta3))**(1/2)+6*LBase6*(AA2**2+ \
    AO2**2+JA2**2+(-2)*AO2*JA2*cos(theta3)+2*AA2*JA2*sin(theta3) \
    )**(1/2)+(-6)*((AA2**2+(AO2+(-1)*JA2)**2)*(AA2**2+AO2**2+ \
    JA2**2+(-2)*AO2*JA2*cos(theta3)+2*AA2*JA2*sin(theta3)))**( \
    1/2))





    return p3p1const, p3p1coef, p4p2const, p4p2coef, p7p5const, p7p5coef, p8p6const, p8p6coef


def v2_SEG_2_compute_pressure_const_and_coef(param, Nfi, Bfi, theta, segBaseRU1, segBasePU1, segBaseRU2, segBasePU2, weights, ujcList, centerGravityList, LActOffsetList, LBaseList):


    [JA1,JA2,UC1,UC2,AA1,AA2,AO1,AO2,LL,JD] = param
    [Nf1, Nf2, Nf3, Nf4, Nf5, Nf6, Nf7, Nf8] = Nfi
    [Bf1, Bf2, Bf3, Bf4, Bf5, Bf6, Bf7, Bf8] = Bfi
    [theta1, theta2, theta3, theta4] = theta

    [[u1r11, u1r12, u1r13], [u1r21, u1r22, u1r23], [u1r31, u1r32, u1r33]] = segBaseRU1
    [u1sbpx, u1sbpy, u1sbpz] = segBasePU1

    [[u2r11, u2r12, u2r13], [u2r21, u2r22, u2r23], [u2r31, u2r32, u2r33]] = segBaseRU2
    [u2sbpx, u2sbpy, u2sbpz] = segBasePU2

    [weightRB1, weightRB2, weightRB3,weightRB4,weightRB5,weightRB6, weightRB7] = weights
    [ujc1x, ujc1y, ujc1z] = ujcList[0] 
    [ujc2x, ujc2y, ujc2z] = ujcList[1] 
    [ujc3x, ujc3y, ujc3z] = ujcList[2] 
    [ujc4x, ujc4y, ujc4z] = ujcList[3] 
    [ujc5x, ujc5y, ujc5z] = ujcList[4] 
    [ujc6x, ujc6y, ujc6z] = ujcList[5]

    [cg1x, cg1y, cg1z] = centerGravityList[0]
    [cg2x, cg2y, cg2z] = centerGravityList[1]
    [cg3x, cg3y, cg3z] = centerGravityList[2]
    [cg4x, cg4y, cg4z] = centerGravityList[3]
    [cg5x, cg5y, cg5z] = centerGravityList[4]
    [cg6x, cg6y, cg6z] = centerGravityList[5]
    [cg7x, cg7y, cg7z] = centerGravityList[6]

    [LActOffset1, LActOffset2, LActOffset3, LActOffset4, LActOffset5, LActOffset6, LActOffset7, LActOffset8] = LActOffsetList
    [LBase1, LBase2, LBase3, LBase4, LBase5, LBase6, LBase7, LBase8] = LBaseList


    p11p9const=(-4)*JA1**(-1)*Nf3**2*pi*(((-1)*cg4x*u1r13*u1r31+(-1)* \
    cg4y*u1r23*u1r31+cg4x*u1r11*u1r33+cg4y*u1r21*u1r33+u1r13* \
    u1r31*ujc3x+(-1)*u1r11*u1r33*ujc3x+u1r23*u1r31*ujc3y+(-1)* \
    u1r21*u1r33*ujc3y)*weightRB4+((-1)*cg5x*u1r13*u1r31+(-1)* \
    cg5y*u1r23*u1r31+cg5x*u1r11*u1r33+cg5y*u1r21*u1r33+u1r13* \
    u1r31*ujc3x+(-1)*u1r11*u1r33*ujc3x+u1r23*u1r31*ujc3y+(-1)* \
    u1r21*u1r33*ujc3y)*weightRB5+((-1)*cg6x*u1r13*u1r31+(-1)* \
    cg6y*u1r23*u1r31+cg6x*u1r11*u1r33+cg6y*u1r21*u1r33+u1r13* \
    u1r31*ujc3x+(-1)*u1r11*u1r33*ujc3x+u1r23*u1r31*ujc3y+(-1)* \
    u1r21*u1r33*ujc3y)*weightRB6+((-1)*cg7x*u1r13*u1r31+(-1)* \
    cg7y*u1r23*u1r31+cg7x*u1r11*u1r33+cg7y*u1r21*u1r33+u1r13* \
    u1r31*ujc3x+(-1)*u1r11*u1r33*ujc3x+u1r23*u1r31*ujc3y+(-1)* \
    u1r21*u1r33*ujc3y)*weightRB7)*sec(theta1)*(AA1*cos(theta2)+ \
    AO1*sin(theta2))**(-1)*(AA1**2+AO1**2+JA1**2+(-2)*AO1*JA1* \
    cos(theta2)+2*AA1*JA1*sin(theta2))**(1/2)*(Bf3**2+(-3)*(( \
    AA1**2+(AO1+(-1)*JA1)**2)**(1/2)+LActOffset3+(-1)*LBase3+(-1)*( \
    AA1**2+AO1**2+JA1**2+(-2)*AO1*JA1*cos(theta2)+2*AA1*JA1*sin( \
    theta2))**(1/2))**2)**(-1)






    p11p9coef=(-1)*Nf1**(-2)*Nf3**2*(AA1*cos(theta2)+(-1)*AO1*sin(theta2)) \
    *(AA1*cos(theta2)+AO1*sin(theta2))**(-1)*(AA1**2+AO1**2+ \
    JA1**2+(-2)*AO1*JA1*cos(theta2)+(-2)*AA1*JA1*sin(theta2))**( \
    -1/2)*(AA1**2+AO1**2+JA1**2+(-2)*AO1*JA1*cos(theta2)+2*AA1* \
    JA1*sin(theta2))**(1/2)*((-6)*AA1**2+(-6)*AO1**2+Bf1**2+6* \
    AO1*JA1+(-6)*JA1**2+(-6)*(AA1**2+(AO1+(-1)*JA1)**2)**(1/2)* \
    LActOffset1+(-3)*LActOffset1**2+6*(AA1**2+(AO1+(-1)*JA1)**2)**( \
    1/2)*LBase1+6*LActOffset1*LBase1+(-3)*LBase1**2+6*AO1*JA1* \
    cos(theta2)+6*AA1*JA1*sin(theta2)+6*LActOffset1*(AA1**2+ \
    AO1**2+JA1**2+(-2)*AO1*JA1*cos(theta2)+(-2)*AA1*JA1*sin( \
    theta2))**(1/2)+(-6)*LBase1*(AA1**2+AO1**2+JA1**2+(-2)*AO1* \
    JA1*cos(theta2)+(-2)*AA1*JA1*sin(theta2))**(1/2)+6*((AA1**2+( \
    AO1+(-1)*JA1)**2)*(AA1**2+AO1**2+JA1**2+(-2)*AO1*JA1*cos( \
    theta2)+(-2)*AA1*JA1*sin(theta2)))**(1/2))*(6*AA1**2+6* \
    AO1**2+(-1)*Bf3**2+(-6)*AO1*JA1+6*JA1**2+6*(AA1**2+(AO1+(-1) \
    *JA1)**2)**(1/2)*LActOffset3+3*LActOffset3**2+(-6)*(AA1**2+( \
    AO1+(-1)*JA1)**2)**(1/2)*LBase3+(-6)*LActOffset3*LBase3+3* \
    LBase3**2+(-6)*AO1*JA1*cos(theta2)+6*AA1*JA1*sin(theta2)+( \
    -6)*LActOffset3*(AA1**2+AO1**2+JA1**2+(-2)*AO1*JA1*cos( \
    theta2)+2*AA1*JA1*sin(theta2))**(1/2)+6*LBase3*(AA1**2+ \
    AO1**2+JA1**2+(-2)*AO1*JA1*cos(theta2)+2*AA1*JA1*sin(theta2) \
    )**(1/2)+(-6)*((AA1**2+(AO1+(-1)*JA1)**2)*(AA1**2+AO1**2+ \
    JA1**2+(-2)*AO1*JA1*cos(theta2)+2*AA1*JA1*sin(theta2)))**( \
    1/2))**(-1)






    p12p10const=4*JA1**(-1)*Nf4**2*pi*((cg4x*u1r13*u1r32+cg4y*u1r23*u1r32+ \
    (-1)*cg4x*u1r12*u1r33+(-1)*cg4y*u1r22*u1r33+(-1)*u1r13* \
    u1r32*ujc3x+u1r12*u1r33*ujc3x+(-1)*u1r23*u1r32*ujc3y+u1r22* \
    u1r33*ujc3y)*weightRB4+(cg5x*u1r13*u1r32+cg5y*u1r23*u1r32+( \
    -1)*cg5x*u1r12*u1r33+(-1)*cg5y*u1r22*u1r33+(-1)*u1r13* \
    u1r32*ujc3x+u1r12*u1r33*ujc3x+(-1)*u1r23*u1r32*ujc3y+u1r22* \
    u1r33*ujc3y)*weightRB5+(cg6x*u1r13*u1r32+cg6y*u1r23*u1r32+( \
    -1)*cg6x*u1r12*u1r33+(-1)*cg6y*u1r22*u1r33+(-1)*u1r13* \
    u1r32*ujc3x+u1r12*u1r33*ujc3x+(-1)*u1r23*u1r32*ujc3y+u1r22* \
    u1r33*ujc3y)*weightRB6+(cg7x*u1r13*u1r32+cg7y*u1r23*u1r32+( \
    -1)*cg7x*u1r12*u1r33+(-1)*cg7y*u1r22*u1r33+(-1)*u1r13* \
    u1r32*ujc3x+u1r12*u1r33*ujc3x+(-1)*u1r23*u1r32*ujc3y+u1r22* \
    u1r33*ujc3y)*weightRB7)*(AA1*cos(theta1)*cos(theta2)+(-1)* \
    AO1*sin(theta1))**(-1)*(AA1**2+AO1**2+JA1**2+(-2)*AO1*JA1* \
    cos(theta1)+(-2)*AA1*JA1*cos(theta2)*sin(theta1))**(1/2)*( \
    Bf4**2+(-3)*((AA1**2+(AO1+(-1)*JA1)**2)**(1/2)+LActOffset4+(-1) \
    *LBase4+(-1)*(AA1**2+AO1**2+JA1**2+(-2)*AO1*JA1*cos(theta1)+( \
    -2)*AA1*JA1*cos(theta2)*sin(theta1))**(1/2))**2)**(-1)






    p12p10coef=Nf2**(-2)*Nf4**2*(AA1*cos(theta1)*cos(theta2)+(-1)*AO1*sin( \
    theta1))**(-1)*(AA1*cos(theta1)*cos(theta2)+AO1*sin(theta1))* \
    (AA1**2+AO1**2+JA1**2+(-2)*AO1*JA1*cos(theta1)+(-2)*AA1*JA1* \
    cos(theta2)*sin(theta1))**(1/2)*(AA1**2+AO1**2+JA1**2+(-2)* \
    AO1*JA1*cos(theta1)+2*AA1*JA1*cos(theta2)*sin(theta1))**( \
    -1/2)*(6*AA1**2+6*AO1**2+(-1)*Bf4**2+(-6)*AO1*JA1+6*JA1**2+ \
    6*(AA1**2+(AO1+(-1)*JA1)**2)**(1/2)*LActOffset4+3* \
    LActOffset4**2+(-6)*(AA1**2+(AO1+(-1)*JA1)**2)**(1/2)*LBase4+( \
    -6)*LActOffset4*LBase4+3*LBase4**2+(-6)*AO1*JA1*cos(theta1)+ \
    (-6)*AA1*JA1*cos(theta2)*sin(theta1)+(-6)*LActOffset4*( \
    AA1**2+AO1**2+JA1**2+(-2)*AO1*JA1*cos(theta1)+(-2)*AA1*JA1* \
    cos(theta2)*sin(theta1))**(1/2)+6*LBase4*(AA1**2+AO1**2+JA1**2+ \
    (-2)*AO1*JA1*cos(theta1)+(-2)*AA1*JA1*cos(theta2)*sin( \
    theta1))**(1/2)+(-6)*((AA1**2+(AO1+(-1)*JA1)**2)*(AA1**2+ \
    AO1**2+JA1**2+(-2)*AO1*JA1*cos(theta1)+(-2)*AA1*JA1*cos( \
    theta2)*sin(theta1)))**(1/2))**(-1)*(6*AA1**2+6*AO1**2+(-1)* \
    Bf2**2+(-6)*AO1*JA1+6*JA1**2+6*(AA1**2+(AO1+(-1)*JA1)**2)**( \
    1/2)*LActOffset2+3*LActOffset2**2+(-6)*(AA1**2+(AO1+(-1)*JA1) \
    **2)**(1/2)*LBase2+(-6)*LActOffset2*LBase2+3*LBase2**2+(-6)* \
    AO1*JA1*cos(theta1)+6*AA1*JA1*cos(theta2)*sin(theta1)+(-6)* \
    LActOffset2*(AA1**2+AO1**2+JA1**2+(-2)*AO1*JA1*cos(theta1)+2* \
    AA1*JA1*cos(theta2)*sin(theta1))**(1/2)+6*LBase2*(AA1**2+ \
    AO1**2+JA1**2+(-2)*AO1*JA1*cos(theta1)+2*AA1*JA1*cos(theta2) \
    *sin(theta1))**(1/2)+(-6)*((AA1**2+(AO1+(-1)*JA1)**2)*(AA1**2+ \
    AO1**2+JA1**2+(-2)*AO1*JA1*cos(theta1)+2*AA1*JA1*cos(theta2) \
    *sin(theta1)))**(1/2))






    p15p13const=(-2)*2**(1/2)*JA2**(-1)*Nf7**2*pi*(cg5x*(u2r13*(u2r31+ \
    u2r32)+(-1)*(u2r11+u2r12)*u2r33)*weightRB5+cg5y*(u2r23*( \
    u2r31+u2r32)+(-1)*(u2r21+u2r22)*u2r33)*weightRB5+(-1)*u2r13* \
    u2r31*ujc4x*weightRB5+(-1)*u2r13*u2r32*ujc4x*weightRB5+ \
    u2r11*u2r33*ujc4x*weightRB5+u2r12*u2r33*ujc4x*weightRB5+(-1) \
    *u2r23*u2r31*ujc4y*weightRB5+(-1)*u2r23*u2r32*ujc4y* \
    weightRB5+u2r21*u2r33*ujc4y*weightRB5+u2r22*u2r33*ujc4y* \
    weightRB5+cg6x*u2r13*u2r31*weightRB6+cg6y*u2r23*u2r31* \
    weightRB6+cg6x*u2r13*u2r32*weightRB6+cg6y*u2r23*u2r32* \
    weightRB6+(-1)*cg6x*u2r11*u2r33*weightRB6+(-1)*cg6x*u2r12* \
    u2r33*weightRB6+(-1)*cg6y*u2r21*u2r33*weightRB6+(-1)*cg6y* \
    u2r22*u2r33*weightRB6+(-1)*u2r13*u2r31*ujc4x*weightRB6+(-1) \
    *u2r13*u2r32*ujc4x*weightRB6+u2r11*u2r33*ujc4x*weightRB6+ \
    u2r12*u2r33*ujc4x*weightRB6+(-1)*u2r23*u2r31*ujc4y* \
    weightRB6+(-1)*u2r23*u2r32*ujc4y*weightRB6+u2r21*u2r33* \
    ujc4y*weightRB6+u2r22*u2r33*ujc4y*weightRB6+cg7x*u2r13* \
    u2r31*weightRB7+cg7y*u2r23*u2r31*weightRB7+cg7x*u2r13* \
    u2r32*weightRB7+cg7y*u2r23*u2r32*weightRB7+(-1)*cg7x*u2r11* \
    u2r33*weightRB7+(-1)*cg7x*u2r12*u2r33*weightRB7+(-1)*cg7y* \
    u2r21*u2r33*weightRB7+(-1)*cg7y*u2r22*u2r33*weightRB7+(-1)* \
    u2r13*u2r31*ujc4x*weightRB7+(-1)*u2r13*u2r32*ujc4x* \
    weightRB7+u2r11*u2r33*ujc4x*weightRB7+u2r12*u2r33*ujc4x* \
    weightRB7+(-1)*u2r23*u2r31*ujc4y*weightRB7+(-1)*u2r23* \
    u2r32*ujc4y*weightRB7+u2r21*u2r33*ujc4y*weightRB7+u2r22* \
    u2r33*ujc4y*weightRB7)*sec(theta4)*(AA2**2+AO2**2+JA2**2+(-2) \
    *AO2*JA2*cos(theta4)+2*AA2*JA2*cos(theta3)*sin(theta4))**( \
    1/2)*(6*AA2**2+6*AO2**2+(-1)*Bf7**2+(-6)*AO2*JA2+6*JA2**2+ \
    6*(AA2**2+(AO2+(-1)*JA2)**2)**(1/2)*LActOffset7+3* \
    LActOffset7**2+(-6)*(AA2**2+(AO2+(-1)*JA2)**2)**(1/2)*LBase7+( \
    -6)*LActOffset7*LBase7+3*LBase7**2+(-6)*AO2*JA2*cos(theta4)+ \
    6*AA2*JA2*cos(theta3)*sin(theta4)+(-6)*LActOffset7*(AA2**2+ \
    AO2**2+JA2**2+(-2)*AO2*JA2*cos(theta4)+2*AA2*JA2*cos(theta3) \
    *sin(theta4))**(1/2)+6*LBase7*(AA2**2+AO2**2+JA2**2+(-2)*AO2* \
    JA2*cos(theta4)+2*AA2*JA2*cos(theta3)*sin(theta4))**(1/2)+( \
    -6)*((AA2**2+(AO2+(-1)*JA2)**2)*(AA2**2+AO2**2+JA2**2+(-2)* \
    AO2*JA2*cos(theta4)+2*AA2*JA2*cos(theta3)*sin(theta4)))**( \
    1/2))**(-1)*(AA2*cos(theta3)+AO2*tan(theta4))**(-1)






    p15p13coef=(-1)*Nf5**(-2)*Nf7**2*(AA2**2+AO2**2+JA2**2+(-2)*AO2*JA2* \
    cos(theta4)+(-2)*AA2*JA2*cos(theta3)*sin(theta4))**(-1/2)*( \
    AA2**2+AO2**2+JA2**2+(-2)*AO2*JA2*cos(theta4)+2*AA2*JA2*cos( \
    theta3)*sin(theta4))**(1/2)*((-6)*AA2**2+(-6)*AO2**2+Bf5**2+ \
    6*AO2*JA2+(-6)*JA2**2+(-6)*(AA2**2+(AO2+(-1)*JA2)**2)**(1/2) \
    *LActOffset5+(-3)*LActOffset5**2+6*(AA2**2+(AO2+(-1)*JA2)**2) \
    **(1/2)*LBase5+6*LActOffset5*LBase5+(-3)*LBase5**2+6*AO2* \
    JA2*cos(theta4)+6*AA2*JA2*cos(theta3)*sin(theta4)+6* \
    LActOffset5*(AA2**2+AO2**2+JA2**2+(-2)*AO2*JA2*cos(theta4)+( \
    -2)*AA2*JA2*cos(theta3)*sin(theta4))**(1/2)+(-6)*LBase5*( \
    AA2**2+AO2**2+JA2**2+(-2)*AO2*JA2*cos(theta4)+(-2)*AA2*JA2* \
    cos(theta3)*sin(theta4))**(1/2)+6*((AA2**2+(AO2+(-1)*JA2)**2)* \
    (AA2**2+AO2**2+JA2**2+(-2)*AO2*JA2*cos(theta4)+(-2)*AA2*JA2* \
    cos(theta3)*sin(theta4)))**(1/2))*(6*AA2**2+6*AO2**2+(-1)* \
    Bf7**2+(-6)*AO2*JA2+6*JA2**2+6*(AA2**2+(AO2+(-1)*JA2)**2)**( \
    1/2)*LActOffset7+3*LActOffset7**2+(-6)*(AA2**2+(AO2+(-1)*JA2) \
    **2)**(1/2)*LBase7+(-6)*LActOffset7*LBase7+3*LBase7**2+(-6)* \
    AO2*JA2*cos(theta4)+6*AA2*JA2*cos(theta3)*sin(theta4)+(-6)* \
    LActOffset7*(AA2**2+AO2**2+JA2**2+(-2)*AO2*JA2*cos(theta4)+2* \
    AA2*JA2*cos(theta3)*sin(theta4))**(1/2)+6*LBase7*(AA2**2+ \
    AO2**2+JA2**2+(-2)*AO2*JA2*cos(theta4)+2*AA2*JA2*cos(theta3) \
    *sin(theta4))**(1/2)+(-6)*((AA2**2+(AO2+(-1)*JA2)**2)*(AA2**2+ \
    AO2**2+JA2**2+(-2)*AO2*JA2*cos(theta4)+2*AA2*JA2*cos(theta3) \
    *sin(theta4)))**(1/2))**(-1)*(AA2*cos(theta3)+(-1)*AO2*tan( \
    theta4))*(AA2*cos(theta3)+AO2*tan(theta4))**(-1)






    p16p14const=2*2**(1/2)*JA2**(-1)*Nf8**2*pi*(cg5x*(u2r13*(u2r31+(-1)* \
    u2r32)+((-1)*u2r11+u2r12)*u2r33)*weightRB5+cg5y*(u2r23*u2r31+ \
    (-1)*u2r23*u2r32+(-1)*u2r21*u2r33+u2r22*u2r33)*weightRB5+( \
    -1)*u2r13*u2r31*ujc4x*weightRB5+u2r13*u2r32*ujc4x* \
    weightRB5+u2r11*u2r33*ujc4x*weightRB5+(-1)*u2r12*u2r33* \
    ujc4x*weightRB5+(-1)*u2r23*u2r31*ujc4y*weightRB5+u2r23* \
    u2r32*ujc4y*weightRB5+u2r21*u2r33*ujc4y*weightRB5+(-1)* \
    u2r22*u2r33*ujc4y*weightRB5+cg6x*u2r13*u2r31*weightRB6+ \
    cg6y*u2r23*u2r31*weightRB6+(-1)*cg6x*u2r13*u2r32*weightRB6+ \
    (-1)*cg6y*u2r23*u2r32*weightRB6+(-1)*cg6x*u2r11*u2r33* \
    weightRB6+cg6x*u2r12*u2r33*weightRB6+(-1)*cg6y*u2r21*u2r33* \
    weightRB6+cg6y*u2r22*u2r33*weightRB6+(-1)*u2r13*u2r31* \
    ujc4x*weightRB6+u2r13*u2r32*ujc4x*weightRB6+u2r11*u2r33* \
    ujc4x*weightRB6+(-1)*u2r12*u2r33*ujc4x*weightRB6+(-1)* \
    u2r23*u2r31*ujc4y*weightRB6+u2r23*u2r32*ujc4y*weightRB6+ \
    u2r21*u2r33*ujc4y*weightRB6+(-1)*u2r22*u2r33*ujc4y* \
    weightRB6+cg7x*u2r13*u2r31*weightRB7+cg7y*u2r23*u2r31* \
    weightRB7+(-1)*cg7x*u2r13*u2r32*weightRB7+(-1)*cg7y*u2r23* \
    u2r32*weightRB7+(-1)*cg7x*u2r11*u2r33*weightRB7+cg7x*u2r12* \
    u2r33*weightRB7+(-1)*cg7y*u2r21*u2r33*weightRB7+cg7y*u2r22* \
    u2r33*weightRB7+(-1)*u2r13*u2r31*ujc4x*weightRB7+u2r13* \
    u2r32*ujc4x*weightRB7+u2r11*u2r33*ujc4x*weightRB7+(-1)* \
    u2r12*u2r33*ujc4x*weightRB7+(-1)*u2r23*u2r31*ujc4y* \
    weightRB7+u2r23*u2r32*ujc4y*weightRB7+u2r21*u2r33*ujc4y* \
    weightRB7+(-1)*u2r22*u2r33*ujc4y*weightRB7)*sec(theta4)*( \
    AA2*cos(theta3)+(-1)*AO2*sin(theta3))**(-1)*(AA2**2+AO2**2+ \
    JA2**2+(-2)*AO2*JA2*cos(theta3)+(-2)*AA2*JA2*sin(theta3))**( \
    1/2)*(6*AA2**2+6*AO2**2+(-1)*Bf8**2+(-6)*AO2*JA2+6*JA2**2+ \
    6*(AA2**2+(AO2+(-1)*JA2)**2)**(1/2)*LActOffset8+3* \
    LActOffset8**2+(-6)*(AA2**2+(AO2+(-1)*JA2)**2)**(1/2)*LBase8+( \
    -6)*LActOffset8*LBase8+3*LBase8**2+(-6)*AO2*JA2*cos(theta3)+ \
    (-6)*AA2*JA2*sin(theta3)+(-6)*LActOffset8*(AA2**2+AO2**2+ \
    JA2**2+(-2)*AO2*JA2*cos(theta3)+(-2)*AA2*JA2*sin(theta3))**( \
    1/2)+6*LBase8*(AA2**2+AO2**2+JA2**2+(-2)*AO2*JA2*cos(theta3)+ \
    (-2)*AA2*JA2*sin(theta3))**(1/2)+(-6)*((AA2**2+(AO2+(-1)*JA2) \
    **2)*(AA2**2+AO2**2+JA2**2+(-2)*AO2*JA2*cos(theta3)+(-2)* \
    AA2*JA2*sin(theta3)))**(1/2))**(-1)






    p16p14coef=Nf6**(-2)*Nf8**2*(AA2*cos(theta3)+(-1)*AO2*sin(theta3))**(-1) \
    *(AA2*cos(theta3)+AO2*sin(theta3))*(AA2**2+AO2**2+JA2**2+(-2) \
    *AO2*JA2*cos(theta3)+(-2)*AA2*JA2*sin(theta3))**(1/2)*( \
    AA2**2+AO2**2+JA2**2+(-2)*AO2*JA2*cos(theta3)+2*AA2*JA2*sin( \
    theta3))**(-1/2)*(6*AA2**2+6*AO2**2+(-1)*Bf8**2+(-6)*AO2* \
    JA2+6*JA2**2+6*(AA2**2+(AO2+(-1)*JA2)**2)**(1/2)*LActOffset8+ \
    3*LActOffset8**2+(-6)*(AA2**2+(AO2+(-1)*JA2)**2)**(1/2)* \
    LBase8+(-6)*LActOffset8*LBase8+3*LBase8**2+(-6)*AO2*JA2*cos( \
    theta3)+(-6)*AA2*JA2*sin(theta3)+(-6)*LActOffset8*(AA2**2+ \
    AO2**2+JA2**2+(-2)*AO2*JA2*cos(theta3)+(-2)*AA2*JA2*sin( \
    theta3))**(1/2)+6*LBase8*(AA2**2+AO2**2+JA2**2+(-2)*AO2*JA2* \
    cos(theta3)+(-2)*AA2*JA2*sin(theta3))**(1/2)+(-6)*((AA2**2+( \
    AO2+(-1)*JA2)**2)*(AA2**2+AO2**2+JA2**2+(-2)*AO2*JA2*cos( \
    theta3)+(-2)*AA2*JA2*sin(theta3)))**(1/2))**(-1)*(6*AA2**2+ \
    6*AO2**2+(-1)*Bf6**2+(-6)*AO2*JA2+6*JA2**2+6*(AA2**2+(AO2+( \
    -1)*JA2)**2)**(1/2)*LActOffset6+3*LActOffset6**2+(-6)*(AA2**2+ \
    (AO2+(-1)*JA2)**2)**(1/2)*LBase6+(-6)*LActOffset6*LBase6+3* \
    LBase6**2+(-6)*AO2*JA2*cos(theta3)+6*AA2*JA2*sin(theta3)+( \
    -6)*LActOffset6*(AA2**2+AO2**2+JA2**2+(-2)*AO2*JA2*cos( \
    theta3)+2*AA2*JA2*sin(theta3))**(1/2)+6*LBase6*(AA2**2+ \
    AO2**2+JA2**2+(-2)*AO2*JA2*cos(theta3)+2*AA2*JA2*sin(theta3) \
    )**(1/2)+(-6)*((AA2**2+(AO2+(-1)*JA2)**2)*(AA2**2+AO2**2+ \
    JA2**2+(-2)*AO2*JA2*cos(theta3)+2*AA2*JA2*sin(theta3)))**( \
    1/2))


    return p11p9const, p11p9coef, p12p10const, p12p10coef, p15p13const, p15p13coef, p16p14const, p16p14coef


def v2_SEG_3_compute_pressure_const_and_coef(param, Nfi, Bfi, theta, segBaseRU1, segBasePU1, segBaseRU2, segBasePU2, weights, ujcList, centerGravityList, LActOffsetList, LBaseList):


    [JA1,JA2,UC1,UC2,AA1,AA2,AO1,AO2,LL,JD] = param
    [Nf1, Nf2, Nf3, Nf4, Nf5, Nf6, Nf7, Nf8] = Nfi
    [Bf1, Bf2, Bf3, Bf4, Bf5, Bf6, Bf7, Bf8] = Bfi
    [theta1, theta2, theta3, theta4] = theta

    [[u1r11, u1r12, u1r13], [u1r21, u1r22, u1r23], [u1r31, u1r32, u1r33]] = segBaseRU1
    [u1sbpx, u1sbpy, u1sbpz] = segBasePU1

    [[u2r11, u2r12, u2r13], [u2r21, u2r22, u2r23], [u2r31, u2r32, u2r33]] = segBaseRU2
    [u2sbpx, u2sbpy, u2sbpz] = segBasePU2

    [weightRB1, weightRB2, weightRB3,weightRB4,weightRB5,weightRB6, weightRB7] = weights
    [ujc1x, ujc1y, ujc1z] = ujcList[0] 
    [ujc2x, ujc2y, ujc2z] = ujcList[1] 
    [ujc3x, ujc3y, ujc3z] = ujcList[2] 
    [ujc4x, ujc4y, ujc4z] = ujcList[3] 
    [ujc5x, ujc5y, ujc5z] = ujcList[4] 
    [ujc6x, ujc6y, ujc6z] = ujcList[5]

    [cg1x, cg1y, cg1z] = centerGravityList[0]
    [cg2x, cg2y, cg2z] = centerGravityList[1]
    [cg3x, cg3y, cg3z] = centerGravityList[2]
    [cg4x, cg4y, cg4z] = centerGravityList[3]
    [cg5x, cg5y, cg5z] = centerGravityList[4]
    [cg6x, cg6y, cg6z] = centerGravityList[5]
    [cg7x, cg7y, cg7z] = centerGravityList[6]

    [LActOffset1, LActOffset2, LActOffset3, LActOffset4, LActOffset5, LActOffset6, LActOffset7, LActOffset8] = LActOffsetList
    [LBase1, LBase2, LBase3, LBase4, LBase5, LBase6, LBase7, LBase8] = LBaseList
    
    
    p19p17const=(-4)*JA1**(-1)*Nf3**2*pi*(((-1)*cg6x*u1r13*u1r31+(-1)* \
    cg6y*u1r23*u1r31+cg6x*u1r11*u1r33+cg6y*u1r21*u1r33+u1r13* \
    u1r31*ujc5x+(-1)*u1r11*u1r33*ujc5x+u1r23*u1r31*ujc5y+(-1)* \
    u1r21*u1r33*ujc5y)*weightRB6+((-1)*cg7x*u1r13*u1r31+(-1)* \
    cg7y*u1r23*u1r31+cg7x*u1r11*u1r33+cg7y*u1r21*u1r33+u1r13* \
    u1r31*ujc5x+(-1)*u1r11*u1r33*ujc5x+u1r23*u1r31*ujc5y+(-1)* \
    u1r21*u1r33*ujc5y)*weightRB7)*sec(theta1)*(AA1*cos(theta2)+ \
    AO1*sin(theta2))**(-1)*(AA1**2+AO1**2+JA1**2+(-2)*AO1*JA1* \
    cos(theta2)+2*AA1*JA1*sin(theta2))**(1/2)*(Bf3**2+(-3)*(( \
    AA1**2+(AO1+(-1)*JA1)**2)**(1/2)+LActOffset3+(-1)*LBase3+(-1)*( \
    AA1**2+AO1**2+JA1**2+(-2)*AO1*JA1*cos(theta2)+2*AA1*JA1*sin( \
    theta2))**(1/2))**2)**(-1)






    p19p17coef=(-1)*Nf1**(-2)*Nf3**2*(AA1*cos(theta2)+(-1)*AO1*sin(theta2)) \
    *(AA1*cos(theta2)+AO1*sin(theta2))**(-1)*(AA1**2+AO1**2+ \
    JA1**2+(-2)*AO1*JA1*cos(theta2)+(-2)*AA1*JA1*sin(theta2))**( \
    -1/2)*(AA1**2+AO1**2+JA1**2+(-2)*AO1*JA1*cos(theta2)+2*AA1* \
    JA1*sin(theta2))**(1/2)*((-6)*AA1**2+(-6)*AO1**2+Bf1**2+6* \
    AO1*JA1+(-6)*JA1**2+(-6)*(AA1**2+(AO1+(-1)*JA1)**2)**(1/2)* \
    LActOffset1+(-3)*LActOffset1**2+6*(AA1**2+(AO1+(-1)*JA1)**2)**( \
    1/2)*LBase1+6*LActOffset1*LBase1+(-3)*LBase1**2+6*AO1*JA1* \
    cos(theta2)+6*AA1*JA1*sin(theta2)+6*LActOffset1*(AA1**2+ \
    AO1**2+JA1**2+(-2)*AO1*JA1*cos(theta2)+(-2)*AA1*JA1*sin( \
    theta2))**(1/2)+(-6)*LBase1*(AA1**2+AO1**2+JA1**2+(-2)*AO1* \
    JA1*cos(theta2)+(-2)*AA1*JA1*sin(theta2))**(1/2)+6*((AA1**2+( \
    AO1+(-1)*JA1)**2)*(AA1**2+AO1**2+JA1**2+(-2)*AO1*JA1*cos( \
    theta2)+(-2)*AA1*JA1*sin(theta2)))**(1/2))*(6*AA1**2+6* \
    AO1**2+(-1)*Bf3**2+(-6)*AO1*JA1+6*JA1**2+6*(AA1**2+(AO1+(-1) \
    *JA1)**2)**(1/2)*LActOffset3+3*LActOffset3**2+(-6)*(AA1**2+( \
    AO1+(-1)*JA1)**2)**(1/2)*LBase3+(-6)*LActOffset3*LBase3+3* \
    LBase3**2+(-6)*AO1*JA1*cos(theta2)+6*AA1*JA1*sin(theta2)+( \
    -6)*LActOffset3*(AA1**2+AO1**2+JA1**2+(-2)*AO1*JA1*cos( \
    theta2)+2*AA1*JA1*sin(theta2))**(1/2)+6*LBase3*(AA1**2+ \
    AO1**2+JA1**2+(-2)*AO1*JA1*cos(theta2)+2*AA1*JA1*sin(theta2) \
    )**(1/2)+(-6)*((AA1**2+(AO1+(-1)*JA1)**2)*(AA1**2+AO1**2+ \
    JA1**2+(-2)*AO1*JA1*cos(theta2)+2*AA1*JA1*sin(theta2)))**( \
    1/2))**(-1)






    p20p18const=4*JA1**(-1)*Nf4**2*pi*((cg6x*u1r13*u1r32+cg6y*u1r23*u1r32+ \
    (-1)*cg6x*u1r12*u1r33+(-1)*cg6y*u1r22*u1r33+(-1)*u1r13* \
    u1r32*ujc5x+u1r12*u1r33*ujc5x+(-1)*u1r23*u1r32*ujc5y+u1r22* \
    u1r33*ujc5y)*weightRB6+(cg7x*u1r13*u1r32+cg7y*u1r23*u1r32+( \
    -1)*cg7x*u1r12*u1r33+(-1)*cg7y*u1r22*u1r33+(-1)*u1r13* \
    u1r32*ujc5x+u1r12*u1r33*ujc5x+(-1)*u1r23*u1r32*ujc5y+u1r22* \
    u1r33*ujc5y)*weightRB7)*(AA1*cos(theta1)*cos(theta2)+(-1)* \
    AO1*sin(theta1))**(-1)*(AA1**2+AO1**2+JA1**2+(-2)*AO1*JA1* \
    cos(theta1)+(-2)*AA1*JA1*cos(theta2)*sin(theta1))**(1/2)*( \
    Bf4**2+(-3)*((AA1**2+(AO1+(-1)*JA1)**2)**(1/2)+LActOffset4+(-1) \
    *LBase4+(-1)*(AA1**2+AO1**2+JA1**2+(-2)*AO1*JA1*cos(theta1)+( \
    -2)*AA1*JA1*cos(theta2)*sin(theta1))**(1/2))**2)**(-1)






    p20p18coef=Nf2**(-2)*Nf4**2*(AA1*cos(theta1)*cos(theta2)+(-1)*AO1*sin( \
    theta1))**(-1)*(AA1*cos(theta1)*cos(theta2)+AO1*sin(theta1))* \
    (AA1**2+AO1**2+JA1**2+(-2)*AO1*JA1*cos(theta1)+(-2)*AA1*JA1* \
    cos(theta2)*sin(theta1))**(1/2)*(AA1**2+AO1**2+JA1**2+(-2)* \
    AO1*JA1*cos(theta1)+2*AA1*JA1*cos(theta2)*sin(theta1))**( \
    -1/2)*(6*AA1**2+6*AO1**2+(-1)*Bf4**2+(-6)*AO1*JA1+6*JA1**2+ \
    6*(AA1**2+(AO1+(-1)*JA1)**2)**(1/2)*LActOffset4+3* \
    LActOffset4**2+(-6)*(AA1**2+(AO1+(-1)*JA1)**2)**(1/2)*LBase4+( \
    -6)*LActOffset4*LBase4+3*LBase4**2+(-6)*AO1*JA1*cos(theta1)+ \
    (-6)*AA1*JA1*cos(theta2)*sin(theta1)+(-6)*LActOffset4*( \
    AA1**2+AO1**2+JA1**2+(-2)*AO1*JA1*cos(theta1)+(-2)*AA1*JA1* \
    cos(theta2)*sin(theta1))**(1/2)+6*LBase4*(AA1**2+AO1**2+JA1**2+ \
    (-2)*AO1*JA1*cos(theta1)+(-2)*AA1*JA1*cos(theta2)*sin( \
    theta1))**(1/2)+(-6)*((AA1**2+(AO1+(-1)*JA1)**2)*(AA1**2+ \
    AO1**2+JA1**2+(-2)*AO1*JA1*cos(theta1)+(-2)*AA1*JA1*cos( \
    theta2)*sin(theta1)))**(1/2))**(-1)*(6*AA1**2+6*AO1**2+(-1)* \
    Bf2**2+(-6)*AO1*JA1+6*JA1**2+6*(AA1**2+(AO1+(-1)*JA1)**2)**( \
    1/2)*LActOffset2+3*LActOffset2**2+(-6)*(AA1**2+(AO1+(-1)*JA1) \
    **2)**(1/2)*LBase2+(-6)*LActOffset2*LBase2+3*LBase2**2+(-6)* \
    AO1*JA1*cos(theta1)+6*AA1*JA1*cos(theta2)*sin(theta1)+(-6)* \
    LActOffset2*(AA1**2+AO1**2+JA1**2+(-2)*AO1*JA1*cos(theta1)+2* \
    AA1*JA1*cos(theta2)*sin(theta1))**(1/2)+6*LBase2*(AA1**2+ \
    AO1**2+JA1**2+(-2)*AO1*JA1*cos(theta1)+2*AA1*JA1*cos(theta2) \
    *sin(theta1))**(1/2)+(-6)*((AA1**2+(AO1+(-1)*JA1)**2)*(AA1**2+ \
    AO1**2+JA1**2+(-2)*AO1*JA1*cos(theta1)+2*AA1*JA1*cos(theta2) \
    *sin(theta1)))**(1/2))






    p23p21const=(-2)*2**(1/2)*JA2**(-1)*Nf7**2*pi*(cg7x*u2r13*(u2r31+u2r32) \
    +cg7y*u2r23*(u2r31+u2r32)+(-1)*cg7x*(u2r11+u2r12)*u2r33+(-1) \
    *cg7y*(u2r21+u2r22)*u2r33+(-1)*u2r13*u2r31*ujc6x+(-1)* \
    u2r13*u2r32*ujc6x+u2r11*u2r33*ujc6x+u2r12*u2r33*ujc6x+(-1)* \
    u2r23*u2r31*ujc6y+(-1)*u2r23*u2r32*ujc6y+u2r21*u2r33*ujc6y+ \
    u2r22*u2r33*ujc6y)*weightRB7*sec(theta4)*(AA2**2+AO2**2+ \
    JA2**2+(-2)*AO2*JA2*cos(theta4)+2*AA2*JA2*cos(theta3)*sin( \
    theta4))**(1/2)*(6*AA2**2+6*AO2**2+(-1)*Bf7**2+(-6)*AO2*JA2+ \
    6*JA2**2+6*(AA2**2+(AO2+(-1)*JA2)**2)**(1/2)*LActOffset7+3* \
    LActOffset7**2+(-6)*(AA2**2+(AO2+(-1)*JA2)**2)**(1/2)*LBase7+( \
    -6)*LActOffset7*LBase7+3*LBase7**2+(-6)*AO2*JA2*cos(theta4)+ \
    6*AA2*JA2*cos(theta3)*sin(theta4)+(-6)*LActOffset7*(AA2**2+ \
    AO2**2+JA2**2+(-2)*AO2*JA2*cos(theta4)+2*AA2*JA2*cos(theta3) \
    *sin(theta4))**(1/2)+6*LBase7*(AA2**2+AO2**2+JA2**2+(-2)*AO2* \
    JA2*cos(theta4)+2*AA2*JA2*cos(theta3)*sin(theta4))**(1/2)+( \
    -6)*((AA2**2+(AO2+(-1)*JA2)**2)*(AA2**2+AO2**2+JA2**2+(-2)* \
    AO2*JA2*cos(theta4)+2*AA2*JA2*cos(theta3)*sin(theta4)))**( \
    1/2))**(-1)*(AA2*cos(theta3)+AO2*tan(theta4))**(-1)






    p23p21coef=(-1)*Nf5**(-2)*Nf7**2*(AA2**2+AO2**2+JA2**2+(-2)*AO2*JA2* \
    cos(theta4)+(-2)*AA2*JA2*cos(theta3)*sin(theta4))**(-1/2)*( \
    AA2**2+AO2**2+JA2**2+(-2)*AO2*JA2*cos(theta4)+2*AA2*JA2*cos( \
    theta3)*sin(theta4))**(1/2)*((-6)*AA2**2+(-6)*AO2**2+Bf5**2+ \
    6*AO2*JA2+(-6)*JA2**2+(-6)*(AA2**2+(AO2+(-1)*JA2)**2)**(1/2) \
    *LActOffset5+(-3)*LActOffset5**2+6*(AA2**2+(AO2+(-1)*JA2)**2) \
    **(1/2)*LBase5+6*LActOffset5*LBase5+(-3)*LBase5**2+6*AO2* \
    JA2*cos(theta4)+6*AA2*JA2*cos(theta3)*sin(theta4)+6* \
    LActOffset5*(AA2**2+AO2**2+JA2**2+(-2)*AO2*JA2*cos(theta4)+( \
    -2)*AA2*JA2*cos(theta3)*sin(theta4))**(1/2)+(-6)*LBase5*( \
    AA2**2+AO2**2+JA2**2+(-2)*AO2*JA2*cos(theta4)+(-2)*AA2*JA2* \
    cos(theta3)*sin(theta4))**(1/2)+6*((AA2**2+(AO2+(-1)*JA2)**2)* \
    (AA2**2+AO2**2+JA2**2+(-2)*AO2*JA2*cos(theta4)+(-2)*AA2*JA2* \
    cos(theta3)*sin(theta4)))**(1/2))*(6*AA2**2+6*AO2**2+(-1)* \
    Bf7**2+(-6)*AO2*JA2+6*JA2**2+6*(AA2**2+(AO2+(-1)*JA2)**2)**( \
    1/2)*LActOffset7+3*LActOffset7**2+(-6)*(AA2**2+(AO2+(-1)*JA2) \
    **2)**(1/2)*LBase7+(-6)*LActOffset7*LBase7+3*LBase7**2+(-6)* \
    AO2*JA2*cos(theta4)+6*AA2*JA2*cos(theta3)*sin(theta4)+(-6)* \
    LActOffset7*(AA2**2+AO2**2+JA2**2+(-2)*AO2*JA2*cos(theta4)+2* \
    AA2*JA2*cos(theta3)*sin(theta4))**(1/2)+6*LBase7*(AA2**2+ \
    AO2**2+JA2**2+(-2)*AO2*JA2*cos(theta4)+2*AA2*JA2*cos(theta3) \
    *sin(theta4))**(1/2)+(-6)*((AA2**2+(AO2+(-1)*JA2)**2)*(AA2**2+ \
    AO2**2+JA2**2+(-2)*AO2*JA2*cos(theta4)+2*AA2*JA2*cos(theta3) \
    *sin(theta4)))**(1/2))**(-1)*(AA2*cos(theta3)+(-1)*AO2*tan( \
    theta4))*(AA2*cos(theta3)+AO2*tan(theta4))**(-1)






    p24p22const=(-2)*2**(1/2)*JA2**(-1)*Nf8**2*pi*(cg7x*u2r13*(u2r31+(-1)* \
    u2r32)+cg7y*u2r23*(u2r31+(-1)*u2r32)+cg7x*((-1)*u2r11+u2r12) \
    *u2r33+cg7y*((-1)*u2r21+u2r22)*u2r33+(-1)*u2r13*u2r31* \
    ujc6x+u2r13*u2r32*ujc6x+u2r11*u2r33*ujc6x+(-1)*u2r12*u2r33* \
    ujc6x+(-1)*u2r23*u2r31*ujc6y+u2r23*u2r32*ujc6y+u2r21*u2r33* \
    ujc6y+(-1)*u2r22*u2r33*ujc6y)*weightRB7*sec(theta4)*(AA2* \
    cos(theta3)+(-1)*AO2*sin(theta3))**(-1)*(AA2**2+AO2**2+JA2**2+( \
    -2)*AO2*JA2*cos(theta3)+(-2)*AA2*JA2*sin(theta3))**(1/2)*(( \
    -6)*AA2**2+(-6)*AO2**2+Bf8**2+6*AO2*JA2+(-6)*JA2**2+(-6)*( \
    AA2**2+(AO2+(-1)*JA2)**2)**(1/2)*LActOffset8+(-3)* \
    LActOffset8**2+6*(AA2**2+(AO2+(-1)*JA2)**2)**(1/2)*LBase8+6* \
    LActOffset8*LBase8+(-3)*LBase8**2+6*AO2*JA2*cos(theta3)+6* \
    AA2*JA2*sin(theta3)+6*LActOffset8*(AA2**2+AO2**2+JA2**2+(-2)* \
    AO2*JA2*cos(theta3)+(-2)*AA2*JA2*sin(theta3))**(1/2)+(-6)* \
    LBase8*(AA2**2+AO2**2+JA2**2+(-2)*AO2*JA2*cos(theta3)+(-2)* \
    AA2*JA2*sin(theta3))**(1/2)+6*((AA2**2+(AO2+(-1)*JA2)**2)*( \
    AA2**2+AO2**2+JA2**2+(-2)*AO2*JA2*cos(theta3)+(-2)*AA2*JA2* \
    sin(theta3)))**(1/2))**(-1)






    p24p22coef=Nf6**(-2)*Nf8**2*(AA2*cos(theta3)+(-1)*AO2*sin(theta3))**(-1) \
    *(AA2*cos(theta3)+AO2*sin(theta3))*(AA2**2+AO2**2+JA2**2+(-2) \
    *AO2*JA2*cos(theta3)+(-2)*AA2*JA2*sin(theta3))**(1/2)*( \
    AA2**2+AO2**2+JA2**2+(-2)*AO2*JA2*cos(theta3)+2*AA2*JA2*sin( \
    theta3))**(-1/2)*(6*AA2**2+6*AO2**2+(-1)*Bf8**2+(-6)*AO2* \
    JA2+6*JA2**2+6*(AA2**2+(AO2+(-1)*JA2)**2)**(1/2)*LActOffset8+ \
    3*LActOffset8**2+(-6)*(AA2**2+(AO2+(-1)*JA2)**2)**(1/2)* \
    LBase8+(-6)*LActOffset8*LBase8+3*LBase8**2+(-6)*AO2*JA2*cos( \
    theta3)+(-6)*AA2*JA2*sin(theta3)+(-6)*LActOffset8*(AA2**2+ \
    AO2**2+JA2**2+(-2)*AO2*JA2*cos(theta3)+(-2)*AA2*JA2*sin( \
    theta3))**(1/2)+6*LBase8*(AA2**2+AO2**2+JA2**2+(-2)*AO2*JA2* \
    cos(theta3)+(-2)*AA2*JA2*sin(theta3))**(1/2)+(-6)*((AA2**2+( \
    AO2+(-1)*JA2)**2)*(AA2**2+AO2**2+JA2**2+(-2)*AO2*JA2*cos( \
    theta3)+(-2)*AA2*JA2*sin(theta3)))**(1/2))**(-1)*(6*AA2**2+ \
    6*AO2**2+(-1)*Bf6**2+(-6)*AO2*JA2+6*JA2**2+6*(AA2**2+(AO2+( \
    -1)*JA2)**2)**(1/2)*LActOffset6+3*LActOffset6**2+(-6)*(AA2**2+ \
    (AO2+(-1)*JA2)**2)**(1/2)*LBase6+(-6)*LActOffset6*LBase6+3* \
    LBase6**2+(-6)*AO2*JA2*cos(theta3)+6*AA2*JA2*sin(theta3)+( \
    -6)*LActOffset6*(AA2**2+AO2**2+JA2**2+(-2)*AO2*JA2*cos( \
    theta3)+2*AA2*JA2*sin(theta3))**(1/2)+6*LBase6*(AA2**2+ \
    AO2**2+JA2**2+(-2)*AO2*JA2*cos(theta3)+2*AA2*JA2*sin(theta3) \
    )**(1/2)+(-6)*((AA2**2+(AO2+(-1)*JA2)**2)*(AA2**2+AO2**2+ \
    JA2**2+(-2)*AO2*JA2*cos(theta3)+2*AA2*JA2*sin(theta3)))**( \
    1/2))






    return p19p17const, p19p17coef, p20p18const, p20p18coef, p23p21const, p23p21coef, p24p22const, p24p22coef

# write a small visualizer to test

def quasi_static_visualizer(q=np.zeros(12, dtype=float), initpos=np.eye(4)):
    fig = plt.figure()
    ax = plt.axes(projection='3d')

    # Set the starting perspective
    ax.view_init(elev=10, azim=225)

    ax.tick_params(axis='both', labelsize=8) # Adjust size for x, y, and z axes

    init_pos_link_0 = initpos

    theta1234 = q[0:4]
    [ee_pos,xx1,yy1,zz1,link_lines_collection1,bottom_actuator_lines_collection1,top_actuator_lines_collection1] = p5.print_link_color(rc.param_link0, theta1234, init_pos_link_0, ('blue', 1), 'lightcoral', 'lightcoral', 0.3)

    theta1234 = q[4:8]
    [ee_pos,xx2,yy2,zz2,link_lines_collection2,bottom_actuator_lines_collection2,top_actuator_lines_collection2] = p5.print_link_color(rc.param_link1, theta1234, ee_pos, ('blue', 1), 'lightcoral', 'lightcoral', 0.3)

    theta1234 = q[8:12]
    [ee_pos,xx3,yy3,zz3,link_lines_collection3,bottom_actuator_lines_collection3,top_actuator_lines_collection3] = p5.print_link_color(rc.param_link2, theta1234, ee_pos, ('blue', 1), 'lightcoral', 'lightcoral', 0.3)


    ax.add_collection3d(link_lines_collection1)
    ax.add_collection3d(bottom_actuator_lines_collection1)
    ax.add_collection3d(top_actuator_lines_collection1)

    ax.add_collection3d(link_lines_collection2)
    ax.add_collection3d(bottom_actuator_lines_collection2)
    ax.add_collection3d(top_actuator_lines_collection2)

    ax.add_collection3d(link_lines_collection3)
    ax.add_collection3d(bottom_actuator_lines_collection3)
    ax.add_collection3d(top_actuator_lines_collection3)

    x_all = np.stack([xx1,xx2,xx3])
    y_all = np.stack([yy1,yy2,yy3])
    z_all = np.stack([zz1,zz2,zz3])

    plot_nodes = ax.scatter(x_all,y_all,z_all, c=('coral', 0.5), s=3)


    # ------------
    # axis settings
    # ------------


    ax.set_xlim([-0.7, 0.7])
    ax.set_ylim([-0.7, 0.7])
    ax.set_zlim([0.3, 1.2])
    ax.set_aspect('equal')

    # Define the origin
    origin = [0, 0, 0]

    # Define the axis vectors
    x_axis = [1, 0, 0]
    y_axis = [0, 1, 0]
    z_axis = [0, 0, 1]

    # Plot the arrows
    ax.quiver(*origin, *x_axis, color='r', length=0.1, normalize=True, arrow_length_ratio=0.1)
    ax.quiver(*origin, *y_axis, color='g', length=0.1, normalize=True, arrow_length_ratio=0.1)
    ax.quiver(*origin, *z_axis, color='b', length=0.1, normalize=True, arrow_length_ratio=0.1)


    plt.show()

    return

if __name__ == "__main__":

    robot_base_test = np.eye(4)
    robot_base_test[0:3,3] = np.array([0.1,0.2,1.0])
    target_theta_test = np.ones(12)*(-0.3)
