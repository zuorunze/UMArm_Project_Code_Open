#! /usr/bin/env python3

###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2018 Kinova inc. All rights reserved.
#
# This software may be modified and distributed
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

import sys
import os
import time
import threading
import utilities
import re

import numpy as np
from numpy import pi, cos, sin, tan
from scipy.spatial.transform import Rotation as R

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2

import kinematics_mp
import robot_constants as rc

# Maximum allowed waiting time during actions (in seconds)
TIMEOUT_DURATION = 20

# Create closure to set an event after an END or an ABORT
def check_for_end_or_abort(e):
    """Return a closure checking for END or ABORT notifications

    Arguments:
    e -- event to signal when the action is completed
        (will be set when an END or ABORT occurs)
    """
    def check(notification, e = e):
        print("EVENT : " + \
              Base_pb2.ActionEvent.Name(notification.action_event))
        if notification.action_event == Base_pb2.ACTION_END \
        or notification.action_event == Base_pb2.ACTION_ABORT:
            e.set()
    return check
 
def example_move_to_home_position(base):
    # Make sure the arm is in Single Level Servoing mode
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)
    
    # Move arm to ready position
    print("Moving the arm to a safe position")
    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
    action_list = base.ReadAllActions(action_type)
    action_handle = None
    for action in action_list.action_list:
        if action.name == "Home":
            action_handle = action.handle

    if action_handle == None:
        print("Can't reach safe position. Exiting")
        return False

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    base.ExecuteActionFromReference(action_handle)
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Safe position reached")
    else:
        print("Timeout on action notification wait")
    return finished

def example_angular_action_movement(base):
    
    print("Starting angular action movement ...")
    action = Base_pb2.Action()
    action.name = "Example angular action movement"
    action.application_data = ""

    actuator_count = base.GetActuatorCount()

    # Place arm straight up
    for joint_id in range(actuator_count.count):
        joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
        joint_angle.joint_identifier = joint_id
        joint_angle.value = 0

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )
    
    print("Executing action")
    base.ExecuteAction(action)

    print("Waiting for movement to finish ...")
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Angular movement completed")
    else:
        print("Timeout on action notification wait")
    return finished


def example_cartesian_action_movement(base, base_cyclic):
    
    print("Starting Cartesian action movement ...")
    action = Base_pb2.Action()
    action.name = "Example Cartesian action movement"
    action.application_data = ""

    feedback = base_cyclic.RefreshFeedback()

    cartesian_pose = action.reach_pose.target_pose
    cartesian_pose.x = feedback.base.tool_pose_x          # (meters)
    cartesian_pose.y = feedback.base.tool_pose_y - 0.1    # (meters)
    cartesian_pose.z = feedback.base.tool_pose_z - 0.2    # (meters)
    cartesian_pose.theta_x = feedback.base.tool_pose_theta_x # (degrees)
    cartesian_pose.theta_y = feedback.base.tool_pose_theta_y # (degrees)
    cartesian_pose.theta_z = feedback.base.tool_pose_theta_z # (degrees)


    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    print("Executing action")
    base.ExecuteAction(action)

    print("Waiting for movement to finish ...")
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Cartesian movement completed")
    else:
        print("Timeout on action notification wait")
    return finished


def console_cartesian_control_kinova_gen3(base, base_cyclic):
    print("Starting Console to Control Cartesian Movement")
    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    quit_all = False
    try:
        while not quit_all:
            inchars = input('Enter command or (\'h\' for list of commands)\n')
            if len(inchars)>0:
                c1 = inchars[0].lower()
                args = re.findall(r'[-+]?\d+\.?\d*', inchars)
                args = [float(num) if '.' in num else int(num) for num in args]
                # shutdown
                if c1 == 'q':
                    base.Unsubscribe(notification_handle)
                    quit_all = True
                
                if c1 == 'm': # manual control of cartesian movements
                    action = apply_delta_pose(args[0], args[1], args[2], args[3], args[4], args[5], base_cyclic)
                    print("Executing action")
                    base.ExecuteAction(action)
                    print("Waiting for movement to finish ...")
                    finished = e.wait(TIMEOUT_DURATION)
                    if finished:
                        print("Cartesian movement completed")
                    else:
                        print("Timeout on action notification wait")

                    pass
            

    except Exception as e:
        print("console: ", e)


def apply_delta_pose(dx,dy,dz,dtx,dty,dtz,base_cyclic):

    action = Base_pb2.Action()
    action.name = "Example Cartesian action movement"
    action.application_data = ""

    feedback = base_cyclic.RefreshFeedback()

    cartesian_pose = action.reach_pose.target_pose
    cartesian_pose.x = feedback.base.tool_pose_x + dx      # (meters)
    cartesian_pose.y = feedback.base.tool_pose_y + dy      # (meters)
    cartesian_pose.z = feedback.base.tool_pose_z + dz      # (meters)
    cartesian_pose.theta_x = feedback.base.tool_pose_theta_x + dtx # (degrees)
    cartesian_pose.theta_y = feedback.base.tool_pose_theta_y + dty # (degrees)
    cartesian_pose.theta_z = feedback.base.tool_pose_theta_z + dtz # (degrees)


    # tool_pose_as_SE4(cartesian_pose.x, cartesian_pose.y, cartesian_pose.z, cartesian_pose.theta_x, cartesian_pose.theta_y, cartesian_pose.theta_z)
    

    return action

def main():
    

    # Parse arguments
    args = utilities.parseConnectionArguments()
    
    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:

        # Create required services
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)

        # initialize
        example_move_to_home_position(base)
        
        console_cartesian_control_kinova_gen3(base, base_cyclic)


def tool_pose_as_SE4(x,y,z,tx,ty,tz):
    
    euler_angles = [tx, ty, tz]
    rotation_order = 'xyz' # Lowercase for extrinsic rotations, uppercase for intrinsic
    rotation = R.from_euler(rotation_order, euler_angles, degrees=True)
    matrix = rotation.as_matrix()

    new_SE4 = np.eye(4)
    new_SE4[0:3,0:3] = matrix
    new_SE4[0:3,3] = np.array([x,y,z])

    return new_SE4






def mp_kinova(mp_kinova_ee_kinova_frame, mp_kinova_move_spatial, mp_kinova_move_delta, mp_rigid_body_homo_list, mp_control_flags):
    try:
        # ================= initializing connection =================
        
        # Parse arguments
        args = utilities.parseConnectionArguments()

        kinova_homed = False
        
        # Create connection to the device and get the router
        with utilities.DeviceConnection.createTcpConnection(args) as router:

            # Create required services
            base = BaseClient(router)
            base_cyclic = BaseCyclicClient(router)

            


            # ================= Receive command and take action =================

            while True:
                # update current SE4 pose:
                feedback = base_cyclic.RefreshFeedback()
                kinova_x = feedback.base.tool_pose_x
                kinova_y = feedback.base.tool_pose_y
                kinova_z = feedback.base.tool_pose_z
                kinova_tx = feedback.base.tool_pose_theta_x
                kinova_ty = feedback.base.tool_pose_theta_y
                kinova_tz = feedback.base.tool_pose_theta_z

                kinova_ee_kinova_frame = tool_pose_as_SE4(kinova_x, kinova_y, kinova_z, kinova_tx, kinova_ty, kinova_tz) 
                mp_kinova_ee_kinova_frame[0] = kinova_ee_kinova_frame
                
                # spatial frame of the motion capture bracket on the end-effector of the kinova arm
                kinova_ee_mocap = mp_rigid_body_homo_list[8]
                # actual spatial_frame of the kinova arm ee
                kinova_ee_spatial_frame = kinova_ee_mocap @ rc.kinova_ee_frame_to_mocap_SE4
                # spatial frame of the base of the kinova arm
                kinova_base_spatial = kinova_ee_spatial_frame @ kinematics_mp.homo_inverse(kinova_ee_kinova_frame)


                if mp_control_flags[rc.kinova_enable] == 1:

                    if mp_control_flags[rc.kinova_move_home] == 1:
                        example_move_to_home_position(base)
                        mp_control_flags[rc.kinova_move_home] = 0
                        

                    elif mp_control_flags[rc.kinova_move_delta] == 1:
                        # generate the action item for the desired movement
                        [dx,dy,dz,dtx,dty,dtz] = mp_kinova_move_delta[0]

                        action = apply_delta_pose(dx,dy,dz,dtx,dty,dtz, base_cyclic)

                        e = threading.Event()
                        notification_handle = base.OnNotificationActionTopic(
                            check_for_end_or_abort(e),
                            Base_pb2.NotificationOptions()
                        )
                        # actuate the kinova arm
                        print("Executing action")
                        base.ExecuteAction(action)
                        print("Waiting for movement to finish ...")
                        finished = e.wait(TIMEOUT_DURATION)
                        base.Unsubscribe(notification_handle)

                        if finished:
                            print("Cartesian movement completed")
                        else:
                            print("Timeout on action notification wait")
                        
                        # reset the move delta flag
                        mp_control_flags[rc.kinova_move_delta] = 0
                        
                        
                    elif mp_control_flags[rc.kinova_move_spatial] == 1:
                        
                        # apply the desired movement in the spatial frame:
                        # pre-multiply means that apply the mp_kinova_move_spatial in spatial frame
                        target_SE4_spatial_frame = mp_kinova_move_spatial[0] @ kinova_ee_spatial_frame
                        # convert the target from spatial frame into kinova frame
                        target_SE4_kinova_frame = kinematics_mp.spatial_frame_to_robot_frame(target_SE4_spatial_frame, kinova_base_spatial)
                        
                        # convert this target frame from SE4 form to xyz dx dy dz form
                        rotm = target_SE4_kinova_frame[0:3,0:3]
                        tran = target_SE4_kinova_frame[0:3,3]
                        
                        rotm = R.from_matrix(rotm)
                        euler_angles = rotm.as_euler('xyz', degrees=True)
                        
                        # generate the action using these.
                        action = Base_pb2.Action()
                        action.name = "Example Cartesian action movement"
                        action.application_data = ""
                        
                        cartesian_pose = action.reach_pose.target_pose
                        cartesian_pose.x = tran[0]
                        cartesian_pose.y = tran[1]
                        cartesian_pose.z = tran[2]
                        cartesian_pose.theta_x = euler_angles[0]
                        cartesian_pose.theta_y = euler_angles[1]
                        cartesian_pose.theta_z = euler_angles[2]


                        e = threading.Event()
                        notification_handle = base.OnNotificationActionTopic(
                            check_for_end_or_abort(e),
                            Base_pb2.NotificationOptions()
                        )
                        # actuate the kinova arm
                        base.ExecuteAction(action)

                        print("Waiting for movement to finish ...")
                        finished = e.wait(TIMEOUT_DURATION)
                        base.Unsubscribe(notification_handle)


                        if finished:
                            print("Cartesian movement completed")
                        else:
                            print("Timeout on action notification wait")

                        # reset the flag
                        mp_control_flags[rc.kinova_move_spatial] = 0
                    
                    elif mp_control_flags[rc.kinova_move_toolframe] == 1:
                        pass

                else:
                    time.sleep(0.1)
    except Exception as e:
        print("mp_kinova: ", e)


if __name__ == "__main__":
    exit(main())