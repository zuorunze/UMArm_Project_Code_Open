#Copyright © 2018 Naturalpoint
#
#Licensed under the Apache License, Version 2.0 (the "License")
#you may not use this file except in compliance with the License.
#You may obtain a copy of the License at
#
#http://www.apache.org/licenses/LICENSE-2.0
#
#Unless required by applicable law or agreed to in writing, software
#distributed under the License is distributed on an "AS IS" BASIS,
#WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#See the License for the specific language governing permissions and
#limitations under the License.


# OptiTrack NatNet direct depacketization sample for Python 3.x
#
# Uses the Python NatNetClient.py library to establish a connection (by creating a NatNetClient),
# and receive data via a NatNet connection and decode it using the NatNetClient library.

import sys
import time
from NatNetClient import NatNetClient
import DataDescriptions as DataDescriptions
import MoCapData
from scipy.spatial.transform import Rotation
import numpy as np

import kinematics_mp

import robot_constants as rc

# This is a callback function that gets connected to the NatNet client
# and called once per mocap frame.
def receive_new_frame(data_dict):
    global q_current, control_flags, jacobian_matrix_output

    try:
        # compute the robot configuration from the rigid bodies
        kinematics_mp.mocap_to_config_main(homogeneous_of_rigid_bodies, q_current)

        # compute the jacobian matrix of the current configuration, assuming the end effector is the rod
        # first, compute the forward kinematics

        # ee_tip_SE4 = kinematics_mp.fkine_mk5_with_rod_ee(rc.params, rc.my_end_effector_homo, q_current)
        # # then, use this ee_tip as the end effector position, compute the analytic jacobian
        # Jx = kinematics_mp.get_analytic_jacobian(rc.params, q_current, ee_tip_SE4)
        # # update the analytic jacobian using the multiprocess shared memory
        # jacobian_matrix_output[0] = Jx

        control_flags[rc.recorder_new_data] = 1
        control_flags[rc.compliance_new_data] = 1


        ##### NOTE: should test if this really can run at 120Hz.

        # for i in range(rc.num_rigid_bodies):
        #     posvec = homogeneous_of_rigid_bodies[i][0:3, 3]
        #     rc.print_formatted(posvec)

        # print("=========")


        # rc.print_formatted(q_current)
    except Exception as e:
        print("receive_new_frame: ", e)
    pass

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame(new_id, position, rotation):
    id_mask = 1000
    if new_id in range(id_mask + rc.num_rigid_bodies):
        rotation_matrix = Rotation.from_quat(rotation).as_matrix()
        # Set the top-left 3x3 part to the rotation matrix
        homogeneous_of_rigid_bodies[new_id-id_mask][:3, :3] = rotation_matrix
        # Set the top-right 3x1 part to the position vector
        homogeneous_of_rigid_bodies[new_id-id_mask][:3, 3] = position
        homogeneous_of_rigid_bodies[new_id-id_mask][3,3] = 1
    pass

def add_lists(totals, totals_tmp):
    totals[0]+=totals_tmp[0]
    totals[1]+=totals_tmp[1]
    totals[2]+=totals_tmp[2]
    return totals

def print_configuration(natnet_client):
    natnet_client.refresh_configuration()
    print("Connection Configuration:")
    print("  Client:          %s"% natnet_client.local_ip_address)
    print("  Server:          %s"% natnet_client.server_ip_address)
    print("  Command Port:    %d"% natnet_client.command_port)
    print("  Data Port:       %d"% natnet_client.data_port)

    changeBitstreamString = "  Can Change Bitstream Version = "
    if natnet_client.use_multicast:
        print("  Using Multicast")
        print("  Multicast Group: %s"% natnet_client.multicast_address)
        changeBitstreamString+="false"
    else:
        print("  Using Unicast")
        changeBitstreamString+="true"

    #NatNet Server Info
    application_name = natnet_client.get_application_name()
    nat_net_requested_version = natnet_client.get_nat_net_requested_version()
    nat_net_version_server = natnet_client.get_nat_net_version_server()
    server_version = natnet_client.get_server_version()

    print("  NatNet Server Info")
    print("    Application Name %s" %(application_name))
    print("    MotiveVersion  %d %d %d %d"% (server_version[0], server_version[1], server_version[2], server_version[3]))
    print("    NatNetVersion  %d %d %d %d"% (nat_net_version_server[0], nat_net_version_server[1], nat_net_version_server[2], nat_net_version_server[3]))
    print("  NatNet Bitstream Requested")
    print("    NatNetVersion  %d %d %d %d"% (nat_net_requested_version[0], nat_net_requested_version[1],\
       nat_net_requested_version[2], nat_net_requested_version[3]))

    print(changeBitstreamString)
    #print("command_socket = %s"%(str(natnet_client.command_socket)))
    #print("data_socket    = %s"%(str(natnet_client.data_socket)))
    print("  PythonVersion    %s"%(sys.version))


def print_commands(can_change_bitstream):
    outstring = "Commands:\n"
    outstring += "Return Data from Motive\n"
    outstring += "  s  send data descriptions\n"
    outstring += "  r  resume/start frame playback\n"
    outstring += "  p  pause frame playback\n"
    outstring += "     pause may require several seconds\n"
    outstring += "     depending on the frame data size\n"
    outstring += "Change Working Range\n"
    outstring += "  o  reset Working Range to: start/current/end frame 0/0/end of take\n"
    outstring += "  w  set Working Range to: start/current/end frame 1/100/1500\n"
    outstring += "Return Data Display Modes\n"
    outstring += "  j  print_level = 0 supress data description and mocap frame data\n"
    outstring += "  k  print_level = 1 show data description and mocap frame data\n"
    outstring += "  l  print_level = 20 show data description and every 20th mocap frame data\n"
    outstring += "Change NatNet data stream version (Unicast only)\n"
    outstring += "  3  Request NatNet 3.1 data stream (Unicast only)\n"
    outstring += "  4  Request NatNet 4.1 data stream (Unicast only)\n"
    outstring += "General\n"
    outstring += "  t  data structures self test (no motive/server interaction)\n"
    outstring += "  c  print configuration\n"
    outstring += "  h  print commands\n"
    outstring += "  q  quit\n"
    outstring += "\n"
    outstring += "NOTE: Motive frame playback will respond differently in\n"
    outstring += "       Endpoint, Loop, and Bounce playback modes.\n"
    outstring += "\n"
    outstring += "EXAMPLE: PacketClient [serverIP [ clientIP [ Multicast/Unicast]]]\n"
    outstring += "         PacketClient \"192.168.10.14\" \"192.168.10.14\" Multicast\n"
    outstring += "         PacketClient \"127.0.0.1\" \"127.0.0.1\" u\n"
    outstring += "\n"
    print(outstring)

def request_data_descriptions(s_client):
    # Request the model definitions
    s_client.send_request(s_client.command_socket, s_client.NAT_REQUEST_MODELDEF,    "",  (s_client.server_ip_address, s_client.command_port) )

def test_classes():
    totals = [0,0,0]
    print("Test Data Description Classes")
    totals_tmp = DataDescriptions.test_all()
    totals=add_lists(totals, totals_tmp)
    print("")
    print("Test MoCap Frame Classes")
    totals_tmp = MoCapData.test_all()
    totals=add_lists(totals, totals_tmp)
    print("")
    print("All Tests totals")
    print("--------------------")
    print("[PASS] Count = %3.1d"%totals[0])
    print("[FAIL] Count = %3.1d"%totals[1])
    print("[SKIP] Count = %3.1d"%totals[2])

def my_parse_args(arg_list, args_dict):
    # set up base values
    arg_list_len=len(arg_list)
    if arg_list_len>1:
        args_dict["serverAddress"] = arg_list[1]
        if arg_list_len>2:
            args_dict["clientAddress"] = arg_list[2]
        if arg_list_len>3:
            if len(arg_list[3]):
                args_dict["use_multicast"] = True
                if arg_list[3][0].upper() == "U":
                    args_dict["use_multicast"] = False

    return args_dict


def mocap_routine(mp_rigid_body_homo_list, mp_q, mp_jacobian_current, mp_control_flags):
    global homogeneous_of_rigid_bodies
    global q_current
    global control_flags
    global jacobian_matrix_output

    homogeneous_of_rigid_bodies = np.zeros([rc.num_rigid_bodies, 4, 4], dtype=float)
    q_current = mp_q
    control_flags = mp_control_flags
    jacobian_matrix_output = mp_jacobian_current

    optionsDict = {}
    optionsDict["clientAddress"] = "192.168.1.120"
    optionsDict["serverAddress"] = "192.168.1.100"
    optionsDict["use_multicast"] = True

    # This will create a new NatNet client
    optionsDict = my_parse_args(sys.argv, optionsDict)

    streaming_client = NatNetClient()
    streaming_client.set_client_address(optionsDict["clientAddress"])
    streaming_client.set_server_address(optionsDict["serverAddress"])
    streaming_client.set_use_multicast(optionsDict["use_multicast"])

    # Configure the streaming client to call our rigid body handler on the emulator to send data out.
    streaming_client.new_frame_listener = receive_new_frame
    streaming_client.rigid_body_listener = receive_rigid_body_frame

    # Start up the streaming client now that the callbacks are set up.
    # This will run perpetually, and operate on a separate thread.
    is_running = streaming_client.run()
    if not is_running:
        print("ERROR: Could not start streaming client.")
        try:
            sys.exit(1)
        except SystemExit:
            print("...")
        finally:
            print("exiting")

    is_looping = True
    time.sleep(1)
    if streaming_client.connected() is False:
        print("ERROR: Could not connect properly.  Check that Motive streaming is on.")
        try:
            sys.exit(2)
        except SystemExit:
            print("...")
        finally:
            print("exiting")

    while True:
        for i in range(rc.num_rigid_bodies):
            mp_rigid_body_homo_list[i] = homogeneous_of_rigid_bodies[i]
            # keep the routine running


def mocap_routine_main(mp_rigid_body_homo_list, mp_q, mp_jacobian_current, mp_control_flags):
    try:
        mocap_routine(mp_rigid_body_homo_list, mp_q, mp_jacobian_current, mp_control_flags)
    except Exception as e:
        print("mocap_routine_main: ", e)

if __name__ == "__main__":
    mocap_routine()