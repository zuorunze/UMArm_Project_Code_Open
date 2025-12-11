import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.tri import Triangulation


import numpy as np

import robot_constants as rc
import print_link_mk5 as p5
import kinematics_mp
import pressure_to_config
import forward_compliance

import time
import random

from numpy import sin, cos, tan, pi

class visualizer_umarm:
    mp_q = None
    mp_rigid_body_se4_list = None
    mp_pressure = None

    ani = None
    fig = None
    ax  = None
    
    params = None
    ee_SE4 = None
    base_SE4 = None
    

    def __init__(self, q_input, rigid_body_se4_list_input, actuator_pressure_input, params, ee_SE4, base_SE4):
        self.fig = plt.figure()
        self.ax =  self.fig.add_subplot(projection='3d')


        self.mp_q = q_input
        self.mp_rigid_body_se4_list = rigid_body_se4_list_input
        self.mp_pressure = actuator_pressure_input

        self.params = params
        self.ee_SE4 = ee_SE4
        self.base_SE4 = base_SE4



    def visualizer_umarm_main(self):
        
        # Set the starting perspective
        self.ax.view_init(elev=10, azim=180)  # Elevation: 30 degrees, Azimuth: 45 degrees

        self.ani = animation.FuncAnimation(self.fig, self.visualizer_umarm_update_from_config, fargs=(), interval=0.001, cache_frame_data=True,  save_count=240)

        plt.show()

        print("visualizer_umarm_main quitted")
        return
                

    def visualizer_umarm_update_from_config(self,dummy):

        
            
        init_pos_link_0 = (self.mp_rigid_body_se4_list)[0]

        theta1234 = (self.mp_q)[0:4]
        [ee_pos,xx1,yy1,zz1,link_lines_collection1,bottom_actuator_lines_collection1,top_actuator_lines_collection1, text_pos_1] = p5.print_link_mk7(self.params[0], theta1234, init_pos_link_0)

        theta1234 = (self.mp_q)[4:8]
        [ee_pos,xx2,yy2,zz2,link_lines_collection2,bottom_actuator_lines_collection2,top_actuator_lines_collection2, text_pos_2] = p5.print_link_mk7(self.params[1], theta1234, ee_pos)

        theta1234 = (self.mp_q)[8:12]
        [ee_pos,xx3,yy3,zz3,link_lines_collection3,bottom_actuator_lines_collection3,top_actuator_lines_collection3, text_pos_3] = p5.print_link_mk7(self.params[2], theta1234, ee_pos)


        self.ax.clear()

        self.ax.add_collection3d(link_lines_collection1)
        self.ax.add_collection3d(bottom_actuator_lines_collection1)
        self.ax.add_collection3d(top_actuator_lines_collection1)

        self.ax.add_collection3d(link_lines_collection2)
        self.ax.add_collection3d(bottom_actuator_lines_collection2)
        self.ax.add_collection3d(top_actuator_lines_collection2)

        self.ax.add_collection3d(link_lines_collection3)
        self.ax.add_collection3d(bottom_actuator_lines_collection3)
        self.ax.add_collection3d(top_actuator_lines_collection3)

        x_all = np.stack([xx1,xx2,xx3])
        y_all = np.stack([yy1,yy2,yy3])
        z_all = np.stack([zz1,zz2,zz3])

        plot_nodes = self.ax.scatter(x_all,y_all,z_all, c=('coral', 0.5), s=3)

        actuator_node_list = np.concatenate([text_pos_1, text_pos_2, text_pos_3])

        ee_SE4_spatial =  ee_pos @ self.ee_SE4
        self.ax.scatter(*ee_SE4_spatial[0:3,3], c='blue', s=15)

        # ------------
        # text annotation settings
        # ------------
        
        for i in range(rc.num_actuators):
            
            label = f"{(self.mp_pressure[i] / 6894.76):.2f}" # str(i) + "," + 
            pos = actuator_node_list[i,0:3,3]
            self.ax.text(*pos, label, fontsize=8)
            

        # ------------
        # axis settings
        # ------------

        self.ax.set_xlim([-0.3, 0.3])
        self.ax.set_ylim([-0.3, 0.3])
        self.ax.set_zlim([-0.3, 1.2])
        self.ax.set_aspect('equal')

        # Define the origin
        origin = [0, 0, 0]

        # Define the axis vectors
        x_axis = [1, 0, 0]
        y_axis = [0, 1, 0]
        z_axis = [0, 0, 1]

        # Plot the arrows
        self.ax.quiver(*origin, *x_axis, color='r', length=0.1, normalize=True, arrow_length_ratio=0.1)
        self.ax.quiver(*origin, *y_axis, color='g', length=0.1, normalize=True, arrow_length_ratio=0.1)
        self.ax.quiver(*origin, *z_axis, color='b', length=0.1, normalize=True, arrow_length_ratio=0.1)



    def static_arm_plot(self, q_vec, p_vec_list, robot_base_SE4, enable_comliance=False, optional_Cx=None, forcetest=False):
        self.ax.view_init(elev=17, azim=-140)

        init_pos_link_0 = robot_base_SE4

        theta1234 = q_vec[0:4]
        [ee_pos,xx1,yy1,zz1,link_lines_collection1,bottom_actuator_lines_collection1,top_actuator_lines_collection1, text_pos_1] = p5.print_link_mk7(self.params[0], theta1234, init_pos_link_0)

        theta1234 = q_vec[4:8]
        [ee_pos,xx2,yy2,zz2,link_lines_collection2,bottom_actuator_lines_collection2,top_actuator_lines_collection2, text_pos_2] = p5.print_link_mk7(self.params[1], theta1234, ee_pos)

        theta1234 = q_vec[8:12]
        [ee_pos,xx3,yy3,zz3,link_lines_collection3,bottom_actuator_lines_collection3,top_actuator_lines_collection3, text_pos_3] = p5.print_link_mk7(self.params[2], theta1234, ee_pos)

        self.ax.add_collection3d(link_lines_collection1)
        self.ax.add_collection3d(bottom_actuator_lines_collection1)
        self.ax.add_collection3d(top_actuator_lines_collection1)

        self.ax.add_collection3d(link_lines_collection2)
        self.ax.add_collection3d(bottom_actuator_lines_collection2)
        self.ax.add_collection3d(top_actuator_lines_collection2)

        self.ax.add_collection3d(link_lines_collection3)
        self.ax.add_collection3d(bottom_actuator_lines_collection3)
        self.ax.add_collection3d(top_actuator_lines_collection3)

        x_all = np.stack([xx1,xx2,xx3])
        y_all = np.stack([yy1,yy2,yy3])
        z_all = np.stack([zz1,zz2,zz3])

        plot_nodes = self.ax.scatter(x_all,y_all,z_all, c=('coral', 0.5), s=3)

        actuator_node_list = np.concatenate([text_pos_1, text_pos_2, text_pos_3])

        # ------------
        # draw ee
        ee_SE4_spatial =  ee_pos @ self.ee_SE4
        ee_SE4_robot = kinematics_mp.spatial_frame_to_robot_frame(ee_SE4_spatial, self.base_SE4)

        print("ee_SE4_robot", ee_SE4_robot)
        print("ee_SE4_spatial", ee_SE4_spatial)

        self.ax.scatter(*ee_SE4_spatial[0:3,3], c='blue', s=15)

        # Plot the arrows
        self.ax.quiver(*ee_SE4_spatial[0:3,3], *ee_SE4_spatial[0:3,0], color='r', length=0.1, normalize=True, arrow_length_ratio=0.1)
        self.ax.quiver(*ee_SE4_spatial[0:3,3], *ee_SE4_spatial[0:3,1], color='g', length=0.1, normalize=True, arrow_length_ratio=0.1)
        self.ax.quiver(*ee_SE4_spatial[0:3,3], *ee_SE4_spatial[0:3,2], color='b', length=0.1, normalize=True, arrow_length_ratio=0.1)

        
        # ------------
        # text annotation settings
        # ------------
        
        # for i in range(rc.num_actuators):
            
        #     label = str(i) + "," + f"{(p_vec[i] / 6894.76):.2f}"
        #     pos = actuator_node_list[i,0:3,3]
        #     self.ax.text(*pos, label, fontsize=6)
            

        # ------------
        # axis settings
        # ------------

        self.ax.set_xlim([-0.3, 0.3])
        self.ax.set_ylim([-0.3, 0.3])
        self.ax.set_zlim([-0.3, 1.2])
        self.ax.set_aspect('equal')

        # Define the origin
        origin = [0, 0, 0]

        # Define the axis vectors
        x_axis = [1, 0, 0]
        y_axis = [0, 1, 0]
        z_axis = [0, 0, 1]

        # Plot the arrows
        self.ax.quiver(*origin, *x_axis, color='r', length=0.1, normalize=True, arrow_length_ratio=0.1)
        self.ax.quiver(*origin, *y_axis, color='g', length=0.1, normalize=True, arrow_length_ratio=0.1)
        self.ax.quiver(*origin, *z_axis, color='b', length=0.1, normalize=True, arrow_length_ratio=0.1)

        # -------------------
        # compliance

        if enable_comliance is False:
            plt.show()
            return
        
        fcompl = forward_compliance.forward_compliance_solver(rc.params, rc.nlinks, rc.Bfis, rc.Nfis, rc.LBases, rc.LActOffsets, rc.my_end_effector_homo)

        color_list = ['purple', 'pink', 'coral']

        num_points = 400
        force_magnitude = 5

        for j in range(len(p_vec_list)):
            p_vec = p_vec_list[j,:]

            Cx = fcompl.get_task_space_compliance(q=q_vec, p=p_vec)
            # Cx = np.eye(3)
            # generate vector around end-effector using spherical coordinates
            # Create a grid in spherical coordinates
        
            fvec = fibonacci_sphere(num_points)

            fvec = fvec * force_magnitude

            # cvd: compliance virtual displacement
            cvd_vec = np.zeros((num_points, 3))
            cvd_pos = np.zeros((num_points, 3))
            cvd_pos_spatial = np.zeros((num_points, 3))
            
            for i in range(num_points):
                # NOTE: fvec should be given in robot frame!

                cvd_vec[i] = Cx @ fvec[i]
                cvd_pos[i] = cvd_vec[i] + ee_SE4_robot[0:3,3]


                cvd_pos_spatial[i] = (kinematics_mp.robot_frame_to_spatial_frame(np.array([*cvd_pos[i], 1]), self.base_SE4))[0:3]
                


            cvd_x = cvd_pos_spatial[:,0]
            cvd_y = cvd_pos_spatial[:,1]
            cvd_z = cvd_pos_spatial[:,2]

            self.ax.scatter(cvd_x, cvd_y, cvd_z, c=color_list[j], s=0.1)
            # self.ax.plot_trisurf(cvd_x, cvd_y, cvd_z, color=color_list[j], alpha=0.7)

        color_list = ['green', 'blue', 'red']

        if optional_Cx is not None:
            for j in range(len(optional_Cx)):
                Cx = optional_Cx[j]
                fvec = fibonacci_sphere(num_points)

                fvec = fvec * force_magnitude

                # cvd: compliance virtual displacement
                cvd_vec = np.zeros((num_points, 3))
                cvd_pos = np.zeros((num_points, 3))
                cvd_pos_spatial = np.zeros((num_points, 3))
                
                for i in range(num_points):
                    # NOTE: fvec should be given in robot frame!

                    cvd_vec[i] = Cx @ fvec[i]
                    cvd_pos[i] = cvd_vec[i] #  + ee_SE4_robot[0:3,3]

                    
                    cvd_pos_spatial[i] = (kinematics_mp.robot_frame_to_spatial_frame(np.array([*cvd_pos[i], 1]), ee_SE4_spatial))[0:3]
                    # print("cvd_pos", cvd_pos[i])
                    # print("cvd_pos_spatial", cvd_pos_spatial[i])


                cvd_x = cvd_pos_spatial[:,0]
                cvd_y = cvd_pos_spatial[:,1]
                cvd_z = cvd_pos_spatial[:,2]

                self.ax.scatter(cvd_x, cvd_y, cvd_z, c=color_list[j], s=0.1)

        # test some force vector and corresponding displacement
        if forcetest is True:
            Cx = optional_Cx[0]
            fvec = fibonacci_sphere(num_points)

            fvec = fvec * force_magnitude
            # cvd: compliance virtual displacement
            cvd_vec = np.zeros((num_points, 3))
            cvd_vec_spatial = np.zeros((num_points, 3))
            f_vec_spatial = np.zeros((num_points, 3))

            for i in range(num_points):
                # NOTE: fvec should be given in robot frame!

                cvd_vec[i] = Cx @ fvec[i]
                
                # get f_vec in spatial frame
                f_vec_spatial[i] = (kinematics_mp.robot_frame_to_spatial_frame(np.array([*fvec[i], 0]), self.base_SE4))[0:3]
                cvd_vec_spatial[i] = (kinematics_mp.robot_frame_to_spatial_frame(np.array([*cvd_vec[i], 0]), self.base_SE4))[0:3]

            # plot
            plot_idx_list = [105]
            
            for idx in plot_idx_list:
                self.ax.quiver(*ee_SE4_spatial[0:3,3], *f_vec_spatial[idx], color='blue', length=np.linalg.norm(f_vec_spatial[idx]), normalize=True, arrow_length_ratio=0.1)
                self.ax.quiver(*ee_SE4_spatial[0:3,3], *cvd_vec_spatial[idx], color='green', length=np.linalg.norm(cvd_vec_spatial[idx]), normalize=True, arrow_length_ratio=0.1)
                
        
        plt.show()





def visualization_mk7_main(mp_rbse4=[np.eye(4)]*8, mp_q=np.zeros(12,dtype=float), mp_pressure=np.zeros(rc.num_actuators, dtype=float)):
    try:
        vobj = visualizer_umarm(mp_q, mp_rbse4, mp_pressure, rc.params, rc.my_end_effector_homo, mp_rbse4[0])
        vobj.visualizer_umarm_main()
    except Exception as e:
        print("error in visualization_mk7_main: ", e)



def visualization_ee_main(mp_rigid_body_homo_list, mp_q_robot, mp_homo_desired, mp_task_compliance, mp_ft, mp_kinova_ee_current):
    try:
        visualize_animation(mp_rigid_body_homo_list, mp_q_robot, mp_homo_desired, mp_task_compliance, mp_ft, mp_kinova_ee_current)
    except Exception as e:
        print("error in visualization_main: ", e)

def visualize_animation(mp_rigid_body_homo_list, mp_q_robot, mp_homo_desired, mp_task_compliance, mp_ft, mp_kinova_ee_current):

    global fig
    global ax
    global rigid_body_homo
    global q_robot_current
    global desired_homo
    global task_compliance
    global force_torque_readout
    global kinova_ee_current

    global traj_actual, traj_target, traj_ikine, traj_tracing_size, traj_tracing_ptr

    q_robot_current = mp_q_robot

    desired_homo = mp_homo_desired

    task_compliance = mp_task_compliance

    force_torque_readout = mp_ft

    kinova_ee_current = mp_kinova_ee_current

    traj_tracing_size = 400
    traj_tracing_ptr = 0
    traj_actual = np.zeros((traj_tracing_size, 3), dtype=float)
    traj_target = np.zeros((traj_tracing_size, 3), dtype=float)
    traj_ikine =  np.zeros((traj_tracing_size, 3), dtype=float)

    fig = plt.figure()
    ax = plt.axes(projection='3d')
    rigid_body_homo = mp_rigid_body_homo_list

    # Set the starting perspective
    ax.view_init(elev=30, azim=150)  # Elevation: 30 degrees, Azimuth: 45 degrees

    ani = animation.FuncAnimation(fig, animation_visualize_arm_and_raw_mocap, fargs=(), interval=0.001, cache_frame_data=True,  save_count=240)
    plt.show()
    print("arm_visualizer_quitted")
    return


def animation_visualize_arm_and_raw_mocap(dummy):
    global traj_actual, traj_target, traj_ikine, traj_tracing_size, traj_tracing_ptr
    global kinova_ee_current

    try:   

        # rc.print_formatted(rigid_body_homo[6][0:3,3])

        # print("-------")
        # for i in range(1,7):
        #     print(np.linalg.norm(rigid_body_homo[i][0:3,3] - rigid_body_homo[i-1][0:3,3]))
        #     # rc.print_formatted(rigid_body_homo[i][0:3,3])

        # rc.print_formatted(rigid_body_homo[6][0:3,3])
        # rc.print_formatted(q_robot_current)
        
        
        ax.clear()

        pos_current = np.zeros((rc.num_rigid_bodies,3))

        for i in range(rc.num_rigid_bodies):
            pos_current[i, 0:3] = rigid_body_homo[i][0:3,3]

        x_mocap = pos_current[0:6,0]
        y_mocap = pos_current[0:6,1]
        z_mocap = pos_current[0:6,2]
        
        ax.scatter(x_mocap, y_mocap, z_mocap, c='r', s=25)

        x_mocap = pos_current[6,0]
        y_mocap = pos_current[6,1]
        z_mocap = pos_current[6,2]
        
        ax.scatter(x_mocap, y_mocap, z_mocap, c='b', s=25)

        
        init_pos_link_0 = rigid_body_homo[0][:,:]
        
        theta1234 = q_robot_current[0:4]
        [ee_pos,xx,yy,zz,link_lines_collection,bottom_actuator_lines_collection,top_actuator_lines_collection] = p5.print_link(rc.param_link0, theta1234, init_pos_link_0)

        ax.scatter(xx,yy,zz, c='b', s=5)
        ax.add_collection3d(link_lines_collection)
        ax.add_collection3d(bottom_actuator_lines_collection)
        ax.add_collection3d(top_actuator_lines_collection)

        theta1234 = q_robot_current[4:8]
        [ee_pos,xx,yy,zz,link_lines_collection,bottom_actuator_lines_collection,top_actuator_lines_collection] = p5.print_link(rc.param_link1, theta1234, ee_pos)

        ax.scatter(xx,yy,zz, c='b', s=5)
        ax.add_collection3d(link_lines_collection)
        ax.add_collection3d(bottom_actuator_lines_collection)
        ax.add_collection3d(top_actuator_lines_collection)

        theta1234 = q_robot_current[8:12]
        [ee_pos,xx,yy,zz,link_lines_collection,bottom_actuator_lines_collection,top_actuator_lines_collection] = p5.print_link(rc.param_link2, theta1234, ee_pos)

        ax.scatter(xx,yy,zz, c='b', s=5)
        ax.add_collection3d(link_lines_collection)
        ax.add_collection3d(bottom_actuator_lines_collection)
        ax.add_collection3d(top_actuator_lines_collection)
        
        # plot the deisred ee position in homogeneous form
        ax.scatter(*desired_homo[0][0:3,3], c='b', s=7)

        ee_tip_SE4_robot_frame = kinematics_mp.fkine_mk5_with_rod_ee(rc.params, rc.my_end_effector_homo, q_robot_current)
        ee_tip_SE4 = kinematics_mp.robot_frame_to_spatial_frame(ee_tip_SE4_robot_frame, init_pos_link_0)

        #ee
        ax.scatter(*ee_tip_SE4[0:3,3], c='g', s=7)

        #wand
        ax.scatter(*rigid_body_homo[7][0:3,3], c='yellow', s=5)


        #kinova
        ax.scatter(*rigid_body_homo[8][0:3,3], c='purple', s=8)
        ax.quiver(*rigid_body_homo[8][0:3,3], *rigid_body_homo[8][0:3,0], color='r', length=0.1, normalize=True, arrow_length_ratio=0.1)
        ax.quiver(*rigid_body_homo[8][0:3,3], *rigid_body_homo[8][0:3,1], color='g', length=0.1, normalize=True, arrow_length_ratio=0.1)
        ax.quiver(*rigid_body_homo[8][0:3,3], *rigid_body_homo[8][0:3,2], color='b', length=0.1, normalize=True, arrow_length_ratio=0.1)
        


        #compute the base frame of kinova arm
        

        #kinova_feedback_spatial
        kinova_mocap_spatial_frame = rigid_body_homo[8]
        #derive the spatial frame of kinova ee from the mocap frame
        kinova_ee_spatial = kinova_mocap_spatial_frame @ rc.kinova_ee_frame_to_mocap_SE4

        kinova_base_spatial = kinova_ee_spatial @ kinematics_mp.homo_inverse(kinova_ee_current[0])

        # kinova_ee_kinova_frame = kinova_ee_current[0]
        # kinematics_mp.robot_frame_to_spatial_frame()

        ax.scatter(*kinova_ee_spatial[0:3,3], c='purple', s=8)
        ax.quiver(*kinova_ee_spatial[0:3,3], *kinova_ee_spatial[0:3,0], color='yellow', length=0.1, normalize=True, arrow_length_ratio=0.1)
        ax.quiver(*kinova_ee_spatial[0:3,3], *kinova_ee_spatial[0:3,1], color='purple', length=0.1, normalize=True, arrow_length_ratio=0.1)
        ax.quiver(*kinova_ee_spatial[0:3,3], *kinova_ee_spatial[0:3,2], color='black', length=0.1, normalize=True, arrow_length_ratio=0.1)


        ax.scatter(*kinova_base_spatial[0:3,3], c='purple', s=8)
        ax.quiver(*kinova_base_spatial[0:3,3], *kinova_base_spatial[0:3,0], color='red', length=0.1, normalize=True, arrow_length_ratio=0.1)
        ax.quiver(*kinova_base_spatial[0:3,3], *kinova_base_spatial[0:3,1], color='orange', length=0.1, normalize=True, arrow_length_ratio=0.1)
        ax.quiver(*kinova_base_spatial[0:3,3], *kinova_base_spatial[0:3,2], color='yellow', length=0.1, normalize=True, arrow_length_ratio=0.1)

        
        enable_traj_tracing = False
        

        if enable_traj_tracing is True:
            traj_actual[traj_tracing_ptr, :] = rigid_body_homo[6][0:3,3]
            traj_target[traj_tracing_ptr, :] = desired_homo[0][0:3,3]
            traj_ikine[traj_tracing_ptr, :] = ee_tip_SE4[0:3,3]

            traj_tracing_ptr += 1
            if traj_tracing_ptr >= traj_tracing_size:
                traj_tracing_ptr = 0
            
            # plot the trajectory
            ax.scatter(traj_actual[:,0], traj_actual[:,1], traj_actual[:,2], c='b', s=4)
            ax.scatter(traj_target[:,0], traj_target[:,1], traj_target[:,2], c='r', s=4)
            ax.scatter(traj_ikine[:,0], traj_ikine[:,1], traj_ikine[:,2], c='g', s=4)
        

        # print("--------------")
        # rc.print_formatted(ee_tip_SE4[0:3,3])
        # rc.print_formatted(rigid_body_homo[0][0:3,3])
        # print("--------------")

        #==========visualizing force=========#
        enable_force = True

        if enable_force is True:

            wand_shift = np.array([
                [1,0,0,0],
                [0,1,0,0],
                [0,0,1,0.017],
                [0,0,0,1]
            ], dtype=float)


            # force_base_SE4 = rigid_body_homo[6]
            tool_frame_mount = rigid_body_homo[5]

            # shift_z_from_force_base = np.array([
            #     [1,0,0,0],
            #     [0,1,0,0],
            #     [0,0,1,-0.028],
            #     [0,0,0,1]
            # ], dtype=float)

            shift_z_from_tool_frame = np.array([
                [1,0,0,0],
                [0,1,0,0],
                [0,0,1,-0.028+rc.my_end_effector_homo[2][3]],
                [0,0,0,1]
            ], dtype=float)

            # R_force_base_to_sensor = np.array([
            #     [cos(pi), 0, sin(pi),   0],
            #     [0,       1,    0,      0],
            #     [-sin(pi), 0, cos(pi),  0],
            #     [0,0,0,1]
            # ], dtype=float)


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

            # post multiply so it shift along the z axis of ee_tip
            force_origin = (tool_frame_mount @ shift_z_from_tool_frame)[0:3,3]

            # wand_vec = wand_origin - force_origin
            
            # Combined_matrix is the sensor frame
            sensor_frame =  tool_frame_mount @ shift_z_from_tool_frame @ R_tool_frame_to_sensor_y180 @ R_tool_frame_to_sensor_zn90

            # need to convert the force vector into world frame
            roboframe_force_vector = np.array([force_torque_readout[0], force_torque_readout[1], force_torque_readout[2], 0])

            spatial_force_vector = kinematics_mp.robot_frame_to_spatial_frame(roboframe_force_vector, sensor_frame)

            arrow_length = np.linalg.norm(roboframe_force_vector[0:3]) * 0.05

            ax.quiver(*force_origin, *(spatial_force_vector[0:3]), color='g', length=arrow_length, normalize=True, arrow_length_ratio=0.4)


            # display the force amplitude
            label = f"{(np.linalg.norm(spatial_force_vector[0:3])):.4f}" # str(i) + "," + 
            pos = np.array([init_pos_link_0[0,3], init_pos_link_0[1,3], init_pos_link_0[2,3] - 1], dtype=float)
            ax.text(*pos, label, fontsize=8)


            # ax.quiver(*force_origin, *(wand_vec[0:3]), color='r', length=np.linalg.norm(wand_vec), normalize=True, arrow_length_ratio=0.4)

            # ax.quiver(*tool_frame_mount[0:3,3], *tool_frame_mount[0:3,0], color='r', length=0.1, normalize=True, arrow_length_ratio=0.1)
            # ax.quiver(*tool_frame_mount[0:3,3], *tool_frame_mount[0:3,1], color='g', length=0.1, normalize=True, arrow_length_ratio=0.1)
            # ax.quiver(*tool_frame_mount[0:3,3], *tool_frame_mount[0:3,2], color='b', length=0.1, normalize=True, arrow_length_ratio=0.1)


            ax.quiver(*sensor_frame[0:3,3], *sensor_frame[0:3,0], color='r', length=0.1, normalize=True, arrow_length_ratio=0.1)
            ax.quiver(*sensor_frame[0:3,3], *sensor_frame[0:3,1], color='g', length=0.1, normalize=True, arrow_length_ratio=0.1)
            ax.quiver(*sensor_frame[0:3,3], *sensor_frame[0:3,2], color='b', length=0.1, normalize=True, arrow_length_ratio=0.1)


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

            kinova_ee_target = sensor_frame @ R_x90 @ R_z180 @ T_z10cm

            # ax.quiver(*kinova_ee_target[0:3,3], *kinova_ee_target[0:3,0], color='r', length=0.1, normalize=True, arrow_length_ratio=0.1)
            # ax.quiver(*kinova_ee_target[0:3,3], *kinova_ee_target[0:3,1], color='g', length=0.1, normalize=True, arrow_length_ratio=0.1)
            # ax.quiver(*kinova_ee_target[0:3,3], *kinova_ee_target[0:3,2], color='b', length=0.1, normalize=True, arrow_length_ratio=0.1)


        #===============COMPLIANCE ELLIPSE================
        enable_compl = False

        if enable_compl is True:
            Cx = task_compliance[0]
            # print(Cx)


            # generate vector around end-effector using spherical coordinates
            # Create a grid in spherical coordinates
            num_points = 150

            force_magnitude = 2

            fvec = fibonacci_sphere(num_points)

            fvec = fvec * force_magnitude

            # cvd: compliance virtual displacement
            cvd_vec = np.zeros((num_points, 3))
            cvd_pos = np.zeros((num_points, 3))
            cvd_pos_spatial = np.zeros((num_points, 3))
            
            for i in range(num_points):
                # NOTE: fvec should be given in robot frame!

                cvd_vec[i] = Cx @ fvec[i]
                cvd_pos[i] = cvd_vec[i] + ee_tip_SE4_robot_frame[0:3,3]


                cvd_pos_spatial[i] = (kinematics_mp.robot_frame_to_spatial_frame(np.array([*cvd_pos[i], 1]), rigid_body_homo[0]))[0:3]
                
            

            ax.scatter(cvd_pos_spatial[:,0], cvd_pos_spatial[:,1], cvd_pos_spatial[:,2], c='purple', s=7)

        #===============COMPLIANCE ELLIPSE END================


        ax.set_xlim([-0.4 + init_pos_link_0[0,3], 0.4 + init_pos_link_0[0,3]])
        ax.set_ylim([-0.4 + init_pos_link_0[1,3], 0.4 + init_pos_link_0[1,3]])
        ax.set_zlim([init_pos_link_0[2,3] - 1,   + init_pos_link_0[2,3]])
        ax.set_aspect('equal')

        # Define the origin
        origin = [init_pos_link_0[0,3], init_pos_link_0[1,3], init_pos_link_0[2,3] - 1]

        # Define the axis vectors
        x_axis = [1, 0, 0]
        y_axis = [0, 1, 0]
        z_axis = [0, 0, 1]

        # Plot the arrows
        ax.quiver(*origin, *x_axis, color='r', length=0.1, normalize=True, arrow_length_ratio=0.1)
        ax.quiver(*origin, *y_axis, color='g', length=0.1, normalize=True, arrow_length_ratio=0.1)
        ax.quiver(*origin, *z_axis, color='b', length=0.1, normalize=True, arrow_length_ratio=0.1)

    except Exception as e:
        print("ani: ", e)

    return

def fibonacci_sphere(n):
    points = np.zeros((n, 3))
    phi = np.pi * (3. - np.sqrt(5))  # golden angle

    for i in range(n):
        y = 1 - (i / float(n - 1)) * 2  # y goes from 1 to -1
        radius = np.sqrt(1 - y * y)     # radius at y

        theta = phi * i  # golden angle increment

        x = np.cos(theta) * radius
        z = np.sin(theta) * radius

        points[i] = [x, y, z]

    return points




if __name__ == "__main__":
    
    vobj = visualizer_umarm(np.zeros(12), [np.eye(4)], np.arange(24))
    vobj.visualizer_umarm_main()

   

        

