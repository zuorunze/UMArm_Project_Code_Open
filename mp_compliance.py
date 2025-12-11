import numpy as np

import forward_compliance
import joint_compliance_matrix
import robot_constants as rc

def mp_compliance_main(mp_q_robot, mp_p_actuator, mp_jacobian_current, mp_task_compliance, mp_control_flags):
    try:
        while True:
            # if there is new frame, do the computation (120hz)
            if mp_control_flags[rc.compliance_new_data] == 1:
                # set the new frame flag to 0 for next read.
                mp_control_flags[rc.compliance_new_data] = 0
                # get the current q:
                q_curr = np.zeros((rc.num_joints), dtype=float)
                for i in range(rc.num_joints):
                    q_curr[i] = mp_q_robot[i]

                # get the current p:
                p_curr = np.zeros((rc.num_actuators), dtype=float)
                for i in range(rc.num_actuators):
                    p_curr[i] = mp_p_actuator[i]

                # first, get the current jacobian
                Jx = mp_jacobian_current[0]


                # then, compute the configuration space compliance matrix
                Cq = joint_compliance_matrix.get_joint_space_compliance_matrix(
                    nlinks=rc.nlinks, params=rc.params, 
                    Nfis=rc.Nfis, Bfis=rc.Bfis, LBases=rc.LBases, LActOffsets=rc.LActOffsets,
                    thetas=q_curr, p_robot=p_curr
                    )

                # finally, compute the task space compliance
                Cx = forward_compliance.get_task_space_compliance_matrix(Jx, Cq)
                
                # update the task-space compliance to the multiprocessing shared variable
                mp_task_compliance[0] = Cx

                # print("task space compliance:")
                # print(mp_task_compliance[0])
            else:
                pass
    except Exception as e:
        print("Error in mp_compliance_main: ", e)


