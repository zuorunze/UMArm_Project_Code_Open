import numpy as np

import kinematics_mp
import joint_compliance_matrix
import robot_constants as rc

def get_task_space_compliance_matrix(Jx, Cq):

    Cx = Jx @ Cq @ (Jx.T)

    return Cx



class forward_compliance_solver:

    params = None
    nlinks = None
    Bfis = None
    Nfis = None
    LBases = None
    LActOffsets = None
    end_effector_offset_SE4 = None


    def __init__(self, input_params, input_nlinks, input_Bfis, input_Nfis, input_LBases, input_LActOffsets, input_ee_SE4):
        self.params = input_params
        self.nlinks = input_nlinks
        self.Bfis   = input_Bfis
        self.Nfis   = input_Nfis
        self.LBases = input_LBases
        self.LActOffsets = input_LActOffsets
        self.end_effector_offset_SE4 = input_ee_SE4
        return

    def get_task_space_compliance(self, q, p):
        ee_tip_SE4 = kinematics_mp.fkine_mk5_with_rod_ee(self.params, self.end_effector_offset_SE4, q)
        # get the current jacobian
        jacobian = kinematics_mp.get_analytic_jacobian(self.params, q, ee_tip_SE4)
        # then, compute the configuration space compliance matrix
        # first, the compliance coefs
        arm_compl_coef = joint_compliance_matrix.get_arm_compliance_matrix_coef(self.nlinks, self.params, self.Nfis, self.Bfis, self.LBases, self.LActOffsets, q)
        # then, the actual compliance matrix
        joint_compl_matrix = joint_compliance_matrix.get_joint_space_compliance_matrix_from_arm_compliance_coef(self.nlinks, p, arm_compl_coef)
        # then, compute task-space compliance
        Cx = get_task_space_compliance_matrix(jacobian, joint_compl_matrix)

        return Cx

    pass