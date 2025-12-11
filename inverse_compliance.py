# the main file for the inverse compliance solver
# this file contains a process that serves at the top level module of the inverse compliance solver
# the top level module will use null space projection method to generate random robot configurations.

import numpy as np
import time

import robot_constants as rc
import kinematics_mp
import pressure_to_config
import joint_compliance_matrix
import forward_compliance
import visualization_ee_mp
import test_generate_cx


from scipy.optimize import differential_evolution, minimize


# note that this solver class uses multiprocessing
class inverse_compliance_solver:
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

    def inverse_compliance_solver_main(self):
        pass

    def mutate_configuration(self):
        pass

    def generate_random_pressure_at_q(self, q, robot_base_SE4):
        # compute the pressure coefficient according to configuration
        ret_array = pressure_to_config.v2_compute_const_and_coef_for_3_segment_robot(self.params, q, robot_base_SE4)

        const_vec = (ret_array.T)[0,:]
        coef_vec  = (ret_array.T)[1,:]

        rand_PA = []
        PActMin = 0.1* 6894.76
        PActMax = 40 * 6894.76
        for i, const_and_coef in enumerate(ret_array):
            [const, coef] = const_and_coef
            PBmin = max(PActMin, (PActMin-const)/coef)
            PBmax = min(PActMax, (PActMax-const)/coef)
            # assign a starting pressure
            rand_PA.append(np.random.uniform(const + coef * PBmin, const + coef * PBmax))

        rand_PB = self.compute_PB_from_PA(rand_PA, const_vec, coef_vec)

        return self.get_p_array_from_PA_PB(rand_PA, rand_PB)


    def setup_optimization_for_fixed_q(self, q, robot_base_SE4):
        # compute the pressure coefficient according to configuration
        ret_array = pressure_to_config.v2_compute_const_and_coef_for_3_segment_robot(self.params, q, robot_base_SE4)
        # generate constraints using the const and coef
        antagonistic_pair_id = [(3,1),(4,2),(7,5),(8,6),  (11,9),(12,10),(15,13),(16,14),  (19,17),(20,18),(23,21),(24,22)]
        # first, compute the upper and lower bounds for the learding variables
        bounds = []
        optvar_init = []
        PActMin = 0.1* 6894.76
        PActMax = 40 * 6894.76
        for i, const_and_coef in enumerate(ret_array):
            [const, coef] = const_and_coef
            PBmin = max(PActMin, (PActMin-const)/coef)
            PBmax = min(PActMax, (PActMax-const)/coef)
            bounds.append((const + coef * PBmin, const + coef * PBmax))
            # assign a starting pressure
            optvar_init.append((const + coef * PBmin + const + coef * PBmax) / 2.0)

        optvar_init = np.array(optvar_init)
        
        const_vec = (ret_array.T)[0,:]
        coef_vec  = (ret_array.T)[1,:]

        arm_compl_coef = joint_compliance_matrix.get_arm_compliance_matrix_coef(self.nlinks, self.params, self.Nfis, self.Bfis, self.LBases, self.LActOffsets, q)
        
        # get jacobian
        ee_tip_SE4 = kinematics_mp.fkine_mk5_with_rod_ee(self.params, self.end_effector_offset_SE4, q)
        # then, use this ee_tip as the end effector position, compute the analytic jacobian
        jacobian = kinematics_mp.get_analytic_jacobian(self.params, q, ee_tip_SE4)

        return optvar_init, bounds, const_vec, coef_vec, arm_compl_coef, jacobian


    def compute_PB_from_PA(self, PA, Const, Coef):
        # input should all be numpy array
        return (PA - Const) / Coef
    
    def compute_PA_from_PB(self, PB, Const, Coef):
        # input should all be numpy array
        return PB * Coef + Const
    
    def get_p_array_from_PA_PB(self, PA, PB):
        antagonistic_pair_id = np.array([(3,1),(4,2),(7,5),(8,6),  (11,9),(12,10),(15,13),(16,14),  (19,17),(20,18),(23,21),(24,22)])
        p_array = np.zeros(24)
        
        p_array[antagonistic_pair_id[:, 0]-1] = PA
        p_array[antagonistic_pair_id[:, 1]-1] = PB

        return p_array


    def run_optimize_for_fixed_q(self, q, robot_base_SE4, target_Cx):

        optvar_init, bounds, const_vec, coef_vec, arm_compl_coef, jacobian = self.setup_optimization_for_fixed_q(q, robot_base_SE4)

        result = minimize(self.compliance_objective_function,
                          optvar_init,
                          args=(target_Cx, const_vec, coef_vec, jacobian, arm_compl_coef),
                          bounds=bounds,
                          method='nelder-mead', options={'maxiter': 1000, 'xatol': 1e-5, 'disp': True})
        
        # result = differential_evolution(self.compliance_objective_function,
        #                                 x0=optvar_init,
        #                                 args=(target_Cx, const_vec, coef_vec, jacobian, arm_compl_coef),
        #                                 bounds=bounds,
        #                                 popsize=15)
        
        return result, const_vec, coef_vec

    def compliance_objective_function(self, optvar, target_Cx, const_vec, coef_vec, jacobian, arm_compl_coef):
        # input: p3,p4,p7,p8,  p11,p12,p15,p16, p19,p20,p23,p24 (the leading actuator(fixed) pressure in an antagonistic pair)
        # incerasing/decreasing those pressures will cause linear inc/dec for their corresponding antagonistic pressure.
        # the input will be bounded (according to max/min actuator pressure) before running.

        p_vec = self.get_p_array_from_PA_PB(optvar, self.compute_PB_from_PA(optvar, const_vec, coef_vec))
        # use the p array to compute task space compliance matrix
        # first, compute the joint space compliance matrix
        
        joint_compl_matrix = joint_compliance_matrix.get_joint_space_compliance_matrix_from_arm_compliance_coef(self.nlinks, p_vec, arm_compl_coef)
        cartesian_compl_matrix = forward_compliance.get_task_space_compliance_matrix(jacobian, joint_compl_matrix)        
        
        err = np.sum((cartesian_compl_matrix - target_Cx) ** 2) * 1000

        return err



if __name__ == "__main__":
    

    # test_q = np.random.uniform(-0.5, 0.5, size=12)

    test_q = np.array([0.3, 0,    0.3, -0.3,     -0.3,0,     -0.3,0.3,     -0.3,0,     -0.3,0.3], dtype=float) * 1

    test_base_SE4 = np.eye(4)
    test_base_SE4[2,3] = 0.8
    
    R_y = np.array([
            [np.cos(np.pi/4), 0, np.sin(np.pi/4)],
            [0,       1,    0],
            [-np.sin(np.pi/4), 0, np.cos(np.pi/4)],
        ], dtype=float)
    
    # test_base_SE4[0:3,0:3] = R_y
    
    armvis = visualization_ee_mp.visualizer_umarm(None, None, None, rc.params, rc.my_end_effector_homo, test_base_SE4)

    fcompl_solver = forward_compliance.forward_compliance_solver(rc.params, rc.nlinks, rc.Bfis, rc.Nfis, rc.LBases, rc.LActOffsets, rc.my_end_effector_homo)

    icompl_solver = inverse_compliance_solver(rc.params, rc.nlinks, rc.Bfis, rc.Nfis, rc.LBases, rc.LActOffsets, rc.my_end_effector_homo)

    # generate a random p
    rand_p = icompl_solver.generate_random_pressure_at_q(test_q, test_base_SE4)

    test_target_Cx = fcompl_solver.get_task_space_compliance(test_q, rand_p)

    #test_target_Cx = np.eye(3) * 0.01


    # ================ test for generated Cx ====================
    x_axis = np.array([1,0,0],dtype=float)
    y_axis = np.array([0,1,0],dtype=float)
    z_axis = np.array([0,0,1],dtype=float)

    x_val = 2 / 100.0
    y_val = 12 / 100.0
    z_val = 0.3 / 100.0

    test_target_Cx = test_generate_cx.generate_Cx(x_axis, y_axis, z_axis, x_val, y_val, z_val)

    # we should convert this Cx into robot frame!
    test_target_Cx_rframe_SE4 = np.eye(4)
    test_target_Cx_rframe_SE4[0:3,0:3] = test_target_Cx

    # get the ee frame
    ee_frame_SE4 = kinematics_mp.fkine_mk5_with_rod_ee(rc.params, rc.my_end_effector_homo, test_q)
    # convert the ee frame to spatial frame
    ee_frame_SE4 = kinematics_mp.robot_frame_to_spatial_frame(ee_frame_SE4, test_base_SE4)
    # test_target_Cx_rframe_SE4 = kinematics_mp.spatial_frame_to_robot_frame(test_target_Cx_rframe_SE4, ee_frame_SE4)
    test_target_Cx = test_target_Cx_rframe_SE4[0:3,0:3]

    print("generated Cx: \n", test_target_Cx)


    # ===========================================================
    

    start_time = time.time()
    result, const_vec, coef_vec = icompl_solver.run_optimize_for_fixed_q(test_q, test_base_SE4, test_target_Cx)
    print("execution time: ", time.time() - start_time)
    
    result_PB = icompl_solver.compute_PB_from_PA(result.x, const_vec, coef_vec)
    result_p_vec = icompl_solver.get_p_array_from_PA_PB(result.x, result_PB)

    print("rand_p: ", rand_p / 6894.76)
    print("result_p_vec: ", result_p_vec / 6894.76)

    result_target_Cx = fcompl_solver.get_task_space_compliance(test_q, result_p_vec)

    print("test_target_cx: \n", test_target_Cx)
    print("result_target_Cx: \n", result_target_Cx)

    armvis.static_arm_plot(test_q, [], test_base_SE4, enable_comliance=True, optional_Cx=[result_target_Cx, test_target_Cx], forcetest=False)

    pass