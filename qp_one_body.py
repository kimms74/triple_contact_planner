from triple_contact_planner import python_wrapper_cpp
import numpy as np

tt = python_wrapper_cpp.QpSolver()

t1 = np.array([-0.704605, -0.0200731, 0.0205463, 0.730345, 0.059292, 0.0639174, 0.677492])
t2 = np.array([-0.538106, -0.00185028, 0.0219165, -0.0582586, 0.677931, 0.730818, -0.0540426])
t3 = np.array([-0.529481, -0.017858, 0.328078, 0.996136, 0.079409, 0.00298125, -0.0373972])
rt = np.array([0,0,0,1])

if tt.qp_solve_one_body(t1,t2,t3,rt):
    print(tt.get_result_force(0))
    print(tt.get_result_force(1))
    print(tt.get_result_force(2))