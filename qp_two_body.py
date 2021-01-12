from triple_contact_planner import python_wrapper_cpp
import numpy as np
import yaml

class ContinuousGraspCandid():
    def __init__(self):
        self.file_path = 'ikea_stefan_without_bottom_cont_grasp.yaml'
        with open(self.file_path, 'r') as stream:
            self.yaml = yaml.safe_load(stream)

    def get_grasp(self, index, ratio):
        lb = np.array(self.yaml["grasp_points"][index]['lower_bound'])
        ub = np.array(self.yaml["grasp_points"][index]['upper_bound'])
        ori = np.array(self.yaml["grasp_points"][index]['orientation'])
        # print((ub-lb) * ratio + lb)
        # print(ori)
        trans = (ub-lb) * ratio + lb 
        grasp = np.concatenate((trans, ori))
        return grasp
tt = python_wrapper_cpp.QpSolver()

idx_lists = [ 
[12, 0.19341686393452673],
[43, 0.32571935556810216],
[48, 0.40444404066775363]] 

grasp = ContinuousGraspCandid()

t1 = grasp.get_grasp(idx_lists[0][0], idx_lists[0][1])
t2 = grasp.get_grasp(idx_lists[1][0], idx_lists[1][1])
t3 = grasp.get_grasp(idx_lists[2][0], idx_lists[2][1])

rt = np.array([0,0,0,1])

if tt.qp_solve(t1,t2,t3,rt):
    print(tt.get_result_force(0))
    print(tt.get_result_force(1))
    print(tt.get_result_force(2))
else:
    print('failed')