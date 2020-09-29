import numpy as np
import yaml

path1 = '/home/ms/catkin_ws/src/triple_contact_planner/yaml/bottom'
path2 = '/home/ms/catkin_ws/src/triple_contact_planner/yaml/top'
path3 = '/home/ms/catkin_ws/src/triple_contact_planner/yaml/permutation'
if __name__ == '__main__':
    members = []
    with open (path3 + '/bottom_all.yaml','w') as bottom_all:
        for link_num in range(1,9):
            with open (path1 + '/link' + str(link_num) + '.yaml','r') as bottom_links:
                members = yaml.safe_load(bottom_links)        

            bottom = []
            for member in members:
                # lower_bound = (members[p][0]['lower_bound'], members[p][0]['upper_bound'])      #tuple: () 요소 변경 불가(요소 추가 불가능)
                bound = np.asarray([members[member][0]['lower_bound'], members[member][0]['upper_bound']])    #list: [] 요소 변경 가능 (.append()를 이용해 요소 추가 가능하다)
                bound_dist = np.linalg.norm(bound[1]-bound[0])
                bound_vector = ((bound[1]) - (bound[0]))/bound_dist
                ori =members[member][0]['orientation']

                if bound_dist < 0.2:
                    # for i in np.arange(0,1.1,0.5):
                    i = 0.5
                    translation = ((bound[1]) -(bound[0]))*i + (bound[0])
                    tf = np.append(translation,ori)
                    bottom.append(tf.tolist())
                else:
                    for j in np.arange(0.1, bound_dist, 0.1):
                        translation = (bound_vector)*j + (bound[0])
                        tf = np.append(translation,ori)
                        bottom.append(tf.tolist())

            bottom_all.write('\n')
            bottom_all.write('link' + str(link_num) + ': ')
            bottom_all.write('\n')
            for s in bottom:
                bottom_all.write('  - ' +str(s))
                bottom_all.write('\n')