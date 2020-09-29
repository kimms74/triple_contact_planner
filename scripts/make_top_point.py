import numpy as np
import yaml

path1 = '/home/ms/catkin_ws/src/triple_contact_planner/yaml/bottom'
path2 = '/home/ms/catkin_ws/src/triple_contact_planner/yaml/top'
path3 = '/home/ms/catkin_ws/src/triple_contact_planner/yaml/permutation'
if __name__ == '__main__':
    members = []
    with open (path3 + '/top_all.yaml','w') as top_all:
        for link_num in range(1,11):
            with open (path2 + '/link' + str(link_num) + '.yaml','r') as bottom_link1:
                members = yaml.safe_load(bottom_link1)
        # with open (path1 + '/link' + str(1) + '.yaml','r') as bottom_link1:
        #     members = yaml.safe_load(bottom_link1)            

            bottom = []
            for member in members:
                # lower_bound = (members[p][0]['lower_bound'], members[p][0]['upper_bound'])      #tuple: () 요소 변경 불가(요소 추가 불가능)
                bound = np.asarray([members[member][0]['lower_bound'], members[member][0]['upper_bound']])    #list: [] 요소 변경 가능 (.append()를 이용해 요소 추가 가능하다)
                bound_dist = members[member][0]['distance']
                bound_vector = ((bound[1]) - (bound[0]))/bound_dist
                ori =members[member][0]['orientation']
                #To do: lower_bound와 upper_bound 지점이 파지 가능한 지점인가? 
                # 아니면 그리퍼의 크기를 고려하여 그 사이만 잡을 수 있는 건가
                if bound_dist < 0.2:
                    for i in np.arange(0,1.1,0.5):
                        translation = ((bound[1]) -(bound[0]))*i + (bound[0])
                        tf = np.append(translation,ori)
                        bottom.append(tf.tolist())
                else:
                    for i in np.arange(0, bound_dist, 0.1):
                        translation = (bound_vector)*i + (bound[0])
                        tf = np.append(translation,ori)
                        bottom.append(tf.tolist())

            top_all.write('\n')
            top_all.write('link' + str(link_num) + ': ')
            top_all.write('\n')
            for s in bottom:
                top_all.write('  - ' +str(s))
                top_all.write('\n')

            # yaml.dump(bottom,top_all)