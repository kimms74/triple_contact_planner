import numpy as np
import itertools as it
import yaml

path = '/home/ms/catkin_ws/src/triple_contact_planner/yaml/permutation'
if __name__ == '__main__':
    bottom_members = []
    top_members = []
    # with open (path + '/bottom_permutation.yaml','w') as bottom_permutation:
    with open (path + '/bottom1_top2_permutation.yaml','w') as hand_permutation:
        with open (path + '/top_all.yaml','r') as top_all:
            top_members = yaml.safe_load(top_all)
            with open (path + '/bottom_all.yaml','r') as bottom_all:
                bottom_members = yaml.safe_load(bottom_all)
                # for i in range(1,9):
                    # for top_member in top_members:
                        # print(len(link))
                        # # print(link)
                # print(len(bottom_members))
                # print(len(top_members))
                permutation = it.permutations(top_members,2)
                per = list(permutation)
                # print(len(per))
                # bottom_per =[]
                hand_per = []

                for i in range(0,len(per)):
                    for j in range(0,len(top_members[per[i][0]])):
                        for k in range(0,len(top_members[per[i][1]])):
                            for bottom_member in bottom_members:
                                # print(len(top_members[top_member]))
                                for m in range(0,len(bottom_members[bottom_member])):
                                    # bottom_per.append([bottom_members[per[i][0]][j], bottom_members[per[i][1]][k]])
                                    hand_per.append([bottom_members[bottom_member][m], top_members[per[i][0]][j], top_members[per[i][1]][k]])
                # bottom_permutation.write('\n')
                # for l in bottom_per:
                #     bottom_permutation.write('  - tf1: ' + str(l[0]))
                #     bottom_permutation.write('\n')
                #     bottom_permutation.write('    tf2: ' + str(l[1]))
                #     bottom_permutation.write('\n')
                hand_permutation.write('\n')
                for n in hand_per:
                    hand_permutation.write('  - tf1: ' + str(n[0]))
                    hand_permutation.write('\n')
                    hand_permutation.write('    tf2: ' + str(n[1]))
                    hand_permutation.write('\n')
                    hand_permutation.write('    tf3: ' + str(n[2]))
                    hand_permutation.write('\n')
