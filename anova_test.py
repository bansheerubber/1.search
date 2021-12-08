from os import listdir
from os.path import join
#from scipy.stats import f_oneway
from scipy.stats import ttest_ind
#import pingouin as pt
from scipy.stats import ttest_rel
from matplotlib import pyplot as plt

onlyfiles = [f for f in listdir("results")]

#automate following 2 lists in future
maze_list = ['.stats.big-maze','.stats.sparse-maze','.stats.big-corners-maze-generator', '.stats.big-corners-maze-generator-sparse']
search_agents = ['astar', 'bi', 'bfs', 'dfs', 'ucs']

for maze in maze_list:
    node_expanded = []
    path_cost = []
    for agent in search_agents:
        node_expanded.append([])
        path_cost.append([])
        result_file = 'results/'+agent+maze
        f = open(result_file).readlines()
        for i in range(len(f)):
            x = f[i][:-1].split(',')
            #print(x)
            node_expanded[-1].append(int(x[0]))
            path_cost[-1].append(int(x[1]))
    print("----------------------")
    for i in range(len(search_agents)):
        #print(search_agents[i],'in maze ',maze[7:], ':', ttest_ind(node_expanded[1], node_expanded[i]))
        #print(len(node_expanded[1]),len(node_expanded[i]))
        print(search_agents[i],'in maze ',maze[7:], ':', ttest_ind(node_expanded[1][:7000], node_expanded[i][:7000],equal_var=False))
        #print(len(node_expanded[i][:7000]))
        # print(min(path_cost[1][:100]))
        # plt.plot([i for i in range(50)],node_expanded[1][:50],label='bi')
        # plt.plot([i for i in range(50)],node_expanded[i][:50],label='astar')
        # plt.legend()
        # plt.show()

    #print('------------------------------------------')
#    print(maze, ':',f_oneway(node_expanded[0],node_expanded[1],node_expanded[2],node_expanded[3],node_expanded[4]))
