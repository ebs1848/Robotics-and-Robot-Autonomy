import search_simulation
import graph_search
import matplotlib.pyplot as plt
from matplotlib.path import Path
import numpy as np
import time

# Initialize the parameters
prev_positions = None
plt.ion()

class Robot:
    def __init__(self, position, shape):
        self.position = position
        self.shape = shape


###################################################### MAIN ##################################################

if __name__ == "__main__":
    fig, ax = plt.subplots(figsize=(20, 20))
    simTime = 0.1

    method = 'astar' #Allowed inputs:'bfs', 'dfs', 'djikstra', 'astar'                   # selected search method
    environment = 3 #Allowed inputs: 1,2,3                                             # selected environment
    position,target_position,obstacle_corners = search_simulation.environment(environment) 
    print(method)
    print(environment)

    obstacles=[]
    for obs_corner in obstacle_corners:
        obs = Path(obs_corner+[obs_corner[0]])
        obstacles.append(obs)
    
    shape    = [(0,0),(8,0),(0,10)]
    #create a robot
    robot = Robot(position=position,shape=shape)
    
    fig, ax = search_simulation.plot_environment(fig,ax,obstacles,target_position)
    prev_positions=search_simulation.simulation(fig, ax, robot, prev_positions)

    configuration_obstacles=graph_search.configurationSpace(obstacles,shape)
    prev_positions=None
    fig, ax = search_simulation.plot_environment(fig,ax,configuration_obstacles,target_position,1)
    prev_positions=search_simulation.simulation(fig, ax, robot, prev_positions)

    nodes=graph_search.visibilityGraph(robot,target_position,configuration_obstacles)
    search_simulation.plot_graph(ax,nodes)

    start_time = time.time()
    success=graph_search.graphsearch(method,nodes,ax)
    elapsed_time = time.time()-start_time

    print('elapsed time (sec):',elapsed_time)

    if success:
        path=[]
        length=0
        current_node=-1
        while nodes[current_node].backpointer != None:
            position2=nodes[current_node].position
            current_node=nodes[current_node].backpointer
            position1=nodes[current_node].position
            length += np.linalg.norm(position2 - position1)
            path.append(current_node)
        print('Path length (m):',length)
        path=path[::-1]
        path.append(nodes[-1].id)
        print('Visited nodes',path)
        for i in range(len(path)-1):
            x_values = [nodes[path[i]].position[0],nodes[path[i+1]].position[0]]
            y_values = [nodes[path[i]].position[1],nodes[path[i+1]].position[1]]
            ax.plot(x_values,y_values,color='blue',linewidth=3)
    else:
        print("No solution")
    plt.pause(15)
