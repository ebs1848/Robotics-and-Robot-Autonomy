import numpy as np
from matplotlib.path import Path
from matplotlib.patches import Circle
import matplotlib.pyplot as plt

class Node:
    def __init__(self, node_id, position):
        self.id = node_id
        self.position = position
        self.neighbors = np.array([], dtype=int)
        self.backpointer = None
        self.is_visited = False
        self.obstacleid = 0
        self.g = None
        self.h = None
        self.f = None

def graphsearch(algorithm,nodes,ax):
    if algorithm == 'bfs':
        success = bfs(nodes,ax)
    elif algorithm == 'dfs':
        success = dfs(nodes,ax)
    elif algorithm == 'djikstra':
        success = djikstra(nodes,ax)
    elif algorithm == 'astar':
        success = astar(nodes,ax)
    return success

######## configuration space
def vectorFormat(shape,origin,negative=0):
    vector = np.array([np.array(shape[i]) - np.array(origin) for i in range(len(shape))])
    if negative==1:
        vector=-1*vector
    return vector

def minkowskiSum(obstacle,vector):
    conf_obstacle = np.array([[0,0]])
    for i in range(len(obstacle)):
            for j in range(len(vector)):
                conf_obstacle=np.append(conf_obstacle,np.array([obstacle.vertices[i]+vector[j]]),axis=0)
    conf_obstacle=conf_obstacle[1:]
    return conf_obstacle

def convexHull(obstacle,vectorRobot):
    conf_obstacle=np.empty((0,2))
    for i in range(len(obstacle)-1):
        corner=np.array([obstacle.vertices[i]])
        cornerNext=np.array([obstacle.vertices[i+1]])
        if i==0:
            cornerPrev=np.array([obstacle.vertices[len(obstacle)-2]])
        else:
            cornerPrev=np.array([obstacle.vertices[i-1]])
        vectorNext=cornerNext-corner
        vectorPrev=cornerPrev-corner
        crossNext=np.array([])
        crossPrev=np.array([])
        for i in range(len(vectorRobot)):
            crossNext=np.append(crossNext,np.cross(vectorNext,vectorRobot[i]))
            crossPrev=np.append(crossPrev,np.cross(vectorPrev,vectorRobot[i]))
        indices = np.where(crossPrev == np.max(crossPrev))
        for i in range(len(indices)):
            conf_obstacle=np.vstack([conf_obstacle,vectorRobot[indices[i]]+corner])
        indices = np.where(crossNext == np.min(crossNext))
        for i in range(len(indices)):
            conf_obstacle=np.vstack([conf_obstacle,vectorRobot[indices[i]]+corner])

    will_deleted=[]
    for i in range(len(conf_obstacle)-1):
        for j in range(i+1,len(conf_obstacle)):
            if all(conf_obstacle[i]==conf_obstacle[j]):
                will_deleted.append(j)
    conf_obstacle_flattened = np.delete(conf_obstacle,will_deleted, axis=0)
    conf_obstacle = conf_obstacle_flattened.reshape((-1,conf_obstacle.shape[1]))
    
    will_deleted=[]
    for i in range(len(conf_obstacle)-2):
        for j in range(i+1,len(conf_obstacle)-1):
            for k in range(j+1,len(conf_obstacle)):
                v2=conf_obstacle[k]-conf_obstacle[i]
                v1=conf_obstacle[j]-conf_obstacle[i]
                if np.cross(v1,v2)==0:
                    if np.dot(v1,v2)<0:
                        will_deleted.append(i)
                    else:
                        if np.linalg.norm(v2)>np.linalg.norm(v1):
                            will_deleted.append(j)
                        else:
                            will_deleted.append(k)
    conf_obstacle_flattened = np.delete(conf_obstacle,will_deleted, axis=0)
    conf_obstacle = conf_obstacle_flattened.reshape((-1,conf_obstacle.shape[1]))
    
    return conf_obstacle

def configurationSpace(obstacles,shape):
    #configuration_obstacles=obstacles
    #return configuration_obstacles
    vector=vectorFormat(shape,shape[0],1)
    configuration_obstacles=[]
    for obstacle in obstacles:
        #conf_obstacle=minkowskiSum(obstacle,vector)
        conf_obstacle=convexHull(obstacle,vector)
        obs = Path(np.vstack([conf_obstacle,conf_obstacle[0]]))
        configuration_obstacles.append(obs)
    return configuration_obstacles
   
######## visibility graph
def lineEquation(line):
    ydif=line[1][1]-line[0][1] #y2-y1
    xdif=line[1][0]-line[0][0] #x2-x1
    if xdif != 0:
        m=ydif/xdif
        c=line[1][1]-m*line[1][0]
        nf=0
    else:
        m=None
        c=line[1][0]
        nf=1
    return m,c,nf

def isIntersect(line,obs_edge):
    #line and obs_edge are in the form start and end point of lines s.t. [(x1,y1),(x2,y2)]
    m1,c1,nf1 = lineEquation(line) #line eqn with nintyflag
    m2,c2,nf2 = lineEquation(obs_edge) #line eqn with nintyflag
    if m1 != m2: #2 lines does not have same slope
        if nf1==0 and nf2==0: # 2 lines does not vertical
            xi=round((c2-c1)/(m1-m2),7) #xi: x coordinate intersection point 
            yi=round(m1*xi+c1,5)        #yi: y coordinate of the intersection point 
            #we found intersection coordinate (xi,yi)
            #need to check this intersection is inside the line segments or not
            if min(line[0][0],line[1][0])<xi<max(line[0][0],line[1][0]) and min(line[0][1],line[1][1])<=yi<=max(line[0][1],line[1][1]):
                if min(obs_edge[0][0],obs_edge[1][0])<=xi<=max(obs_edge[0][0],obs_edge[1][0]) and min(obs_edge[0][1],obs_edge[1][1])<=yi<=max(obs_edge[0][1],obs_edge[1][1]):
                    return True           
        elif nf1==1: # line1 is vertical
            xi=c1
            yi=m2*xi+c2
            if min(line[0][1],line[1][1])<yi<max(line[0][1],line[1][1]):
                if min(obs_edge[0][0],obs_edge[1][0])<xi<max(obs_edge[0][0],obs_edge[1][0]) and min(obs_edge[0][1],obs_edge[1][1])<=yi<=max(obs_edge[0][1],obs_edge[1][1]):
                    return True
        elif nf2==1: # line2 is vertical
            xi=c2
            yi=m1*xi+c1
            if min(line[0][0],line[1][0])<xi<max(line[0][0],line[1][0]) and min(line[0][1],line[1][1])<=yi<=max(line[0][1],line[1][1]):
                if min(obs_edge[0][1],obs_edge[1][1])<yi<max(obs_edge[0][1],obs_edge[1][1]):
                    return True 
        return False
    else:
        return False

def isVisible(obstacles,line):
    intersection=[]
    for obstacle in obstacles:
        for i in range(len(obstacle)-1):
            obs_edge=[obstacle.vertices[i],obstacle.vertices[i+1]]
            intersection = isIntersect(line,obs_edge)
            if intersection: #check 2 line segment intersects
                return 0
    return 1

def visibilityGraph(robot,target_position,obstacles):
    ########## CREATE NODES #########
    nodes = np.array([])
    nodes = np.append(nodes,Node(0,np.array(robot.position)))
    obs_start=[]
    k=1
    for obstacle in obstacles:
        obs_start.append(len(nodes))
        for i in range(len(obstacle)-1):
            nodes = np.append(nodes,Node(len(nodes),np.array(obstacle.vertices[i])))
            nodes[-1].obstacleid=k
        k=k+1
        for j in range(obs_start[-1],len(nodes)):
            if j!=obs_start[-1]:
                nodes[j].neighbors=np.append(nodes[j].neighbors,int(j-1))
            if j!=len(nodes)-1:
                nodes[j].neighbors=np.append(nodes[j].neighbors,int(j+1))
        nodes[len(nodes)-1].neighbors=np.append(nodes[len(nodes)-1].neighbors,int(obs_start[-1]))
        nodes[obs_start[-1]].neighbors=np.append(nodes[obs_start[-1]].neighbors,int(len(nodes)-1))
    nodes = np.append(nodes,Node(len(nodes),target_position))

    ########### CREATE VISIBILITY GRAPH #########
    if isVisible(obstacles,[nodes[0].position,nodes[-1].position]): # check start and target nodes visible from each other
        nodes[0].neighbors=np.append(nodes[0].neighbors,int(nodes[-1].id))
        nodes[-1].neighbors=np.append(nodes[-1].neighbors,int(nodes[0].id))
    for i in range(len(nodes)-1): #all nodes except start and end and last obstacle
        for j in range(i+1,len(nodes)):
            if nodes[i].obstacleid != nodes[j].obstacleid:
                if isVisible(obstacles,[nodes[i].position,nodes[j].position]):
                    nodes[i].neighbors=np.append(nodes[i].neighbors,int(nodes[j].id))
                    nodes[j].neighbors=np.append(nodes[j].neighbors,int(nodes[i].id))
    
    for i in range(len(nodes)):
            nodes[i].neighbors = np.sort(nodes[i].neighbors)#[::-1]
            #print(nodes[i].id,":",nodes[i].neighbors)
    return nodes


def plot_traversed_path(nodes,current_node,ax):
    if current_node !=0 :
        #ax.add_patch(Circle(nodes[current_node].position, radius=2, fill=True, color='green', linewidth=2))
        x_values = [nodes[current_node].position[0],nodes[nodes[current_node].backpointer].position[0]]
        y_values = [nodes[current_node].position[1],nodes[nodes[current_node].backpointer].position[1]]
        ax.plot(x_values,y_values,color='green')
        plt.pause(1.0)


############# BFS ##########################
def bfs(nodes,ax):
    start = nodes[0].id
    target_node=nodes[-1].id
    
    queue = np.array([], dtype=int)
    def enqueue(node):
        nonlocal queue
        queue = np.append(queue, node)
    def dequeue():
        nonlocal queue
        current_node = queue[0]
        queue = queue[1:]
        return current_node
    
    enqueue(start)
    nodes[start].is_visited=1

    success = 0
    while queue.size != 0: 
        current_node = dequeue()
        plot_traversed_path(nodes,current_node,ax)
        if current_node == target_node:
            success = 1
            break

        for neighbor in nodes[current_node].neighbors:
            if nodes[neighbor].is_visited == 0:
                enqueue(neighbor)
                nodes[neighbor].is_visited = 1
                nodes[neighbor].backpointer = current_node
    return success

############# DFS ############# 
def dfs(nodes, ax):
    start = nodes[0].id
    target_node = nodes[-1].id

    stack = np.array([], dtype=int)

    def push(node):
        nonlocal stack
        stack = np.append(stack, node)

    def pop():
        nonlocal stack
        if len(stack) > 0:
            current_node = stack[-1]
            stack = stack[:-1]  
            return current_node
        return None  

    push(start)
    nodes[start].is_visited = True

    success = False
    while len(stack) > 0:
        current_node = pop()
        plot_traversed_path(nodes, current_node, ax)

        if current_node == target_node:
            success = True
            break

        for neighbor in nodes[current_node].neighbors:
            if not nodes[neighbor].is_visited:
                push(neighbor)
                nodes[neighbor].is_visited = True
                nodes[neighbor].backpointer = current_node

    return success

###### DJIKSTRA #############
def djikstra(nodes, ax):
    start = nodes[0].id
    target_node = nodes[-1].id

    priorityQueue = np.array([], dtype=[('distance', float), ('node', int)])

    def enqueue(node):
        nonlocal priorityQueue
        distance = nodes[node].g if nodes[node].g is not None else float('inf')
        new_entry = np.array([(distance, node)], dtype=priorityQueue.dtype)
        priorityQueue = np.append(priorityQueue, new_entry)
        priorityQueue = np.sort(priorityQueue, order='distance')

    def update_queue():
        nonlocal priorityQueue
        priorityQueue = np.sort(priorityQueue, order='distance')

    def dequeue():
        nonlocal priorityQueue
        node = priorityQueue[0]['node']
        priorityQueue = np.delete(priorityQueue, 0)
        return node
    
    enqueue(start)
    nodes[start].g = 0
    
    success = 0
    while priorityQueue.size != 0:
        current_node = dequeue()
        plot_traversed_path(nodes, current_node, ax)

        if nodes[current_node].is_visited:
            continue

        nodes[current_node].is_visited = True

        if current_node == target_node:
            success = True
            break

        for neighbor in nodes[current_node].neighbors:
            if not nodes[neighbor].is_visited:
                new_distance = nodes[current_node].g + np.linalg.norm(nodes[neighbor].position - nodes[current_node].position)

                if nodes[neighbor].g is None or new_distance < nodes[neighbor].g:
                    nodes[neighbor].g = new_distance
                    nodes[neighbor].backpointer = current_node
                    enqueue(neighbor)
                    update_queue()

    return success

######### A STAR #############
def astar(nodes, ax):
    start = nodes[0].id
    target_node = nodes[-1].id

    priorityQueue = np.array([], dtype=int)

    def enqueue(node):
        nonlocal priorityQueue
        priorityQueue = np.append(priorityQueue, node)

    def update_queue():
        nonlocal priorityQueue
        f_values = np.array([nodes[node].g + np.linalg.norm(nodes[node].position - nodes[target_node].position) for node in priorityQueue])
        priorityQueue = priorityQueue[np.argsort(f_values)]

    def dequeue():
        nonlocal priorityQueue
        if priorityQueue.size > 0:
            node = priorityQueue[0]
            priorityQueue = np.delete(priorityQueue, 0)
            return node
        return None

    enqueue(start)
    nodes[start].g = 0

    success = 0
    while priorityQueue.size != 0:
        current_node = dequeue()
        plot_traversed_path(nodes, current_node, ax)

        if current_node == target_node:
            success = 1
            break

        nodes[current_node].is_visited = True

        for neighbor in nodes[current_node].neighbors:
            if not nodes[neighbor].is_visited:
                new_g = nodes[current_node].g + np.linalg.norm(nodes[neighbor].position - nodes[current_node].position)
                
                if nodes[neighbor].g is None or new_g < nodes[neighbor].g:
                    nodes[neighbor].g = new_g
                    nodes[neighbor].backpointer = current_node
                    enqueue(neighbor)

        update_queue()

    return success
