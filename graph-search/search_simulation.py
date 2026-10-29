from matplotlib.path import Path
from matplotlib.patches import Circle
import matplotlib.patches as patches
import matplotlib.pyplot as plt

import numpy as np

# Function to plot the robot body
def plot_robot(ax, robot, prev_pos):
    if prev_pos is not None:
        prev_pos.remove()  # Remove the previous robot plot

    # Plot the robot at current position
    shape = robot.shape + [robot.shape[0]]
    robot_pos = Path([(x + robot.position[0], y + robot.position[1]) for x, y in shape])
    robot_plot = patches.PathPatch(robot_pos, edgecolor='blue', facecolor='blue')
    ax.add_patch(robot_plot)

    return robot_plot  # Return the new rectangle


def simulation(fig, ax, robot, prev_pos):

     # Call the plot_robot function to update the robot position
    prev_pos = plot_robot(ax, robot, prev_pos)
    # (the final path is drawn in path_planning.py once the search finishes)
    return prev_pos


#####################################PLOT SIM ENVIRONMENT######################################
def plot_environment(fig,ax,obstacles,target_position,conf=0):
    # Set up the empty plot with specified boundaries and aspect ratio
    ax.set_xlim(-30, 200)  # Set x-axis boundaries
    ax.set_ylim(-30, 200)  # Set y-axis boundaries
    ax.set_aspect('equal', adjustable='box')  # Preserve aspect ratio

    # Add gridlines
    ax.set_xticks(np.arange(-30, 200, 10))
    ax.set_xticks(np.arange(-30, 200, 2), minor=True)
    ax.set_yticks(np.arange(-30, 200, 10))
    ax.set_yticks(np.arange(-30, 200, 2), minor=True)

    ax.grid(which='minor', alpha=0.2)
    ax.grid(which='major', alpha=0.5)


    # Add labels
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')

    for obstacle in obstacles:
        if conf:
            obs=patches.PathPatch(obstacle, facecolor='blue', alpha=0.4)
        else:
            obs=patches.PathPatch(obstacle, edgecolor='black', facecolor='gray')
        ax.add_patch(obs)
    
    ax.add_patch(Circle(target_position, radius=1, fill=True, color='red', linewidth=2))

    return fig, ax

def plot_graph(ax,nodes):
    for i in range(len(nodes)):
        neighbors=nodes[i].neighbors
        for j in range(len(neighbors)):
            if nodes[i].id<neighbors[j]:
                x_values=[nodes[i].position[0],nodes[int(neighbors[j])].position[0]]
                y_values=[nodes[i].position[1],nodes[int(neighbors[j])].position[1]]
                ax.plot(x_values, y_values, color='lightblue',linewidth=1.2)


################# ENVIRONMENT ###################
def environment(env):
    if env == 1:
        position = np.array([0,0]) #initial position. x,y
        target_position = np.array([120,80])
        obstacle_corners = [[(30,30),(80,30),(80,80)],
                            [(100,60),(180,60),(180,70),(100,70)],
                            [(-20,-20),(-10,-20),(-10,-10),(-20,-10)]]
    
    elif env == 2:
        position = np.array([40,60]) #initial position. x,y
        target_position = np.array([105,175])
        obstacle_corners = [[(-13,0),(15,0),(15,15),(-13,15)],
                            [(15,115),(70,115),(70,130),(15,130)],
                            [(95,20),(115,20),(115,110),(95,110),(75,70)],
                            [(150,130),(170,130),(170,160),(150,160)],
                            [(-20,140),(-10,140),(-10,160)],
                            [(93,155),(130,155),(130,165),(93,165)],
                            [(135,40),(150,25),(195,40),(180,90),(135,90)]]    
    elif env == 3:
        position = np.array([-20,-25]) #initial position. x,y
        target_position = np.array([190,150])
        obstacle_corners = [[(0,0),(10,0),(10,15),(0,15)],
                            [(20,100),(70,100),(70,125),(20,125)],
                            [(90,10),(110,10),(110,100),(90,100),(60,40)],
                            [(140,110),(160,110),(160,140),(140,140)],
                            [(40,0),(60,0),(40,40)],
                            [(90,130),(130,130),(130,140),(90,140)],
                            [(135,40),(150,25),(195,40),(180,90),(135,90)]]

    return position,target_position,obstacle_corners

