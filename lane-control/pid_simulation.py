import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.animation import FuncAnimation
import numpy as np
lane_change = (24,12)  # the reference steps from y=0 to y=lane_change[1] at x=lane_change[0] (cm)

# Function to plot the robot body
def plot_robot(ax, robot, previous_robot=None, previous_wheel=None):
    # Calculate the center of the red rectangle
    x_BL_R = - robot.parameters.length / 2
    y_BL_R = - robot.parameters.width / 2
    
    #         ________________________
    #        |                        |
    #        |                        |
    #        |           |_           |
    #        |       (x_R,y_R)        |
    #        |                        |
    #        |________________________|
    # (x_BL_R,y_BL_R)

    if previous_robot is not None:
        previous_robot.remove()  # Remove the previous robot plot
    if previous_wheel is not None:
        previous_wheel.remove()  # Remove the previous wheel plot

    # Bottom-left corner of the body in the world frame
    theta=np.radians(robot.position.theta)
    x_BL_W = robot.position.x + x_BL_R * np.cos(theta) - y_BL_R * np.sin(theta)
    y_BL_W = robot.position.y + x_BL_R * np.sin(theta) + y_BL_R * np.cos(theta)

    # Plot the robot at current position
    robot_pos = Rectangle((x_BL_W, y_BL_W), robot.parameters.length, robot.parameters.width, angle=np.degrees(theta),
                          fill=True, alpha=0.5, edgecolor='black', linewidth=2)
    ax.add_patch(robot_pos)

    # Call the plot_wheel function
    wheel_pos = plot_wheel(ax, robot)

    return robot_pos, wheel_pos  # Return the new rectangle

# Draw the front wheel as a green rectangle, rotated by the steering angle
def plot_wheel(ax, robot):
    # Bottom left of wheel
    x_wheel_wh = - robot.parameters.wheel_length / 2
    y_wheel_wh = - robot.parameters.wheel_width / 2

    # Wheel corner in the robot frame
    steer = np.radians(robot.input.steer)
    x_wheel_R = robot.parameters.length / 2 + x_wheel_wh * np.cos(steer) - y_wheel_wh * np.sin(steer)
    y_wheel_R = x_wheel_wh * np.sin(steer) + y_wheel_wh * np.cos(steer)

    # Wheel corner in the world frame
    theta=np.radians(robot.position.theta)
    x_wheel_W = robot.position.x + x_wheel_R * np.cos(theta) - y_wheel_R * np.sin(theta)
    y_wheel_W = robot.position.y + x_wheel_R * np.sin(theta) + y_wheel_R * np.cos(theta)

    # Plot the wheel at correct position and steering angle
    wheel_pos = Rectangle((x_wheel_W, y_wheel_W), robot.parameters.wheel_length, robot.parameters.wheel_width, angle=np.degrees(theta+steer),
                      facecolor='green', edgecolor='none')
    ax.add_patch(wheel_pos)

    return wheel_pos

def simulation(fig, ax, robot, controller):
    # Set up the empty plot with specified boundaries and aspect ratio
    ax.set_xlim(-12, 192)  # Set x-axis boundaries
    ax.set_ylim(-12, 30)  # Set y-axis boundaries
    ax.set_aspect('equal', adjustable='box')  # Preserve aspect ratio

    # Add gridlines
    ax.set_xticks(np.arange(-10, 190, 10))
    ax.set_xticks(np.arange(-12, 198, 2), minor=True)
    ax.set_yticks(np.arange(-10, 30, 10))
    ax.set_yticks(np.arange(-12, 30, 2), minor=True)

    ax.grid(which='minor', alpha=0.2)
    ax.grid(which='major', alpha=0.5)


    # Add labels
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')

    # Add reference line
    line1, = ax.plot([0, lane_change[0]], [0, 0], color='black', linewidth=3)  # First line
    line2, = ax.plot([lane_change[0], lane_change[0]], [0, lane_change[1]], color='black', linewidth=3)  # Second line
    line3, = ax.plot([lane_change[0], 198], [lane_change[1], lane_change[1]], color='black', linewidth=3)

    # Initialize the parameters
    previous_robot_pos = None
    previous_wheel_pos = None

    # Initialize the previous position to plot traveled path
    prev_x, prev_y = robot.position.x, robot.position.y

    # Initialization function for the animation
    def init():
        return []

    # Animation function to update the position of the red rectangle's center and rotation angle
    def animate(frame):
        nonlocal previous_robot_pos, previous_wheel_pos, prev_x, prev_y, robot  # Use the previous_robot and prev_x, prev_y from the outer scope
        
        # Compute the control input for this frame
        robot.input = calculate_input(robot, controller)

        # Steering assembly error model
        robot.input.steer -= 15 # 15 degree offset in the clockwise direction

        # Update the position with bicycle kinematic model
        robot.position = bicycle_kinematic(robot)

        # Call the plot_robot function to update the robot position
        previous_robot_pos, previous_wheel_pos = plot_robot(ax, robot, previous_robot_pos, previous_wheel_pos)

        # Plot the traveled path
        line = ax.plot([prev_x, robot.position.x], [prev_y, robot.position.y], 'r.', lw=1, ls='dotted')
        prev_x, prev_y = robot.position.x, robot.position.y

    # Create the animation
    ani = FuncAnimation(fig, animate, frames=600, init_func=init, blit=False, repeat=False, interval=100)
    # Increase the interval (e.g. 200 ms) to slow the animation down
    
    # Show the animation
    plt.show()

# Function to update the position of robot according to bicycle kinematic model
def bicycle_kinematic(robot):
    theta = np.radians(robot.position.theta)
    steer = np.radians(robot.input.steer)
    
    beta  = np.arctan(0.5*np.tan(steer)) # Center of gravity is the middle point of the robot
    
    robot.position.x+=robot.input.vel*np.cos(theta+beta)*robot.parameters.dt
    robot.position.y+=robot.input.vel*np.sin(theta+beta)*robot.parameters.dt
    robot.position.theta+=np.degrees((robot.input.vel/robot.parameters.length)*np.cos(beta)*np.tan(steer)*robot.parameters.dt)

    return robot.position

# Picks the reference y for the current x and calls the controller with 
# y_ref as the reference point and the robot's y position as the measurement
def calculate_input(robot, controller):
    if robot.position.x < lane_change[0]: #check the x position of robot.
        y_ref=0
    else: # past x=24 the reference switches to the new lane
        y_ref=lane_change[1]
    
    # Calculate the steering angle by calling the controller
    robot.input.steer=controller.update(y_ref,robot.position.y)

    return robot.input
