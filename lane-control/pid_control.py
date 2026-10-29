import pid_simulation
import matplotlib.pyplot as plt
import numpy as np

class Robot:
    class Position: #create states of robot
        def __init__(self, x, y, theta):
            self.x = x
            self.y = y
            self.theta = theta #in degree

    class Parameters: #physical parameters of robot
        def __init__(self, length, width, wheel_length, wheel_width,dt):
            self.length = length
            self.width = width
            self.wheel_length = wheel_length
            self.wheel_width = wheel_width
            self.dt = dt

    class Input: #define the input of robot as velocity and steering angle
        def __init__(self, vel, steer):
            self.vel = vel
            self.steer = steer

    def __init__(self, position, parameters, input):
        self.position = position
        self.parameters = parameters
        self.input = input

class DummyController:
    def __init__(self,simTime):
        self.simTime = simTime
        self.output  = 0.0

    def update(self, reference_point, measurement):
        # constant steering command in degrees (0.0 keeps the robot driving straight)

        return self.output

class PIDController:
    def __init__(self, kp, ki, kd, simTime, integral_limit=100.0):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain

        self.prev_error = 0.0  # Previous error
        self.prev_derivative = 0.0  # reserved for derivative-kick filtering (not used yet)
        self.integral = 0.0  # Integral term
        self.output = 0.0  # Controller output
        self.dt = simTime  # Time step
        self.integral_limit = integral_limit  # Integral limit

    def update(self, reference_point, measurement):
        # Calculate the error
        error = reference_point - measurement

        # Update necessary parameters
        self.integral += error * self.dt

        # Calculate the derivative of the error
        derivative = (error - self.prev_error) / self.dt

        # Apply anti-windup to integral term
        if self.integral > self.integral_limit:
            self.integral = self.integral_limit
        elif self.integral < -self.integral_limit:
            self.integral = -self.integral_limit

        # Calculate the control output
        self.output = self.kp * error + self.ki * self.integral + self.kd * derivative  # PID sum

        # Update necessary parameters for the next iteration
        self.prev_error = error

        return self.output

class BangBangController:
  def __init__(self,margin,turn_angle):
        self.margin=margin
        self.turn_angle=turn_angle

  def update(self, reference_point, measurement):
                # Calculate the error
        error = reference_point - measurement

         #Apply Bang-Bang control
        if error > self.margin:
            self.output = self.turn_angle  # Turn right (positive angle)
        elif error < -self.margin:
            self.output = -self.turn_angle  # Turn left (negative angle)
        else:
            self.output = 0.0  # Stay straight

        return self.output


    

if __name__ == "__main__":
    fig, ax = plt.subplots(figsize=(40, 8))
    simTime = 0.1
    
    position=Robot.Position(x=0,y=0,theta=0) #initial position. x,y cm theta degree
    input_=Robot.Input(vel=10,steer=0)  #initial control input. vel cm/s, steer degree
    params=Robot.Parameters(length=16,width=8,wheel_length=5,wheel_width=1,dt=simTime) #robot parameters (L=16cm, W=8cm)

    robot=Robot(position=position,input=input_,parameters=params) #create a robot

    ########## Controller selection (the last assignment wins) ##########
    controller = DummyController(simTime=simTime)
    #controller = BangBangController(margin=2.0, turn_angle=30.0)
    controller = PIDController(kp=3.0, ki=0.3, kd=5.0, simTime=simTime)
    #controller = BangBangController(margin=0.0, turn_angle=0.0) # margin in cm, turn_angle in degrees
    #controller = PIDController(kp=0.1, ki=0.0, kd=0.0, simTime=simTime) # alternative, lower PID gains
    # the steering assembly error is applied inside pid_simulation.py

    pid_simulation.simulation(fig, ax, robot, controller)
