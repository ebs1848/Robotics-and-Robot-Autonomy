# Lane control

Steering control of a robot for lane changing.

A bicycle-model robot follows a lane-change reference (y = 0, then y = 12 cm after x = 24 cm). `pid_control.py` holds the robot and the controllers (Dummy, Bang-Bang and PID) and `pid_simulation.py` runs the Matplotlib animation. The simulation adds a 15 degree steering assembly error, and the tuned PID gains (kp=3.0, ki=0.3, kd=5.0) keep the overshoot around 1.4 cm with a steady-state error under 0.1 cm.

## Run

```
python pid_control.py
```

The controller is chosen in the `__main__` block of `pid_control.py`.
