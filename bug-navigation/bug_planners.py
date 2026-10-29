import numpy as np

def planner(robot,sensor,target_position):
    if robot.planner == 0:
        bug0(robot,sensor,target_position)
    elif robot.planner == 1:
        bug1(robot,sensor,target_position)
    elif robot.planner == 2:
        bug2(robot,sensor,target_position)
    elif robot.planner == 3:
        tangent_bug(robot,sensor,target_position)

####################################BUG0################################

def check_obstacle(position,target_position):
    vt=[target_position[0]-position.x,target_position[1]-position.y]
    vr=[np.cos(np.radians(position.theta)),np.sin(np.radians(position.theta))]
    direction = np.cross(vr,vt)
    if direction<=0:
        free_side=0
    else:
        free_side=1
    return free_side

def bug0(robot,bump_sensors,target_position):
    if robot.fsm == "m2g":
        if any(sensor.activated for sensor in bump_sensors):
            robot.fsm = "wf"
        else:
            robot.fsm = "m2g"
    elif robot.fsm == "wf":
        if check_obstacle(robot.position,target_position):
            robot.fsm = "m2g"
        else:
            robot.fsm = "wf"

######################################BUG1###############################
def write_to_memory(robot,target_position):
    robot.memory.pos = np.append(robot.memory.pos,[robot.position.x,robot.position.y])
    d = np.linalg.norm(target_position-np.array([robot.position.x,robot.position.y]))
    robot.memory.d = np.append(robot.memory.d,d)

def check_point_hit(position,point):
    dist = np.linalg.norm(np.array(point)-np.array([position.x,position.y]))
    return dist<1.25

def bug1(robot,bump_sensors,target_position):
    if robot.fsm == "m2g":
        if any(sensor.activated for sensor in bump_sensors):
            robot.fsm = "wf"
            robot.touchpoint = [robot.position.x, robot.position.y]
        else:
            robot.fsm = "m2g"
    elif robot.fsm == "wf":
        if check_point_hit(robot.position,robot.touchpoint):
            robot.fsm = "m2lp"
            robot.leavepoint = np.array([robot.memory.pos[2*np.argmin(robot.memory.d)], robot.memory.pos[2*np.argmin(robot.memory.d)+1]])
        else:
            robot.fsm = "wf" 
    elif robot.fsm == "m2lp":
        if check_point_hit(robot.position,robot.leavepoint):
            if check_obstacle(robot.position,target_position):
                robot.fsm = "m2g"
            else:
                robot.fsm = "noSoln"
        else:
            robot.fsm = "m2lp"

#################################BUG2##################################
def calculate_mline(robot,target_position):
    if robot.position.x == target_position[0]:
        nf=1
        m=None
        c=robot.position.x
    else:
        nf=0
        m=(target_position[1]-robot.position.y)/(target_position[0]-robot.position.x)
        c=robot.position.y-m*robot.position.x
    robot.memory.bug2param=np.append(robot.memory.bug2param,[nf,m,c,None,0])

def check_mline_hit(robot):
    if robot.memory.bug2param[1]==1:
        return np.abs(robot.position.x-robot.memory.bug2param[3])<1.2
    else:
        m=robot.memory.bug2param[2]
        c=robot.memory.bug2param[3]
        dist=np.abs(robot.position.y-m*robot.position.x-c)/(np.sqrt(1+m*m))
        return dist<1.2
    
def bug2(robot,bump_sensors,target_position):
    if robot.memory.bug2param[0]==1:
        robot.memory.bug2param[0]=0
        calculate_mline(robot,target_position)

    if robot.fsm == "m2g":
        if any(sensor.activated for sensor in bump_sensors) and not robot.memory.bug2param[5]:
            robot.fsm = "wf"
            dmin=np.linalg.norm(target_position-np.array([robot.position.x,robot.position.y]))
            robot.memory.bug2param[4]=dmin
        else:
            robot.fsm = "m2g"
            robot.memory.bug2param[5]=0
    elif robot.fsm == "wf":
        d=np.linalg.norm(target_position-np.array([robot.position.x,robot.position.y]))
        if check_mline_hit(robot) and d<robot.memory.bug2param[4] and check_obstacle(robot.position,target_position):
            robot.fsm = "m2g"
            robot.memory.bug2param[5]=1
        else:
            robot.fsm = "wf"

##################################TANGENT BUG###########################
# Tangent Bug works from the lidar readings. While the way to the goal is free the
# robot drives straight at it. When an obstacle blocks the goal it heads for the
# obstacle edge (tangent point) with the shortest estimated path, and when that
# estimate stops improving (local minimum) it follows the boundary. It leaves the
# boundary once the lidar sees a point closer to the goal than anything seen on the
# followed obstacle. The heading is handed to the controller through memory.Oi.
TB_CLEARANCE = 5.0     # cm kept free on both sides of the robot's path
TB_FOLLOW_DIST = 8.0   # cm distance kept to the wall during boundary following
TB_JUMP = 4.0          # cm range jump between two beams that counts as an edge
TB_LEAVE_MARGIN = 2.0  # cm the goal distance must improve before leaving the wall

def _wrap(angle):
    return (angle + 180) % 360 - 180

def _tb_state(robot):
    if not hasattr(robot.memory, "tb"):
        robot.memory.tb = {"h_best": np.inf, "side": 1, "d_followed": np.inf,
                           "entry": None, "prev": None, "travel": 0.0}
    return robot.memory.tb

def _goal_blocked(r, ang, hit, dist_goal, goal_ang):
    diff = np.radians(_wrap(ang - goal_ang))
    ahead = np.abs(diff) < np.pi / 2
    along = r * np.cos(diff)
    lateral = r * np.abs(np.sin(diff))
    return bool(np.any(hit & ahead & (lateral < TB_CLEARANCE) & (along < dist_goal)))

def _tangent_candidates(r, ang, hit, p, q):
    # An edge is a hit/no-hit change or a large range jump between neighbouring beams.
    n = len(r)
    candidates = []
    for i in range(n):
        j = (i + 1) % n
        if hit[i] != hit[j] or (hit[i] and abs(r[i] - r[j]) > TB_JUMP):
            near, far = (i, j) if r[i] <= r[j] else (j, i)
            side = 1 if far == j else -1  # which way the free space lies
            delta = np.degrees(np.arcsin(min(1.0, TB_CLEARANCE / max(r[near], TB_CLEARANCE)))) + 1
            heading = ang[near] + side * delta
            point = p + r[near] * np.array([np.cos(np.radians(heading)), np.sin(np.radians(heading))])
            candidates.append((r[near] + np.linalg.norm(point - q), heading, side))
    return candidates

def tangent_bug(robot, lidar_sensor, target_position):
    tb = _tb_state(robot)
    p = np.array([robot.position.x, robot.position.y], dtype=float)
    q = np.asarray(target_position, dtype=float)
    r = np.array(lidar_sensor.readings, dtype=float)
    ang = lidar_sensor.orientation + np.arange(len(r)) * 360.0 / len(r)
    hit = r < lidar_sensor.max_range - 1e-6
    points = p + r[:, None] * np.column_stack((np.cos(np.radians(ang)), np.sin(np.radians(ang))))
    d_points = np.linalg.norm(points - q, axis=1)
    dist_goal = np.linalg.norm(q - p)
    goal_ang = np.degrees(np.arctan2(q[1] - p[1], q[0] - p[0]))

    if robot.fsm == "m2g":
        if not _goal_blocked(r, ang, hit, dist_goal, goal_ang):
            tb["h_best"] = np.inf
            robot.memory.Oi = q.copy()
            return
        candidates = _tangent_candidates(r, ang, hit, p, q)
        if candidates:
            h, heading, side = min(candidates, key=lambda c: c[0])
            if h <= tb["h_best"] + 1.0:
                tb["h_best"] = min(tb["h_best"], h)
                # driving round the left of the goal line keeps the obstacle on the right
                tb["side"] = 1 if _wrap(heading - goal_ang) > 0 else -1
                robot.memory.Oi = p + 10 * np.array([np.cos(np.radians(heading)), np.sin(np.radians(heading))])
                return
        # local minimum (or fully enclosed): switch to boundary following
        robot.fsm = "wf"
        tb["d_followed"] = d_points[hit].min() if hit.any() else dist_goal
        tb["entry"] = None  # reference point on the wall, set once the robot has settled
        tb["prev"] = p.copy()
        tb["travel"] = 0.0

    if robot.fsm == "wf":
        tb["travel"] += np.linalg.norm(p - tb["prev"])
        tb["prev"] = p.copy()
        if hit.any():
            tb["d_followed"] = min(tb["d_followed"], d_points[hit].min())
        if d_points.min() < tb["d_followed"] - TB_LEAVE_MARGIN or not hit.any():
            robot.fsm = "m2g"
            tb["h_best"] = np.inf
            tangent_bug(robot, lidar_sensor, target_position)
            return
        if tb["entry"] is None:
            if tb["travel"] > 30:
                tb["entry"] = p.copy()
                tb["travel"] = 0.0
        elif tb["travel"] > 60 and np.linalg.norm(p - tb["entry"]) < 5:
            # back at the reference point without a better exit: the goal is unreachable
            robot.fsm = "noSoln"
            return
        nearest = int(np.argmin(r))
        err = np.clip((r[nearest] - TB_FOLLOW_DIST) * 4, -45, 45)
        heading = ang[nearest] + tb["side"] * (90 - err)
        robot.memory.Oi = p + 10 * np.array([np.cos(np.radians(heading)), np.sin(np.radians(heading))])
