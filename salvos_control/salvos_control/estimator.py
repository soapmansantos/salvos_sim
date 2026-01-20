import math
import numpy as np
from scipy.spatial.transform import Rotation as Rt


def state_estimate(sensors, prev_state, dt):
    """
    sensors: dict with:
        'imu': {'orientation':[x,y,z,w], 'angular_velocity':[p,q,r], 'linear_acceleration':[ax,ay,az]}
        'mag': {'mx','my','mz'}
        'baro': {'altitude'}               # absolute altitude
        'lidar': {'distance'}              # distance to ground
        'pos': {'x','y','z'}               # position (from VIO algorithm)
    prev_state: np.array([x,y,z, vx,vy,vz, roll,pitch,yaw, p, q, r])
    dt: timestep (s)
    """
    names = ['x','y','z','vx','vy','vz','roll','pitch','yaw','p','q','r']

    g = np.array([0, 0, -9.81])
    x, y, z, vx, vy, vz, roll, pitch, yaw, p, q, r = (prev_state[n] for n in names)

    quat = sensors['imu']['orientation']
    roll, pitch, yaw_imu = Rt.from_quat(quat).as_euler('xyz', degrees=False)
    p, q, r = sensors['imu']['angular_velocity']

    mx, my, mz = sensors['mag']['mx'], sensors['mag']['my'], sensors['mag']['mz']
    yaw_mag = np.arctan2(my, mx)

    alpha_yaw = 1
    yaw = alpha_yaw * yaw_imu + (1 - alpha_yaw) * yaw_mag

    acc_body = np.array(sensors['imu']['linear_acceleration'])
    Rwb = Rt.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
    acc_world = Rwb @ acc_body + g

    pos = sensors['pos']
    x, y, z = pos['x'], pos['y'], pos['z']

    lidar_dist = sensors['lidar']['distance']

    z_prev = z

    alpha_vz = 1#0.98

    vx += acc_world[0] * dt
    vy += acc_world[1] * dt
    vz += alpha_vz*(acc_world[2] * dt)# + ((1-alpha_vz)*(z-z_prev))/dt



    state = np.array([x, y, z, vx, vy, vz, roll, pitch, yaw, p, q, r])
    state_dict = {name: value for name, value in zip(names, state)}


    return state_dict