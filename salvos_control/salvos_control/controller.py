import math
import numpy as np
from scipy.spatial.transform import Rotation as Rt
np.set_printoptions(legacy='1.25')




class Controller():

    def __init__(self): 
        self.lin_acc = (0.0, 0.0, 0.0) #(x, y, z)
        self.ang_vel = (0.0, 0.0, 0.0)
        self.heading = 0.0 #angle

        self.motor_speeds = [0, 0, 0, 0, 0] #[nose, rr, rl, fl, fr] (form)
        self.t_hover = 0 #164 #185
        self.thrust = 0 #rotor angular speed (r)
        self.height = 0
        self.max_rotor_speed = 378
        self.cmd_names = ['thrust', 'roll', 'pitch', 'yaw', 'height', 'x', 'y']
        self.cmd = {k:0.0 for k in self.cmd_names}

        self.tau = 0.1
        outer_loop_tau = 0.000000000001
        inner_loop_tau = 0.000000000001
        self.R_wb = None

        #position control
        self.x_pid = PID(kp=0, ki=0, kd=0, tau=outer_loop_tau, out_min=5, out_max=5)
        self.y_pid = PID(kp=0, ki=0, kd=0, tau=outer_loop_tau, out_min=-5, out_max=5)

        self.izz = 48.623
        self.iyy = 117.926
        self.ixx = 148.894

        k = [18733.45, 4253662.569, 69.8762]
        k1 = [452.639, 51220.58, 0]
        #k1 = [452.639, 0, 0]
        k1 = [201.173, 10117.64, 0]

        #outer loop
        self.roll_pid = PID(kp=self.ixx*k[0], ki=self.ixx*k[1], kd=self.ixx*k[2], tau=outer_loop_tau, out_min=-2.0944, out_max=2.0944)
        self.pitch_pid = PID(kp=self.iyy*k[0], ki=self.iyy*k[1], kd=self.iyy*k[2], tau=outer_loop_tau, out_min=-1.745, out_max=1.745)
        self.yaw_pid = PID(kp=self.izz*k[0], ki=self.izz*k[1], kd=self.izz*k[2], tau=outer_loop_tau, out_min=-math.pi, out_max=math.pi)
        self.height_pid = PID(kp=1, ki=0.0, kd=0.0, tau=outer_loop_tau, out_min=-10, out_max=10)

        #inner loop
        self.thrust_pid = PID(kp=1, ki=0.4, kd=0.2, tau=inner_loop_tau, out_min=-8, out_max=4.9)
        #self.p_pid = PID(kp=15, ki=1, kd=5, tau=inner_loop_tau, out_min=-360, out_max=360)
        self.p_pid = PID(kp=self.ixx*k1[0], ki=self.ixx*k1[1], kd=self.ixx*k1[2], tau=inner_loop_tau, out_min=-50, out_max=50)
        self.q_pid = PID(kp=self.iyy*k1[0], ki=self.iyy*k1[1], kd=self.iyy*k1[2], tau=inner_loop_tau, out_min=-50, out_max=50)
        self.r_pid = PID(kp=self.izz*k1[0], ki=self.izz*k1[1], kd=self.izz*k1[2], tau=inner_loop_tau, out_min=-100, out_max=100)


        self.pids = {
            'x': self.x_pid,
            'y': self.y_pid,
            'roll': self.roll_pid,
            'pitch': self.pitch_pid,
            'yaw': self.yaw_pid,
            'height': self.height_pid,
            'thrust': self.thrust_pid,
            'p': self.p_pid,
            'q': self.q_pid,
            'r': self.r_pid}


        self.lpf = FirstOrderLPF(tau=self.tau)
        self.test_int_error = 0.0
        self.run = True

    def set_state(self, state):
        self.state = state

    def set_cmd(self, cmd):
        self.cmd = {k: v for k, v in zip(self.cmd_names, cmd)}

        return self.cmd




    def motor_mixer(self, cmd, k_f=0.0111, k_m=0.00170, n_rotors=5):
        """
        tx = math.cos(self.state['pitch'])
        ty = math.sin(self.state['pitch'])

        #cmd_tf = cmd.copy()
        #cmd_tf[1] = tx*cmd[1]*0.98 + (1-tx)*cmd[3]
        #cmd_tf[3] = ty*cmd[1] + (1-ty)*cmd[3]
        #cmd_tf[3] = tx*cmd[3] + (1-tx)*cmd[1]
        """

        L_nose = 2.5
        L_quad = 1.7493

        mixer_mat = np.array([ [1, 0, 0, 1],   #nose
                               [1, -1, 1, -1],#rr
                               [1, 1, 1, 1],   #rl
                               [1, 1, -1, -1],  #fl
                               [1, -1, -1, 1] ]) #fr

        cmd_mat = np.array(cmd).reshape(-1, 1)
        cmd_arr = np.matmul(mixer_mat, cmd_mat).flatten().tolist()

        #print('CCW: ', cmd_arr[1] + cmd_arr[3])
        #print('CW: ', cmd_arr[0] + cmd_arr[2] + cmd_arr[4])

        print('cmd_arr: ', cmd_arr)

        #turn torques into angular velocities (rad/s)
        self.motor_speeds = [max(np.clip(np.sqrt(max(i, 0)/(k_f*n_rotors)), 0.0, self.max_rotor_speed), 0.0) for i in cmd_arr]


        if any([math.isnan(i) for i in self.motor_speeds]):
            print('motor_speeds: ', self.motor_speeds)
            self.run = False


        return self.motor_speeds




    def R_mat(self, phi, theta, psi):
        cphi = np.cos(phi)
        sphi = np.sin(phi)
        cth = np.cos(theta)
        sth = np.sin(theta)
        cpsi = np.cos(psi)
        spsi = np.sin(psi)

        return np.array([
            [ cth*cpsi,                      cth*spsi,                     -sth     ],
            [ sphi*sth*cpsi - cphi*spsi,     sphi*sth*spsi + cphi*cpsi,    sphi*cth ],
            [ cphi*sth*cpsi + sphi*spsi,     cphi*sth*spsi - sphi*cpsi,    cphi*cth ]
        ])


    def vee(self, S):
        return np.array([
            S[2,1] - S[1,2],
            S[0,2] - S[2,0],
            S[1,0] - S[0,1]
        ]) * 0.5


    def position_pid(self, x, y, dt):

        self.heading = self.state['yaw']
        print('heading: ', self.heading)


        if self.heading > math.pi/2:
            self.heading = self.heading - math.pi/2
        elif self.heading < -math.pi/2:
            self.heading = self.heading + math.pi/2

        self.heading = self.state['yaw']

        roll_cmd = abs(math.sin(self.heading))*self.x_pid.compute(x, self.state['x'], dt)
        pitch_cmd = abs(math.cos(self.heading))*self.y_pid.compute(y, self.state['y'], dt)

        print('x: ', self.state['x'])
        print('y: ', self.state['y'])

        return 0, 0 #roll_cmd, pitch_cmd



    def attitude_pid(self, height_ref, roll_ref, pitch_ref, yaw_ref, dt):

        self.R_wb = self.R_mat(self.state['roll'], self.state['pitch'], self.state['yaw'])
        
        R_d = self.R_mat(roll_ref, pitch_ref, yaw_ref)

        E = R_d.T @ self.R_wb - self.R_wb.T @ R_d

        e_R = self.vee(E)

        self.height = self.height_pid.compute(height_ref, self.state['z'], dt)

        self.roll = self.roll_pid.compute(roll_ref, self.state['roll'], dt)
        self.pitch = self.pitch_pid.compute(pitch_ref, self.state['pitch'], dt)
        self.yaw = self.yaw_pid.compute(yaw_ref, self.state['yaw'], dt)


        #cmd = [self.height, self.roll, self.pitch, self.yaw]
        cmd = [self.height] + e_R.tolist()

        return cmd




    def rate_pid(self, vz_ref, p_ref, q_ref, r_ref, dt):
        az_cmd = self.thrust_pid.compute(vz_ref, self.state['vz'], dt)
        """
        print('vz_ref: ', vz_ref)
        print('vz: ', self.state['vz'])
        """

        alpha = math.pi/2 - abs(self.state['pitch'])  #alpha: angle of attack
        if self.state['pitch'] < 0.0:
            alpha = -alpha

        m = 263.0
        g = 9.81
        z = np.array([0, 0, 1])

        u_b = np.array([0, 0, 1.0])
        kt = z @ (self.R_wb @ u_b)

        #print('kt: ', kt)

        kt_min = 0.000001

        kt = max(kt, kt_min)

        T = m*(g + az_cmd) / kt

        #print(z, '\n*', self.R_wb, '\n*', u_b)
        #print('az_cmd: ', az_cmd)


        self.p = self.p_pid.compute(p_ref, self.state['p'], dt)
        self.q = self.q_pid.compute(q_ref, self.state['q'], dt)
        self.r = self.r_pid.compute(r_ref, self.state['r'], dt)

        print('\n---------------------------\n')

        return [T, self.p, self.q, self.r]


    def altitude_pid(self, height_ref, vz_ref, dt):

        return 0



    def plot(self, name):
        c = self.pids[name]

        return {f'{name} target': c.target, f'{name} output': c.output}


    def logic(self, state, cmd, dt):

        self.set_state(state)
        self.set_cmd(cmd)

        #print('user cmd', self.cmd, '\n')

        roll_cmd, pitch_cmd = self.position_pid(self.cmd['x'], self.cmd['y'], dt)


        rate_cmd = self.attitude_pid(self.cmd['height'], roll_cmd + self.cmd['roll'], pitch_cmd + self.cmd['pitch'], self.cmd['yaw'], dt)
        #rate_cmd = [self.lpf.update(i, dt) for i in rate_cmd]


        torque_cmd = self.rate_pid(rate_cmd[0], rate_cmd[1], rate_cmd[2], rate_cmd[3], dt)

        #print('YAW ERROR: ', rate_cmd[3])

        #print('rate_cmd', rate_cmd, '\n')
        #print('torque_cmd', [round(i, 4) for i in torque_cmd])
        """
        #print('state: ', state)
        print('user cmd: ', self.cmd, 'thrust: ', self.thrust)
        print('\nheight: ', self.cmd['height'],', ', self.state['z'], ', height error: ', self.height_pid.error)
        print('\nthrust: ', round(torque_cmd[0], 2))
        print('roll: ', self.cmd['roll'],', ', round(self.state['roll'], 2))
        print('pitch: ', self.cmd['pitch'],', ', round(self.state['pitch'], 2))
        print('yaw: ', self.cmd['yaw'],', ', round(self.state['yaw'], 2))
        """
        motor_cmd = self.motor_mixer(torque_cmd)

        print('\ntorque_cmd', [round(i, 3) for i in torque_cmd])

        #print('\nmotor_cmd', [round(i, 3) for i in motor_cmd])

        return motor_cmd








class PID():

    def __init__(self, kp, ki, kd, tau, out_min, out_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.tau = tau
        self.out_min = out_min
        self.out_max = out_max

        self.target = None
        self.current = None
        self.debug = False
        self.temp = self.ki

        self.error = 0
        self.prev_error = 0
        self.int_error = 0
        self.der_error = 0
        self.saturation = False
        self.clamp = False
        self.out = 0

        self.lpf = FirstOrderLPF(self.tau)


    def compute(self, target, current, dt):

        self.target = target
        self.current = current

        self.error = self.target - self.current
        
        if dt > 0.0 and np.isfinite(self.error):
            if self.clamp:
                pass
                #self.ki = 0
            else:
                self.ki = self.temp
                self.int_error += self.error*dt

        self.der_error = (self.error - self.prev_error) / dt

        self.der_error = self.lpf.update(self.der_error, dt)

        self.prev_error = self.error

        raw_output = self.kp*self.error + self.ki*self.int_error + self.kd*self.der_error

        self.output = np.clip(raw_output, self.out_min, self.out_max)

        if self.output != raw_output:
            self.saturation = True

        if np.sign(self.error) == np.sign(self.output) and self.saturation:
            self.clamp = True
        else:
            self.clamp = False


        if self.debug:
            print(
                f'Error: {self.error}, Output: {self.output}, Setpoint: {self.target}, '
                f'Current: {self.current}, kp: {self.kp}, kd: {self.kd}, ki: {self.ki}'
            )

        return self.output
	


class FirstOrderLPF:
    def __init__(self, tau):
        self.tau = tau
        self.dt = 0
        self.y = 0.0
        self.alpha = 0.0

    def reset(self, x0=0.0):
        self.y = x0

    def update(self, u, dt):
        self.dt = dt
        self.alpha = self.dt / (self.tau + self.dt)

        if not (self.alpha == 0.0 or self.dt == 0.0):
            self.y += self.alpha * (u - self.y)

        #print('LPF log: ', self.alpha, self.y, self.dt, u)

        return self.y








def estimator(sensors, prev_state, dt):
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
    #z = lidar_dist

    alpha_vz = 1#0.98

    vx += acc_world[0] * dt
    vy += acc_world[1] * dt
    vz += alpha_vz*(acc_world[2] * dt)# + ((1-alpha_vz)*(z-z_prev))/dt



    state = np.array([x, y, z, vx, vy, vz, roll, pitch, yaw, p, q, r])
    state_dict = {name: value for name, value in zip(names, state)}


    return state_dict