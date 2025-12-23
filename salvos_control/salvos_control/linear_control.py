import math
import numpy as np
from scipy.spatial.transform import Rotation as Rt
import pid as pid




class Controller():

	def __init__(self):
		self.state = []
		self.motor_speeds = [0, 0, 0, 0, 0] #[nose, rr, rl, fl, fr] (form)
		self.cmd_names = ['thrust', 'roll', 'pitch', 'yaw', 'height', 'x', 'y']


		inner_loop_tau = 0.00002
		outer_loop_tau = 0.000002
		#k = [18733.45, 4253662.569, 69.8762]
		#k = [452.639, 512210.58, 0]

		k_angle = [1, 0, 0]
		k_rate = [1.5, 0.01, 0.2]

		self.ixx = 148.894
		self.iyy = 117.926
		self.izz = 48.623

		self.R_wb = None

		#position pids
		self.height_pid = pid.PID(kp=0.8, ki=0.0, kd=0.0, tau=outer_loop_tau, out_min=-10, out_max=6)
		self.thrust_pid = pid.PID(kp=2, ki=0.1, kd=0.4, tau=inner_loop_tau, out_min=-15, out_max=15)


		#attitude pids
		#outer loop
		self.roll_pid = pid.PID(kp=k_angle[0], ki=k_angle[1], kd=k_angle[2], tau=outer_loop_tau, out_min=-2.0944, out_max=2.0944)
		self.pitch_pid = pid.PID(kp=k_angle[0], ki=k_angle[1], kd=k_angle[2], tau=outer_loop_tau, out_min=-1.745, out_max=1.745)
		self.yaw_pid = pid.PID(kp=k_angle[0], ki=k_angle[1], kd=k_angle[2], tau=outer_loop_tau, out_min=-math.pi, out_max=math.pi)

		#inner loop
		self.p_pid = pid.PID(kp=self.ixx*k_rate[0], ki=self.ixx*k_rate[1], kd=self.ixx*k_rate[2], tau=inner_loop_tau, out_min=-200, out_max=200)
		self.q_pid = pid.PID(kp=self.iyy*k_rate[0], ki=self.iyy*k_rate[1], kd=self.iyy*k_rate[2], tau=inner_loop_tau, out_min=-200, out_max=200)
		self.r_pid = pid.PID(kp=self.izz*k_rate[0], ki=self.izz*k_rate[1], kd=self.izz*k_rate[2], tau=inner_loop_tau, out_min=-100000, out_max=100000)


		self.pids = {
			'thrust': self.thrust_pid,
			'roll': self.roll_pid,
			'pitch': self.pitch_pid,
			'yaw': self.yaw_pid,
			'p': self.p_pid,
			'q': self.q_pid,
			'r': self.r_pid
			}



	def motor_mixer(self, u):
		kt = 0.0111
		kq = 0.00170
		rp = 0.875
		fp = 0.925



		#(form)   B = [nose, rr, rl, fl, fr]
		B = np.array( [[kt, kt, kt, kt, kt],
					  [0, -rp*kt, rp*kt, fp*kt, -fp*kt],
					  [0, rp*kt, rp*kt, -fp*kt, -fp*kt],
					  [kq, -kq, kq, -kq, kq]])


		M = B.T @ np.linalg.inv(B @ B.T)

		s = np.clip(M @ u, 0.0, None)

		motor_speeds = np.sqrt(s)

		return motor_speeds


	def set_state(self, state):
		self.state = state
		#self.state_pos = self.state[0:5]
		#self.state_att = self.state[5:]

		return self.state


	def set_cmd(self, cmd):
		self.cmd = {k: v for k, v in zip(self.cmd_names, cmd)}

		return self.cmd




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




	def attitude_control(self, roll_ref, pitch_ref, yaw_ref, dt):

		self.R_wb = self.R_mat(self.state['roll'], self.state['pitch'], self.state['yaw'])
		
		R_d = self.R_mat(roll_ref, pitch_ref, yaw_ref)

		E = R_d.T @ self.R_wb - self.R_wb.T @ R_d

		e_R = self.vee(E).tolist()

		p = self.roll_pid.compute(e_R[0], 0.0, dt)
		q = self.pitch_pid.compute(e_R[1], 0.0, dt)
		r = self.yaw_pid.compute(e_R[2], 0.0, dt)

		tx = self.p_pid.compute(p, self.state['p'], dt)
		ty = self.q_pid.compute(q, self.state['q'], dt)
		tz = self.r_pid.compute(r, self.state['r'], dt)

		return tx, ty, tz


	def altitude_control(self, z_ref, dt):

		g = 9.81
		m = 263.7
		z = np.array([0, 0, 1])

		u_b = np.array([0, 0, 1.0])
		kt = z @ (self.R_wb @ u_b)

		#print('kt: ', kt)

		kt_min = 0.000001

		kt = max(kt, kt_min)

		vz_cmd = self.height_pid.compute(z_ref, self.state['z'], dt)
		az_cmd = self.thrust_pid.compute(vz_cmd, self.state['vz'], dt)

		T = m*(g + az_cmd) / kt

		return T



	def plot(self, name):
		c = self.pids[name]

		return {f'{name} target': c.target, f'{name} output': c.output}



	def logic(self, state, cmd, dt):

		self.set_state(state)
		self.set_cmd(cmd)

		tx, ty, tz = self.attitude_control(self.cmd['roll'], self.cmd['pitch'], self.cmd['yaw'], dt)

		T = self.altitude_control(self.cmd['height'], dt)


		#u = np.array([T, tx, ty, tz])
		u = np.array([T, tx, ty, tz])

		self.motor_speeds = self.motor_mixer(u.T)

		#print(self.motor_speeds, '\n ----------\n motor_speeds:')
		#print(u, '\n ----------\n u:')
		#print([self.state['vx'], self.state['vy'], self.state['vz']], '\n ----------\n velocity (x,y,z):')
		print([self.state['x'], self.state['y'], self.state['z']], '\n ----------\n position (x,y,z):')


		return self.motor_speeds

