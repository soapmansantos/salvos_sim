import math
import numpy as np
from scipy.spatial.transform import Rotation as Rt
from salvos_control import pid as pid
np.set_printoptions(legacy='1.25')


class Controller():

	def __init__(self):
		self.state = []
		self.motor_speeds = [0, 0, 0, 0, 0]  # (form) [nose, rr, rl, fl, fr]

		self.cmd_names = ['thrust', 'roll', 'pitch', 'yaw', 'height', 'x', 'y', 'z', 'vx', 'vy', 'vz']

		inner_loop_tau = 0.005
		outer_loop_tau = 0.000002

		k_angle = [1, 0, 0]
		k_rate  = [1.5, 0.01, 0.2]

		self.ixx = 148.894
		self.iyy = 117.926
		self.izz = 48.623

		self.R_wb = None
		self.m = 263.7 #mass
		self.g = 9.81


		# cascaded attitude: (angle error -> desired rates) then (rate error -> torque)
		self.roll_pid  = pid.PID(kp=k_angle[0], ki=k_angle[1], kd=k_angle[2], tau=outer_loop_tau, out_min=-2.0944, out_max=2.0944)
		self.pitch_pid = pid.PID(kp=k_angle[0], ki=k_angle[1], kd=k_angle[2], tau=outer_loop_tau, out_min=-1.745,  out_max=1.745)
		self.yaw_pid   = pid.PID(kp=k_angle[0], ki=k_angle[1], kd=k_angle[2], tau=outer_loop_tau, out_min=-math.pi, out_max=math.pi)

		self.p_pid = pid.PID(kp=self.ixx*k_rate[0], ki=self.ixx*k_rate[1], kd=self.ixx*k_rate[2], tau=inner_loop_tau, out_min=-200, out_max=200)
		self.q_pid = pid.PID(kp=self.iyy*k_rate[0], ki=self.iyy*k_rate[1], kd=self.iyy*k_rate[2], tau=inner_loop_tau, out_min=-200, out_max=200)
		self.r_pid = pid.PID(kp=self.izz*k_rate[0], ki=self.izz*k_rate[1], kd=self.izz*k_rate[2], tau=inner_loop_tau, out_min=-100000, out_max=100000)

		# hover controller: pos -> vel -> accel -> tilt refs
		self.x_pid  = pid.PID(kp=0.5, ki=0.0001, kd=0, tau=outer_loop_tau, out_min=-20.0, out_max=20.0)
		self.y_pid  = pid.PID(kp=0.5, ki=0.0001, kd=0, tau=outer_loop_tau, out_min=-20.0, out_max=20.0)
		self.z_pid  = pid.PID(kp=0.25, ki=0.00001, kd=0.0, tau=outer_loop_tau, out_min=-50.0, out_max=50.0)

		self.vx_pid = pid.PID(kp=0.05, ki=0.00, kd=0.01, tau=outer_loop_tau, out_min=-2.5, out_max=2.5)
		self.vy_pid = pid.PID(kp=0.05, ki=0.00, kd=0.01, tau=outer_loop_tau, out_min=-2.5, out_max=2.5)
		self.vz_pid = pid.PID(kp=2.3, ki=0.0000000001, kd=0.0, tau=outer_loop_tau, out_min=-3, out_max=3)

		self.phi_max   = math.radians(20.0)
		self.theta_max = math.radians(20.0)

		self.kv_damp = np.array([0.95, 0.8, 0.95])

		self.use_quad_drag = True
		self.rho = 1.225
		self.Cd  = 0.1308
		self.S   = 3.2 * 0.352

		self.pids = {
			'roll':   self.roll_pid,
			'pitch':  self.pitch_pid,
			'yaw':    self.yaw_pid,
			'p':      self.p_pid,
			'q':      self.q_pid,
			'r':      self.r_pid,
			'x':      self.x_pid,
			'y':      self.y_pid,
			'z':      self.z_pid,
			'vx':     self.vx_pid,
			'vy':     self.vy_pid,
			'vz':     self.vz_pid
		}


		self.count = 0


	def motor_mixer(self, u):
		kt = 0.0111
		kq = 0.00170
		rp = 0.875
		fp = 0.925

		# allocation: solve B*s = u for s>=0 (s = per-motor thrust); then speed = sqrt(s)
		B = np.array([[kt,      kt,      kt,      kt,      kt     ],
					  [0,  -rp*kt,   rp*kt,   fp*kt,  -fp*kt     ],
					  [0,   rp*kt,   rp*kt,  -fp*kt,  -fp*kt     ],
					  [kq,    -kq,     kq,    -kq,     kq       ]])

		M = B.T @ np.linalg.inv(B @ B.T)

		s = np.clip(M @ u, 0.0, None)
		motor_speeds = np.sqrt(s)

		return motor_speeds


	def set_state(self, state):
		self.state = state
		return self.state


	def set_cmd(self, cmd):
		self.cmd = {k: v for k, v in zip(self.cmd_names, cmd)}
		return self.cmd


	def R_mat(self, phi, theta, psi):
		cphi = np.cos(phi)
		sphi = np.sin(phi)
		cth  = np.cos(theta)
		sth  = np.sin(theta)
		cpsi = np.cos(psi)
		spsi = np.sin(psi)

		return np.array([
			[ cth*cpsi,                      cth*spsi,                     -sth     ],
			[ sphi*sth*cpsi - cphi*spsi,     sphi*sth*spsi + cphi*cpsi,    sphi*cth ],
			[ cphi*sth*cpsi + sphi*spsi,     cphi*sth*spsi - sphi*cpsi,    cphi*cth ]
		])


	def vee(self, S):
		# vee(S) maps skew matrix -> vector, used for SO(3) attitude error
		return np.array([
			S[2,1] - S[1,2],
			S[0,2] - S[2,0],
			S[1,0] - S[0,1]
		]) * 0.5


	def attitude_control(self, roll_ref, pitch_ref, yaw_ref, dt):

		# attitude error on SO(3): e_R = vee(Rd^T R - R^T Rd)
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

		z = np.array([0, 0, 1])
		u_b = np.array([0, 0, 1.0])

		# kt is basically “how much of body thrust points up in world frame”
		kt = z @ (self.R_wb @ u_b)

		# avoid divide-by-zero when near-horizontal
		kt_min = 0.00001
		kt = max(kt, kt_min)

		vz_cmd = self.z_pid.compute(z_ref, self.state['z'], dt)
		az_cmd = self.vz_pid.compute(vz_cmd, self.state['vz'], dt)

		T = self.m * (self.g + az_cmd) / kt

		return T



	def quad_drag_accel(self, v):
		V = float(np.linalg.norm(v))
		if V < 1e-6:
			return np.zeros(3)
		return -0.5 * self.rho * self.Cd * self.S * V * v / self.m


	def position_control_hover(self, x_ref, y_ref, yaw_ref, dt):

		p = np.array([self.state['x'],  self.state['y'] ])
		v = np.array([self.state['vx'], self.state['vy']])

		vx_cmd = self.x_pid.compute(x_ref, p[0], dt)
		vy_cmd = self.y_pid.compute(y_ref, p[1], dt)

		ax_cmd = self.vx_pid.compute(vx_cmd, v[0], dt)
		ay_cmd = self.vy_pid.compute(vy_cmd, v[1], dt)

		a_xy = np.array([ax_cmd, ay_cmd])

		a_xy -= self.kv_damp[0:2] * v

		if self.use_quad_drag:
			a_xy += self.quad_drag_accel(np.array([v[0], v[1], 0.0]))[0:2]

		# convert desired world-frame accel into body-frame tilt refs (depends on yaw)
		psi = float(self.state['yaw'])
		cpsi = math.cos(psi)
		spsi = math.sin(psi)

		ax_b =  cpsi * a_xy[0] + spsi * a_xy[1]
		ay_b = -spsi * a_xy[0] + cpsi * a_xy[1]

		theta_ref = np.clip(ax_b / self.g, -self.theta_max, self.theta_max)
		phi_ref   = np.clip(-ay_b / self.g, -self.phi_max, self.phi_max)

		return phi_ref, theta_ref, float(yaw_ref)




	def plot(self, name):
		c = self.pids[name]
		return {f'{name} target': c.target, f'{name} output': c.output}



	def achieved_target(self, target, epsilon):

		pos = [self.state['x'], self.state['y'], self.state['z']]
		vel = [self.state['vx'], self.state['vy'], self.state['vz']]

		err = [i-v for i, v in zip(target, pos)]

		mag_err = math.sqrt(sum([abs(i*i) for i in err]))
		mag_vel = math.sqrt(sum([abs(i*i) for i in vel]))

		# require both “close enough” and “settled” (low velocity)
		if mag_vel < epsilon*0.5 and mag_err < epsilon:
			return True
		else:
			return False



	def logic(self, state, cmd, use_manual_control, dt):

		self.set_state(state)
		self.set_cmd(cmd)

		if not use_manual_control:

			roll_ref, pitch_ref, yaw_ref = self.position_control_hover(
				self.cmd['x'], self.cmd['y'], self.cmd['yaw'], dt
			)
			z_ref = self.cmd['z']

		else:
			roll_ref  = self.cmd['roll']
			pitch_ref = self.cmd['pitch']
			yaw_ref   = self.cmd['yaw']
			z_ref    = self.cmd['height']

		tx, ty, tz = self.attitude_control(roll_ref, pitch_ref, yaw_ref, dt)
		T = self.altitude_control(z_ref, dt)

		u = np.array([T, tx, ty, tz])
		self.motor_speeds = self.motor_mixer(u.T)

		return self.motor_speeds
