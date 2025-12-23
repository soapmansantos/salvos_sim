import rclpy
import threading
from rclpy.node import Node
from actuator_msgs.msg import Actuators
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan, Imu, MagneticField, FluidPressure, NavSatFix
from geometry_msgs.msg import Pose, Transform, TransformStamped
from tf2_msgs.msg import TFMessage
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.srv import ControlWorld
from scipy.spatial.transform import Rotation as Rt
import numpy as np
import subprocess



class RotorsPublisher(Node):

	def __init__(self, cmd):
		super().__init__('rotors_publisher')
		self.pub = self.create_publisher(Actuators, '/hermes/actuators', 10)
		self.cmd = cmd
		self.msg = Actuators()

		rate_hz = 50
		period = 1.0 / float(rate_hz)
		self.timer = self.create_timer(period, self.timer_callback)


	def timer_callback(self):
		self.msg.velocity = list(map(float, self.cmd))
		self.pub.publish(self.msg)
		#self.get_logger().info('Publishing: "%s"' % self.msg.velocity)


	def set_cmd(self, cmd):
		self.cmd = cmd
		self.msg.velocity = list(map(float, self.cmd))




class LidarSubscriber(Node):

	def __init__(self):
		super().__init__('lidar_subscriber')
		self.lidar_subscriber = self.create_subscription(LaserScan, '/lidar', self.listener_callback, 10)
		self.lidar_ranges = []


	def listener_callback(self, msg: LaserScan):
		self.lidar_ranges = msg.ranges[:]
		#self.get_logger().info('Distance to ground: "%s"' % min(self.lidar_ranges))



class ImuSubscriber(Node):

	def __init__(self):
		super().__init__('imu_subscriber')        
		self.imu_subscriber = self.create_subscription(Imu, '/imu', self.listener_callback, qos_profile_sensor_data)


	def listener_callback(self, msg: Imu):
		self.data = msg
		#self.get_logger().info('Linear acceleration: "%s"' % self.data.linear_acceleration)



class MagnetometerSubscriber(Node):
	def __init__(self):
		super().__init__('magnetometer_subscriber')
		self.sub = self.create_subscription(
			MagneticField, '/hermes/mag', self.listener_callback, qos_profile_sensor_data
		)
		self.data = None

	def listener_callback(self, msg: MagneticField):
		self.data = msg
		# self.get_logger().info(f'Mag field (T): x={msg.magnetic_field.x:.3e}, y={msg.magnetic_field.y:.3e}, z={msg.magnetic_field.z:.3e}')



class BaroSubscriber(Node):
	def __init__(self):
		super().__init__('air_pressure_subscriber')
		self.sub = self.create_subscription(
			FluidPressure, '/hermes/air_pressure', self.listener_callback, qos_profile_sensor_data
		)
		self.data = None

	def listener_callback(self, msg: FluidPressure):
		self.data = msg
		# self.get_logger().info(f'Pressure (Pa): {msg.fluid_pressure:.2f}')



class GPSSubscriber(Node):
	def __init__(self):
		super().__init__('gps_subscriber')
		self.sub = self.create_subscription(
			NavSatFix, '/hermes/gps', self.listener_callback, qos_profile_sensor_data
		)
		self.data = None

	def listener_callback(self, msg: NavSatFix):
		self.data = msg
		#self.get_logger().info(f'GPS: lat={msg.latitude:.7f}, lon={msg.longitude:.7f}, alt={msg.altitude:.2f}')


class ClockSubscriber(Node):
	def __init__(self):
		super().__init__('clock_subscriber')
		self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
		self.sub = self.create_subscription(
			Clock, '/clock', self.listener_callback, qos_profile_sensor_data
		)
		self.data = None

	def listener_callback(self, msg: Clock):
		self.data = msg
		t = msg.clock
		#self.get_logger().info(f'Sim Time → {t.sec}.{t.nanosec:09d} s')



class PoseSubscriber(Node):
	def __init__(self):
		super().__init__('pose_subscriber')
		self.sub = self.create_subscription(TFMessage, '/hermes/pose', self.listener_callback, 10)
		self.tf = None
		self.pos = None
		self.heading = 0.0
		self.data = []
		self.count = 0 #cycles through 7

	def listener_callback(self, msg: TFMessage):
		self.tf = msg.transforms#[msg.position.x, msg.position.y]
		self.pose_listener_callback(self.tf[0])


	def pose_listener_callback(self, msg: TransformStamped):
		self.pos = msg.transform.translation
		self.heading = msg.transform.rotation.z

		self.data = [self.pos.x, self.pos.y, self.pos.z]




class WorldResetClient(Node):
	def __init__(self):
		super().__init__('world_reset_client')
		self.cli = self.create_client(ControlWorld, '/world/empty/control')
		while not self.cli.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Waiting for /world/empty/control service...')

	def reset_all(self):
		req = ControlWorld.Request()
		req.world_control.reset.all = True
		future = self.cli.call_async(req)
		rclpy.spin_until_future_complete(self, future)
		if future.result() is None:
			raise RuntimeError(f"Service call failed: {future.exception()}")
		return future.result()



def reset():
	node = WorldResetClient()
	resp = node.reset_all()
	node.get_logger().info(f"Reset returned: {resp}")
	node.destroy_node()



def set_wind(x, y, z):
    cmd = [
        "gz", "topic",
        "-t", "/world/empty/wind",
        "-m", "gz.msgs.Wind",
        "-p", f"enable_wind: true, linear_velocity: {{x: {x}, y: {y}, z: {z}}}"
    ]
    subprocess.run(cmd, check=True)



def get_sensor_data(imu, mag, lidar, baro, pos):
	"""
	return dict with:
		'imu': {'orientatiosn':[x,y,z,w], 'angular_velocity':[p,q,r], 'linear_acceleration':[ax,ay,az]}
		'mag': {'mx','my','mz'}
		'baro': {'altitude'}               # absolute altitude
		'lidar': {'distance'}              # distance to ground
		'pos': {'x','y','z'}               # position (odometry)
	"""

	o = imu.orientation
	av = imu.angular_velocity
	la = imu.linear_acceleration
	m = mag.magnetic_field

	imu_data = {'orientation': [o.x, o.y, o.z, o.w],
				'angular_velocity': [av.x, av.y, av.z],
				'linear_acceleration': [la.x, la.y, la.z]}
	mag_data = {'mx': m.x, 'my': m.y, 'mz': m.z}
	baro_data = {'pressure': baro.fluid_pressure}
	lidar_data = {'distance': lidar}
	pos_data = {'x': pos[0], 'y': pos[1], 'z': pos[2]}


	sensor_data = {'imu': imu_data, 'mag': mag_data, 'baro': baro_data, 'lidar': lidar_data, 'pos': pos_data}


	sensor_data = {
		k: {kk: (round(vv, 3) if isinstance(vv, (int, float)) else [round(x, 3) for x in vv])
			for kk, vv in v.items()}
		for k, v in sensor_data.items()
	}


	return sensor_data






def init_state(sensors):
	x0 = sensors['pos']['x']
	y0 = sensors['pos']['y']

	P = sensors['baro']['pressure']  # Pa
	P0, T0, g, L, Rg, M = 101325.0, 288.15, 9.80665, 0.0065, 8.31447, 0.0289644
	baro_alt = (T0/L) * (1 - (P/P0)**((Rg*L)/(g*M)))

	# Lidar gives AGL; fuse with GPS z (AMSL/local) for an initial z
	z0 = 0.6*baro_alt + 0.4*(sensors['pos']['z'] - sensors['lidar']['distance'])

	z0 = sensors['lidar']['distance']

	# Attitude from IMU quaternion, yaw lightly corrected by magnetometer
	roll0, pitch0, yaw_imu = Rt.from_quat(sensors['imu']['orientation']).as_euler('xyz', False)
	yaw_mag = np.arctan2(sensors['mag']['my'], sensors['mag']['mx'])
	alpha = 1
	yaw0 = alpha*yaw_imu + (1-alpha)*yaw_mag

	# Start at rest
	vx0 = vy0 = vz0 = 0.0
	p0, q0, r0 = sensors['imu']['angular_velocity']

	names = ['x', 'y', 'z', 'vx', 'vy', 'vz', 'roll', 'pitch', 'yaw', 'p', 'q', 'r']

	state = np.array([x0, y0, z0, vx0, vy0, vz0, roll0, pitch0, yaw0, p0, q0, r0])
	state_dict = {name: value for name, value in zip(names, state)}


	return state_dict