import rclpy
import threading
import numpy as np

from salvos_control import ros_nodes as nodes
from salvos_control import manual_control as mc
from salvos_control.plotter import RealtimePlotter
from salvos_control import control as ctrl
from salvos_control import estimator as estimator

import pygame
import time
import random
import math
import sys





def main(args=None):
    rclpy.init(args=args)

    executor = rclpy.executors.MultiThreadedExecutor()

    lidar_subscriber = nodes.LidarSubscriber()
    imu_subscriber = nodes.ImuSubscriber()
    mag_subscriber = nodes.MagnetometerSubscriber()
    baro_subscriber = nodes.BaroSubscriber()
    gps_subscriber = nodes.GPSSubscriber()
    pose_subscriber = nodes.PoseSubscriber()
    clock_subscriber = nodes.ClockSubscriber()

    motor_cmd = [0, 0, 0, 0, 0]

    control = ctrl.Controller()
    timestep = 0

    rotors_publisher = nodes.RotorsPublisher(motor_cmd)


    executor.add_node(lidar_subscriber)
    executor.add_node(imu_subscriber)
    executor.add_node(rotors_publisher)
    executor.add_node(mag_subscriber)
    executor.add_node(baro_subscriber)
    executor.add_node(gps_subscriber)
    executor.add_node(pose_subscriber)
    executor.add_node(clock_subscriber)


    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()


    while rclpy.ok() and (
        clock_subscriber.data is None or
        imu_subscriber.data is None or
        not lidar_subscriber.lidar_ranges
    ):
        time.sleep(0.05)



    pygame.init()
    pygame.display.set_caption("Window")
    screen = pygame.display.set_mode((300, 600))

    try:
        clock = pygame.time.Clock()
        control_keys = ['w', 'a', 's', 'd', 'i', 'p', 'o', 'l', 'r', 'm', 'n', 't']
        t = clock_subscriber.data.clock.nanosec


        sensor_data = nodes.get_sensor_data(imu=imu_subscriber.data,
                                           mag=mag_subscriber.data,
                                           lidar=min(lidar_subscriber.lidar_ranges),
                                           baro=baro_subscriber.data,
                                           pos=[0, 0, 0])

        #state: [x,y,z, vx,vy,vz, roll,pitch,yaw, p,q,r]
        state = nodes.init_state(sensor_data)

        plot = RealtimePlotter(history_len=200, px_per_panel=480)
        nodes.set_wind(random.randrange(-25, 25), random.randrange(-25, 25), 0)

        pos = [state['x'], state['y'], state['z']]

        run = True
        reset = False
        use_manual_control = False
        land = False
        landing = False
        landed = False
        achieved_target = False
        hover_height = 20
        landing_min_dist = 0.9 #can be smaller, need better AGL estimation
        epsilon = 1 #tune

        manual_control = mc.ManualControl(height=hover_height)

        while run:

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    raise KeyboardInterrupt

            keys_pressed = manual_control.detect_keys(control_keys)

            if pygame.key.get_pressed()[pygame.K_ESCAPE]:
                break



            sensor_data = nodes.get_sensor_data(imu=imu_subscriber.data,
                                                mag=mag_subscriber.data,
                                                lidar=min(lidar_subscriber.lidar_ranges),
                                                baro=baro_subscriber.data,
                                                pos=pose_subscriber.data)


            dt = clock_subscriber.data.clock.nanosec - t
            if dt < 0: dt = 50000000

            state = estimator.state_estimate(sensor_data, state, dt=dt*1e-9)


            if land:
                state['z'] = min(lidar_subscriber.lidar_ranges)

            if state['z'] < 0: state['z'] = 0

            pos = [state['x'], state['y'], state['z']]
            user_cmd = manual_control.keys_to_cmd(keys_pressed)
            manual_control.height = hover_height

            if timestep == 0:
                target = [pos[0], pos[1], hover_height]


            if keys_pressed['r']:
                reset = True


            if keys_pressed['m']:
                use_manual_control = not use_manual_control
                if not use_manual_control:
                    target = [pos[0], pos[1], hover_height]

                print("MANUAL CONTROL: ", use_manual_control)


            if keys_pressed['n']:
                if landing:
                    target = [pos[0], pos[1], 0]
                land = True
                use_manual_control = False
                print("LAND: ", land)
                target = pos

            
            if landed and keys_pressed['t']:
                target = [pos[0], pos[1], hover_height]
                landing = False
                landed = False
                use_manual_control = False


            if land:
                if achieved_target:
                    epsilon = 0.1
                    landing = True
                    target = [pos[0], pos[1], 0]

            if landing and pos[2] < landing_min_dist:
                landed = True
                land = False
                landing = False


            cmd = user_cmd + target

            if landed:
                motor_cmd = [0, 0, 0, 0, 0]
            else:
                motor_cmd = control.logic(state, cmd, use_manual_control, dt=dt*1e-9)

            achieved_target = control.achieved_target(target, epsilon)



            rotors_publisher.set_cmd(motor_cmd)


            timestep += 1

            #info logging/plotting for debugging
            print('\nPOSITION: ', pos)
            print('TARGET: ', target)
            print('DISTANCE TO TARGET: ', math.dist(pos, target))
            print('ACHIEVED TARGET: ',achieved_target)
            print('-------------------------------\n')

            try:
                plot.push(control.plot('x'))
                plot.push(control.plot('y'))
                plot.push(control.plot('z'))
            except AttributeError:
                print('No output for pid')


            plot.draw(screen)


            if reset:
                reset = False
                timestep = 0
                sensor_data = nodes.get_sensor_data(imu=imu_subscriber.data,
                                                    mag=mag_subscriber.data,
                                                    lidar=min(lidar_subscriber.lidar_ranges),
                                                    baro=baro_subscriber.data,
                                                    pos=pose_subscriber.data)
                dt = abs(clock_subscriber.data.clock.nanosec - t)
                state = estimator.state_estimate(sensor_data, state, dt=dt*1e-9)
                motor_cmd = [0, 0, 0, 0, 0]
                rotors_publisher.set_cmd(motor_cmd)
                nodes.reset()
                control.__init__()
                nodes.set_wind(random.randrange(-25, 25), random.randrange(-25, 25), 0)
                time.sleep(2.5)


            t = clock_subscriber.data.clock.nanosec



            pygame.display.flip()
            clock.tick(20) 

    except KeyboardInterrupt:
        pass

    finally:
        pygame.quit()
        executor.shutdown()
        ros_thread.join(timeout=1.0)
        rclpy.shutdown()



if __name__ == '__main__':
    main()
