import rclpy
import threading
import numpy as np

import ros_nodes as nodes
import manual_control as mc
from plotter import RealtimePlotter
import linear_control as ctrl
import estimator as estimator
import pygame
import time
import random





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
    manual_control = mc.ManualControl(height=5, x=0, y=0)

    rotors_publisher = nodes.RotorsPublisher(motor_cmd)


    executor.add_node(lidar_subscriber)
    executor.add_node(imu_subscriber)
    executor.add_node(rotors_publisher)
    executor.add_node(mag_subscriber)
    executor.add_node(baro_subscriber)
    executor.add_node(gps_subscriber)
    executor.add_node(pose_subscriber)
    executor.add_node(clock_subscriber)


    #executor.spin()
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()



    pygame.init()
    pygame.display.set_caption("Window")
    screen = pygame.display.set_mode((900, 600))

    try:
        clock = pygame.time.Clock()
        control_keys = ['w', 'a', 's', 'd', 'i', 'p', 'o', 'l', 'r']
        t = clock_subscriber.data.clock.nanosec


        sensor_data = nodes.get_sensor_data(imu=imu_subscriber.data,
                                           mag=mag_subscriber.data,
                                           lidar=min(lidar_subscriber.lidar_ranges),
                                           baro=baro_subscriber.data,
                                           pos=[0, 0, 0])

        #state: [x,y,z, vx,vy,vz, roll,pitch,yaw, p,q,r]
        state = nodes.init_state(sensor_data)

        plot = RealtimePlotter(history_len=200, px_per_panel=480)
        nodes.set_wind(random.randrange(-10, 10), random.randrange(-10, 10), 0)

        run = True

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
            state = estimator.state_estimate(sensor_data, state, dt=dt*1e-9) #state: [x,y,z, vx,vy,vz, roll,pitch,yaw, p,q,r]

            #print(state, ', state esimate\n') #prints rpy estimate

            user_cmd = manual_control.keys_to_cmd(keys_pressed)
            #user_cmd[-1] = pose_subscriber.heading
            #user_cmd.append(8) #height_ref

            manual_control.x = 0
            manual_control.y = 0





            motor_cmd = control.logic(state, user_cmd, dt=dt*1e-9)
            #print('motor_cmd: ', motor_cmd)

            #print(f'\ndt: {dt}\n-----------------------')

            #run = control.run



            #plot.push(control.plot('x'))
            #plot.push(control.plot('y'))           
            plot.push(control.plot('thrust'))
            #plot.push(control.plot('p'))
            #plot.push(control.plot('q'))
            #plot.push(control.plot('r'))
            #screen.fill((12, 12, 14))
            plot.draw(screen)


            rotors_publisher.set_cmd(motor_cmd)


            if keys_pressed['r']:

                sensor_data = nodes.get_sensor_data(imu=imu_subscriber.data,
                                                    mag=mag_subscriber.data,
                                                    lidar=min(lidar_subscriber.lidar_ranges),
                                                    baro=baro_subscriber.data,
                                                    pos=pose_subscriber.data)
                dt = clock_subscriber.data.clock.nanosec - t
                state = estimator.state_estimate(sensor_data, state, dt=dt*1e-9)

                motor_cmd = [0, 0, 0, 0, 0]
                rotors_publisher.set_cmd(motor_cmd)
                nodes.reset()
                control.__init__()
                nodes.set_wind(random.randrange(-10, 10), random.randrange(-10, 10), 0)
                #manual_control = mc.ManualControl(height=5, x=0, y=0)
                time.sleep(1)


            t = clock_subscriber.data.clock.nanosec


            #rotors_publisher.set_cmd([480, 280, 280, 280, 280])


            #print(pose_subscriber.data)
            #print(state['pitch'], state['roll'],state['yaw'])
            #print('\n---------------------------------------\n\n')











            pygame.display.flip()
            clock.tick(20) 

    except KeyboardInterrupt:
        pass
    finally:
        pygame.quit()


    rclpy.shutdown()


if __name__ == '__main__':
    main()


"""
TO-DO:

    > code Visual Inertial Odometry (VIO) for pose of drone
    > ignore outer loop control (for now), control rpyt
"""