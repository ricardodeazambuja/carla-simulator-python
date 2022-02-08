#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Show a window with the spectator's view

"""

TOWN_NAME = 'Town10HD_Opt'

import math

import numpy as np

import carla

from carlasyncmode import CarlaSyncMode
from spectatorcontroller import SpectatorController
from carlapygamehelper import CarlaPygameHelper

SIM_FPS = 40
LIDAR_RANGE = 100

def main():
    actor_list = []

    pgh = CarlaPygameHelper(height=600, width=800)

    # Connect to the CARLA server
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    if world.get_map().name.split('/')[-1] != 'Town10HD_Opt':
        world = client.load_world('Town10HD_Opt') #it takes a while to load, so the client timeout needs to afford that.
    
    try:
        # https://carla.readthedocs.io/en/latest/bp_library/#sensor

        sensors = []

        # https://carla.readthedocs.io/en/latest/ref_sensors/#rgb-camera
        camera_rgb_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        sensors.append({'name':'camera_rgb','blueprint':camera_rgb_bp})

        # https://carla.readthedocs.io/en/latest/ref_sensors/#lidar-sensor
        lidar_raycast_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
        lidar_raycast_bp.range = LIDAR_RANGE
        lidar_raycast_bp.upper_fov = 85
        lidar_raycast_bp.lower_fov = -85
        lidar_raycast_bp.horizontal_fov = 360
        lidar_raycast_bp.points_per_second = 10000
        lidar_raycast_bp.rotation_frequency = 10
        lidar_raycast_bp.channels = 32
        lidar_raycast_bp.dropoff_general_rate = 0.0
        lidar_raycast_bp.dropoff_intensity_limit = 0.0
        lidar_raycast_bp.dropoff_zero_intensity = 0.0
        lidar_raycast_bp.noise_stddev = 0.0
        sensors.append({'name':'lidar_raycast','blueprint':lidar_raycast_bp})

        obstacle_bp = world.get_blueprint_library().find('sensor.other.obstacle')
        obstacle_bp.distance = 5
        obstacle_bp.hit_radius = 0.5
        obstacle_bp.only_dynamics = False
        obstacle_bp.debug_linetrace = True
        obstacle_bp.sensor_tick = 0.0
        sensors.append({'name':'obstacle_detection','blueprint':obstacle_bp})

        # camera_semantic_bp = world.get_blueprint_library().find('sensor.camera.semantic_segmentation')
        # sensors.append({'name':'camera_semantic','blueprint':camera_semantic_bp})

        spec_ctrl = SpectatorController(world, sensors)
        # Don't forget: actors need to be destroyed at the end ;)
     
        # Set initial pose
        spec_ctrl.spectator.set_transform(carla.Transform(carla.Location(z=50), carla.Rotation()))

        with CarlaSyncMode(world, spec_ctrl.sensors.values(), fps=SIM_FPS) as sync_mode:
            while True:
                if pgh.should_quit():
                    return
                pgh.clock.tick(SIM_FPS)

                # move using key presses
                keys = spec_ctrl.pygame.key.get_pressed()
                key_press = any(keys)
                
                ts = pgh.clock.get_time()/1000 # in seconds
                if key_press:
                    # changes the position and angles by this delta
                    delta = 0.5*ts
                    if keys[spec_ctrl.K_SPACE]: # stops it from moving
                        spec_ctrl.transform['loc'] = [0,0,0]
                        spec_ctrl.transform['rot'] = [0,0,0]
                    elif keys[spec_ctrl.K_UP]:
                        spec_ctrl.transform['loc'][0] += delta
                    elif keys[spec_ctrl.K_DOWN]:
                        spec_ctrl.transform['loc'][0] -= delta
                    elif keys[spec_ctrl.K_w]:
                                spec_ctrl.transform['rot'][1] -= delta
                    elif keys[spec_ctrl.K_s]:
                                spec_ctrl.transform['rot'][1] += delta
                    elif keys[spec_ctrl.K_a] or keys[spec_ctrl.K_LEFT]:
                                spec_ctrl.transform['rot'][2] -= delta
                    elif keys[spec_ctrl.K_d] or keys[spec_ctrl.K_RIGHT]:
                                spec_ctrl.transform['rot'][2] += delta

                # # 2) move programmatically (replacing any values set above)
                # spec_ctrl.transform['loc'] = [x, y, z]
                # spec_ctrl.transform['rot'] = [roll, pitch, yaw]

                
                curr_loc = spec_ctrl.spectator.get_transform().location
                curr_rot = spec_ctrl.spectator.get_transform().rotation

                # If we want it to move continously, kind of FPV style, 
                # where x (spec_ctrl.transform['loc'][0]) moves forward or backward
                yaw_rad = math.radians(curr_rot.yaw) # math should be faster than numpy for only one value
                pitch_rad = math.radians(curr_rot.pitch)
                next_loc = curr_loc + carla.Location(x=spec_ctrl.transform['loc'][0]*math.cos(yaw_rad), 
                                                     y=spec_ctrl.transform['loc'][0]*math.sin(yaw_rad), 
                                                     z=spec_ctrl.transform['loc'][0]*math.sin(pitch_rad))

                next_rot = carla.Rotation(roll=spec_ctrl.transform['rot'][0]+curr_rot.roll,
                                          pitch=max(-89.9, min(89.9, spec_ctrl.transform['rot'][1]+curr_rot.pitch)),
                                          yaw=spec_ctrl.transform['rot'][2]+curr_rot.yaw)

                # # Otherwise, just set the global position and the attitude
                # next_loc = carla.Location(x=spec_ctrl.transform['loc'][0], 
                #                           y=spec_ctrl.transform['loc'][1], 
                #                           z=spec_ctrl.transform['loc'][2])
                # next_rot = carla.Rotation(roll=spec_ctrl.transform['rot'][0],
                #                           pitch=spec_ctrl.transform['rot'][1]),
                #                           yaw=spec_ctrl.transform['rot'][2])

                
                next_rot = carla.Rotation(roll=spec_ctrl.transform['rot'][0]+curr_rot.roll,
                                          pitch=max(-89.9, min(89.9, spec_ctrl.transform['rot'][1]+curr_rot.pitch)),
                                          yaw=spec_ctrl.transform['rot'][2]+curr_rot.yaw)

                spec_ctrl.spectator.set_transform(carla.Transform(next_loc, next_rot)) # it will continuously apply the transformation

                # Advance the simulation and wait for the data.
                # snapshot, image_rgb, image_semantic = sync_mode.tick(timeout=2.0)
                received_data = sync_mode.tick(timeout=1/SIM_FPS)
                snapshot = received_data[0]
                sim_data = {k:d for k,d in zip(spec_ctrl.sensors.keys(),received_data[1:])}

                if sim_data['obstacle_detection']:
                    print(f"Distance to obstable [{sim_data['obstacle_detection'].other_actor}] (raycast, ahead): {sim_data['obstacle_detection'].distance}")
                else:
                    print(f"Distance to obstable (raycast, ahead): out of reach")


                if sim_data['lidar_raycast']:
                    # https://github.com/carla-simulator/carla/blob/master/PythonAPI/examples/lidar_to_camera.py
                    # Get the lidar data and convert it to a numpy array.
                    p_cloud_size = len(sim_data['lidar_raycast'])
                    p_cloud = np.copy(np.frombuffer(sim_data['lidar_raycast'].raw_data, dtype=np.dtype('f4')))
                    p_cloud = np.reshape(p_cloud, (p_cloud_size, 4))

                    distance_from_lidar = np.sqrt(np.power(p_cloud[:, :3],2).sum(axis=1))
                    print(f"Distance to obstables (from Lidar {sim_data['lidar_raycast'].horizontal_angle}): {distance_from_lidar.min()}")

                    # Lidar intensity array of shape (p_cloud_size,) but, for now, let's
                    # focus on the 3D points.
                    intensity = np.array(p_cloud[:, 3])

                    # Point cloud in lidar sensor space array of shape (3, p_cloud_size).
                    local_lidar_points = np.array(p_cloud[:, :3]).T

                    # Add an extra 1.0 at the end of each 3d point so it becomes of
                    # shape (4, p_cloud_size) and it can be multiplied by a (4, 4) matrix.
                    local_lidar_points = np.r_[
                        local_lidar_points, [np.ones(local_lidar_points.shape[1])]]

                    # This (4, 4) matrix transforms the points from lidar space to world space.
                    lidar_2_world = spec_ctrl.sensors['lidar_raycast'].get_transform().get_matrix()

                    # Transform the points from lidar space to world space.
                    world_points = np.dot(lidar_2_world, local_lidar_points)
                else:
                    print(f"Distance to obstables (from Lidar): out of reach")

                # image_semseg.convert(carla.ColorConverter.CityScapesPalette)
                fps = round(1.0 / snapshot.timestamp.delta_seconds)

                # Draw the display.
                if sim_data['camera_rgb']:
                    pgh.draw_image(sim_data['camera_rgb'])
                
                # Overlay the semantic segmentation
                # image_semantic.convert(carla.ColorConverter.CityScapesPalette)
                # pgh.draw_image(image_semantic, blend=True)

                msg = 'UpKey:Forward, DownKey:Backward, LeftKey:+Yaw, RightKey:-Yaw, W:+Pitch, S:-Pitch, SPACE: Stop, ESC: Exit'
                pgh.blit(pgh.font.render(msg, True, (255, 255, 255)), (8, 10))
                pgh.blit(pgh.font.render(f'{pgh.clock.get_fps()} FPS (real)', True, (255, 255, 255)), (8, 30))
                pgh.blit(pgh.font.render(f'{fps} FPS (simulated)', True, (255, 255, 255)), (8, 50))
                pgh.blit(pgh.font.render(f'{spec_ctrl.spectator.get_transform().location}', True, (255, 255, 255)), (8, 70))
                pgh.blit(pgh.font.render(f'{spec_ctrl.spectator.get_transform().rotation}', True, (255, 255, 255)), (8, 90))
                pgh.flip()

    finally:

        print('destroying actors.')
        for actor in spec_ctrl.sensors.values():
            actor.destroy()

        pgh.quit()
        print('done.')


if __name__ == '__main__':

    try:
        main()

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
