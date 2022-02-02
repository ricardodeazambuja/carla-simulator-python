#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Show a window with the spectator's view

"""

TOWN_NAME = 'Town10HD_Opt'

import numpy as np

import carla

from carlasyncmode import CarlaSyncMode
from spectatorcontroller import SpectatorController
from carlapygamehelper import CarlaPygameHelper

SIM_FPS = 40

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
        lidar_raycast_bp.range = 200
        lidar_raycast_bp.points_per_second = 10000
        lidar_raycast_bp.rotation_frequency = 10
        lidar_raycast_bp.channels = 2
        sensors.append({'name':'lidar_raycast','blueprint':lidar_raycast_bp})

        # camera_semantic_bp = world.get_blueprint_library().find('sensor.camera.semantic_segmentation')
        # sensors.append({'name':'camera_semantic','blueprint':camera_semantic_bp})

        spec_ctrl = SpectatorController(world, sensors)
        actor_list.append(spec_ctrl.sensors['camera_rgb']) # so it will be destroyed at the end
        actor_list.append(spec_ctrl.sensors['lidar_raycast']) # so it will be destroyed at the end
        # actor_list.append(spec_ctrl.sensors['camera_semantic']) # so it will be destroyed at the end

        spec_ctrl.spectator.set_transform(carla.Transform(carla.Location(z=50), carla.Rotation()))

        sensors_list = actor_list
        with CarlaSyncMode(world, sensors_list, fps=SIM_FPS) as sync_mode:
            while True:
                if pgh.should_quit():
                    return
                pgh.clock.tick(SIM_FPS)

                spec_ctrl.parse_keys(pgh.clock.get_time())

                # Advance the simulation and wait for the data.
                # snapshot, image_rgb, image_semantic = sync_mode.tick(timeout=2.0)
                snapshot, image_rgb, lidar_data = sync_mode.tick(timeout=2.0)


                # https://github.com/carla-simulator/carla/blob/master/PythonAPI/examples/lidar_to_camera.py
                # Get the lidar data and convert it to a numpy array.
                p_cloud_size = len(lidar_data)
                p_cloud = np.copy(np.frombuffer(lidar_data.raw_data, dtype=np.dtype('f4')))
                p_cloud = np.reshape(p_cloud, (p_cloud_size, 4))

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

                # image_semseg.convert(carla.ColorConverter.CityScapesPalette)
                fps = round(1.0 / snapshot.timestamp.delta_seconds)

                # Draw the display.
                pgh.draw_image(image_rgb)
                
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
        for actor in actor_list:
            actor.destroy()

        pgh.quit()
        print('done.')


if __name__ == '__main__':

    try:
        main()

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
