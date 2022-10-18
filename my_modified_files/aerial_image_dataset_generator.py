#!/usr/bin/env python

# Based on original work from...
# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.




from cProfile import label
from email.mime import base
import math
import random

import numpy as np

import carla

import pygame

from carlasyncmode import CarlaSyncMode
from spectatorcontroller import SpectatorController
from carlapygamehelper import CarlaPygameHelper


from PIL import Image


DATA_DIR = 'samples'
TOWN_NAME = 'Town01'

SEED = 42

SIM_FPS = 10
MAX_DEPTH_DIST = 1000

CAM_HEIGHT = 480
CAM_WIDTH = 640


# https://carla.readthedocs.io/en/latest/python_api/#carla.WeatherParameters
weather_presets = ['Default',
                   'ClearNoon',
                   'CloudyNoon',
                   'WetNoon',
                   'WetCloudyNoon',
                   'MidRainyNoon', 
                   'HardRainNoon', 
                   'SoftRainNoon',
                   'ClearSunset',
                   'CloudySunset',
                   'WetSunset',
                   'WetCloudySunset',
                   'MidRainSunset',
                   'HardRainSunset',
                   'SoftRainSunset']

rs = np.random.RandomState(SEED)
def main():
    actor_list = []

    pgh = CarlaPygameHelper(height=CAM_HEIGHT, width=CAM_WIDTH)

    # Connect to the CARLA server
    client = carla.Client('carla-container.local', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    available_maps = client.get_available_maps()

    print(f"Available maps: \n{client.get_available_maps()}")
    

    load_town = TOWN_NAME
    print(f"Current map: {world.get_map().name.split('/')[-1]}")
    print(f"Loading {load_town}")
    if world.get_map().name.split('/')[-1] != load_town:
        world = client.load_world(load_town) #it takes a while to load, so the client timeout needs to afford that.

    try:
        # https://carla.readthedocs.io/en/latest/bp_library/#sensor
        sensors = []

        # https://carla.readthedocs.io/en/latest/ref_sensors/#rgb-camera
        camera_rgb_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        camera_rgb_bp.set_attribute('fov', '69') # OAK-D Lite HFOV
        camera_rgb_bp.set_attribute('fstop', '2.2') # OAK-D Lite FSTOP
        camera_rgb_bp.set_attribute('image_size_x', str(CAM_WIDTH))
        camera_rgb_bp.set_attribute('image_size_y', str(CAM_HEIGHT))
        sensors.append({'name':'camera_rgb','blueprint':camera_rgb_bp,
                        'transform': carla.Transform(carla.Location(), carla.Rotation(pitch=-90.0))})

        # https://carla.readthedocs.io/en/latest/ref_sensors/#sensor.camera.instance_segmentation
        camera_instance_segmentation_bp = world.get_blueprint_library().find('sensor.camera.instance_segmentation')
        camera_instance_segmentation_bp.set_attribute('fov', '69') # OAK-D Lite HFOV
        camera_instance_segmentation_bp.set_attribute('image_size_x', str(CAM_WIDTH))
        camera_instance_segmentation_bp.set_attribute('image_size_y', str(CAM_HEIGHT))
        sensors.append({'name':'camera_instance_segmentation','blueprint':camera_instance_segmentation_bp,
                        'transform': carla.Transform(carla.Location(), carla.Rotation(pitch=-90.0))})

        # https://carla.readthedocs.io/en/latest/ref_sensors/#rgb-camera
        camera_stereo_left_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        camera_stereo_left_bp.set_attribute('fov', '73') # OAK-D Lite HFOV
        camera_stereo_left_bp.set_attribute('fstop', '2.2') # OAK-D Lite FSTOP
        camera_stereo_left_bp.set_attribute('image_size_x', str(CAM_WIDTH))
        camera_stereo_left_bp.set_attribute('image_size_y', str(CAM_HEIGHT))
        sensors.append({'name':'camera_stereo_left',
                        'blueprint':camera_stereo_left_bp,
                        'transform': carla.Transform(carla.Location(y=-(75/1000)/2),
                                     carla.Rotation(pitch=-90.0))}) # OAK-D Lite Baseline (75mm))

        # https://carla.readthedocs.io/en/latest/ref_sensors/#rgb-camera
        camera_stereo_right_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        camera_stereo_right_bp.set_attribute('fov', '73') # OAK-D Lite HFOV
        camera_stereo_right_bp.set_attribute('fstop', '2.2') # OAK-D Lite FSTOP
        camera_stereo_right_bp.set_attribute('image_size_x', str(CAM_WIDTH))
        camera_stereo_right_bp.set_attribute('image_size_y', str(CAM_HEIGHT))
        sensors.append({'name':'camera_stereo_right',
                        'blueprint':camera_stereo_right_bp,
                        'transform': carla.Transform(carla.Location(y=+(75/1000)/2),
                                     carla.Rotation(pitch=-90.0))}) # OAK-D Lite Baseline (75mm))

        # https://carla.readthedocs.io/en/latest/ref_sensors/#depth-camera
        camera_depth_bp = world.get_blueprint_library().find('sensor.camera.depth')
        camera_depth_bp.set_attribute('fov', '73') # FOV matches the stereo pair
        camera_depth_bp.set_attribute('image_size_x', str(CAM_WIDTH))
        camera_depth_bp.set_attribute('image_size_y', str(CAM_HEIGHT))
        sensors.append({'name':'camera_depth','blueprint':camera_depth_bp,
                        'transform': carla.Transform(carla.Location(), carla.Rotation(pitch=-90.0))})


        spec_ctrl = SpectatorController(world, sensors)
        # Don't forget: actors need to be destroyed at the end ;)
     
        # Set initial pose
        spec_ctrl.spectator.set_transform(carla.Transform(carla.Location(z=100), carla.Rotation()))

        sample_counter = 0
        weather = world.get_weather()
        with CarlaSyncMode(world, spec_ctrl.sensors.values(), fps=SIM_FPS) as sync_mode:
            while True:
                if pgh.should_quit():
                    return
                pgh.clock.tick(SIM_FPS)

                # move using key presses
                keys = spec_ctrl.pygame.key.get_pressed()
                key_press = any(keys)
                
                next_loc = spec_ctrl.spectator.get_transform().location
                next_rot = spec_ctrl.spectator.get_transform().rotation

                capture_data = False
                if key_press:
                    # changes the position by this delta
                    delta = 1.0
                    if keys[spec_ctrl.K_SPACE]:
                        capture_data = True
                    elif keys[spec_ctrl.K_UP]:
                        next_loc.x += delta
                    elif keys[spec_ctrl.K_DOWN]:
                        next_loc.x -= delta
                    elif keys[spec_ctrl.K_LEFT]:
                        next_loc.y -= delta
                    elif keys[spec_ctrl.K_RIGHT]:
                        next_loc.y += delta
                    elif keys[spec_ctrl.K_w]:
                        next_loc.z += delta
                    elif keys[spec_ctrl.K_s]:
                        next_loc.z -= delta
                    elif keys[spec_ctrl.K_a]:
                        weather_presets.append(weather_presets.pop(0))
                        weather = getattr(carla.WeatherParameters, weather_presets[0])
                        weather.sun_azimuth_angle = rs.randint(0,360)
                        weather.sun_altitude_angle = rs.randint(1,50)
                        world.set_weather(weather)
                    elif keys[spec_ctrl.K_d]:
                        weather_presets.insert(0, weather_presets.pop(-1))
                        weather = getattr(carla.WeatherParameters, weather_presets[0])
                        weather.sun_azimuth_angle = rs.randint(0,360)
                        weather.sun_altitude_angle = rs.randint(1,50)
                        world.set_weather(weather)

                spec_ctrl.spectator.set_transform(carla.Transform(next_loc, next_rot)) # it will continuously apply the transformation

                # Advance the simulation and wait for the data.
                # snapshot, image_rgb, image_semantic = sync_mode.tick(timeout=2.0)
                received_data = sync_mode.tick(timeout=1/SIM_FPS)
                snapshot = received_data[0]
                if snapshot == None:
                    print("No snapshot???")
                    continue
                
                sim_data = {k:d for k,d in zip(spec_ctrl.sensors.keys(),received_data[1:])}

                fps = round(1.0 / snapshot.timestamp.delta_seconds)

                if sim_data['camera_depth'] and capture_data:
                    carla_image = sim_data['camera_depth']
                    camera_depth = np.ndarray(
                        shape=(carla_image.height, carla_image.width, 4),
                        dtype=np.uint8, buffer=carla_image.raw_data)[..., :3]
                    # Image.fromarray(camera_depth).save("camera_depth.png")
                    
                    # #https://github.com/ricardodeazambuja/carla-ros/blob/b0b9a5ec1bb4e98ad3dfbbcd05c180d370af6348/carla_ros_bridge/src/carla_ros_bridge/camera.py#L299
                    # bgra_image = np.ndarray(
                    #     shape=(carla_image.height, carla_image.width, 4),
                    #     dtype=np.uint8, buffer=carla_image.raw_data)
                    # scales = np.array([65536.0, 256.0, 1.0, 0]) / (256**3 - 1) * 1000
                    # depth_image = np.dot(bgra_image, scales) # max value 1000m
                    # depth_image[depth_image>MAX_DEPTH_DIST] = MAX_DEPTH_DIST
                    # depth_image_255 = (255*depth_image/MAX_DEPTH_DIST).astype('uint8')
                    # Image.fromarray(depth_image_255).save("depth_image_255.png")

                if sim_data['camera_instance_segmentation'] and capture_data:
                    carla_image = sim_data['camera_instance_segmentation']
                    camera_instance_segmentation = np.ndarray(
                        shape=(carla_image.height, carla_image.width, 4),
                        dtype=np.uint8, buffer=carla_image.raw_data)[..., :3]
                    # Image.fromarray(camera_instance_segmentation).save("camera_instance_segmentation.png")

                if sim_data['camera_rgb'] and capture_data:
                    carla_image = sim_data['camera_rgb']
                    bgra_image = np.ndarray(
                        shape=(carla_image.height, carla_image.width, 4),
                        dtype=np.uint8, buffer=carla_image.raw_data)[..., :3]
                    camera_rgb = bgra_image[:, :, ::-1]
                    # Image.fromarray(camera_rgb).save("camera_rgb.png")

                if sim_data['camera_stereo_right'] and capture_data:
                    carla_image = sim_data['camera_stereo_right']
                    bgra_image = np.ndarray(
                        shape=(carla_image.height, carla_image.width, 4),
                        dtype=np.uint8, buffer=carla_image.raw_data)[..., :3]
                    camera_stereo_right = bgra_image[:, :, ::-1]
                    # Image.fromarray(camera_stereo_right).save("camera_stereo_right.png")

                if sim_data['camera_stereo_left'] and capture_data:
                    carla_image = sim_data['camera_stereo_left']
                    bgra_image = np.ndarray(
                        shape=(carla_image.height, carla_image.width, 4),
                        dtype=np.uint8, buffer=carla_image.raw_data)[..., :3]
                    camera_stereo_left = bgra_image[:, :, ::-1]
                    # Image.fromarray(camera_stereo_left).save("camera_stereo_left.png")

                # Draw the display.
                if sim_data['camera_rgb']:
                    pgh.draw_image(sim_data['camera_rgb'])
                       
                    # Overlay the semantic segmentation
                    # image_semantic.convert(carla.ColorConverter.CityScapesPalette)
                    # pgh.draw_image(image_semantic, blend=True)
                    save_data = False
                    if capture_data:
                        save_data = any(v != None for v in sim_data.values())

                    msg = 'Up/Down Keys:Forward/Backward, Left/Right Keys:+/-Yaw, W/S:+/-Pitch, SPACE:Stop, ESC:Exit'
                    pgh.blit(pgh.font.render(msg, True, (255, 255, 255)), (8, 10))
                    pgh.blit(pgh.font.render(f'{pgh.clock.get_fps()} FPS (real)', True, (255, 255, 255)), (8, 30))
                    pgh.blit(pgh.font.render(f'{fps} FPS (simulated)', True, (255, 255, 255)), (8, 50))
                    pgh.blit(pgh.font.render(f'{spec_ctrl.spectator.get_transform().location}', True, (255, 255, 255)), (8, 70))
                    pgh.blit(pgh.font.render(f'{spec_ctrl.spectator.get_transform().rotation}', True, (255, 255, 255)), (8, 90))
                    pgh.blit(pgh.font.render(f'Current sample: {sample_counter:04d}', True, (255, 255, 255)), (8, 110))
                    if save_data:
                        pgh.blit(pgh.font.render(f'***** Saving sample: {sample_counter+1:04d} *****', True, (255, 0, 0)), (8, 130))
                    pgh.flip()

                    if save_data:
                        sample_counter += 1
                        print(f'Savind sample {sample_counter:04d}')
                        x = int(spec_ctrl.spectator.get_transform().location.x)
                        y = int(spec_ctrl.spectator.get_transform().location.y)
                        z = int(spec_ctrl.spectator.get_transform().location.z)
                        yaw = int(spec_ctrl.spectator.get_transform().rotation.yaw)
                        for s in sensors:
                            sname = s['name']
                            base_name = f"{sample_counter:04d}_{TOWN_NAME}_{sname}"
                            sample_transform = f"_{x:03d}_{y:03d}_{z:03d}_{yaw:03d}"
                            weather_detail = f"_{weather_presets[0]}_{int(weather.sun_azimuth_angle):03d}_{int(weather.sun_altitude_angle):03d}"
                            filename = DATA_DIR+"/"+base_name+weather_detail+sample_transform+".png"
                            print(filename)
                            Image.fromarray(vars()[sname]).save(filename)

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
