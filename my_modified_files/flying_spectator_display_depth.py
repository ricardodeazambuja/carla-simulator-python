#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Show a window with the spectator's view

"""

TOWN_NAME = 'Town07'

from cProfile import label
import math
import random

import numpy as np

import carla

import pygame

from carlasyncmode import CarlaSyncMode
from spectatorcontroller import SpectatorController
from carlapygamehelper import CarlaPygameHelper


from PIL import Image

SIM_FPS = 40
MAX_DEPTH_DIST = 1000

CAM_HEIGHT = 480
CAM_WIDTH = 640



def sample_height(world, curr_loc, dist=10000):
    labelled_pnts = (world.cast_ray(curr_loc, carla.Location(x=curr_loc.x,y=curr_loc.y,z=-dist)) or [None]) # it will return an ordered list

    if labelled_pnts:
        for l in labelled_pnts:
            if l:
                if str(l.label) != "NONE":
                    break
    if l:
        return l.label,l.location



def main():
    actor_list = []

    pgh = CarlaPygameHelper(height=CAM_HEIGHT, width=CAM_WIDTH)

    # Connect to the CARLA server
    client = carla.Client('carla-container.local', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    available_maps = client.get_available_maps()

    print(f"Available maps: \n{client.get_available_maps()}")
    
    # Useful for randomization as well
    map_layer_names = [
                        carla.MapLayer.NONE,
                        carla.MapLayer.Buildings,
                        carla.MapLayer.Decals,
                        carla.MapLayer.Foliage,
                        carla.MapLayer.Ground,
                        carla.MapLayer.ParkedVehicles,
                        carla.MapLayer.Particles,
                        carla.MapLayer.Props,
                        carla.MapLayer.StreetLights,
                        carla.MapLayer.Walls,
                        carla.MapLayer.All
                        ]
    # world.unload_map_layer(carla.MapLayer.Buildings)
    # world.load_map_layer(carla.MapLayer.Buildings)

    load_town = TOWN_NAME
    print(f"Current map: {world.get_map().name.split('/')[-1]}")
    print(f"Loading {load_town}")
    if world.get_map().name.split('/')[-1] != load_town:
        world = client.load_world(load_town) #it takes a while to load, so the client timeout needs to afford that.

    try:
        # https://carla.readthedocs.io/en/latest/bp_library/#sensor

        sensors = []

        # https://carla.readthedocs.io/en/latest/ref_sensors/#rgb-camera
        camera_depth_bp = world.get_blueprint_library().find('sensor.camera.depth')
        camera_depth_bp.set_attribute('image_size_x', str(CAM_WIDTH))
        camera_depth_bp.set_attribute('image_size_y', str(CAM_HEIGHT))
        sensors.append({'name':'camera_depth','blueprint':camera_depth_bp})


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

                spec_ctrl.spectator.set_transform(carla.Transform(next_loc, next_rot)) # it will continuously apply the transformation

                # Advance the simulation and wait for the data.
                # snapshot, image_rgb, image_semantic = sync_mode.tick(timeout=2.0)
                received_data = sync_mode.tick(timeout=1/SIM_FPS)
                snapshot = received_data[0]
                if snapshot == None:
                    print("No snapshot...")
                    continue
                
                sim_data = {k:d for k,d in zip(spec_ctrl.sensors.keys(),received_data[1:])}

                fps = round(1.0 / snapshot.timestamp.delta_seconds)

                # Draw the display.
                if sim_data['camera_depth']:
                    
                    carla_image = sim_data['camera_depth']
                    #https://github.com/ricardodeazambuja/carla-ros/blob/b0b9a5ec1bb4e98ad3dfbbcd05c180d370af6348/carla_ros_bridge/src/carla_ros_bridge/camera.py#L299
                    bgra_image = np.ndarray(
                        shape=(carla_image.height, carla_image.width, 4),
                        dtype=np.uint8, buffer=carla_image.raw_data)
                    scales = np.array([65536.0, 256.0, 1.0, 0]) / (256**3 - 1) * 1000
                    depth_image = np.dot(bgra_image, scales).astype(np.float32) # max value 1000m

                    depth_image[depth_image>MAX_DEPTH_DIST] = MAX_DEPTH_DIST
                    depth_image_255 = (255*depth_image/MAX_DEPTH_DIST).astype('uint8')

                    #Image.fromarray(depth_image_255).save("test.png")

                    array = depth_image_255[..., np.newaxis].swapaxes(0, 1)
                    array = np.repeat(array[:, :, :], 3, axis=2)
                    image_surface = spec_ctrl.pygame.surfarray.make_surface(array)
                    pgh.display.blit(image_surface, (0, 0))
                

                    msg = 'Up/Down Keys:Forward/Backward, Left/Right Keys:+/-Yaw, W/S:+/-Pitch, SPACE:Stop, ESC:Exit'
                    pgh.blit(pgh.font.render(msg, True, (255, 0, 0)), (8, 10))
                    pgh.blit(pgh.font.render(f'{pgh.clock.get_fps()} FPS (real)', True, (255, 0, 0)), (8, 30))
                    pgh.blit(pgh.font.render(f'{fps} FPS (simulated)', True, (255, 0, 0)), (8, 50))
                    pgh.blit(pgh.font.render(f'{spec_ctrl.spectator.get_transform().location}', True, (255, 0, 0)), (8, 70))
                    pgh.blit(pgh.font.render(f'{spec_ctrl.spectator.get_transform().rotation}', True, (255, 0, 0)), (8, 90))
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
