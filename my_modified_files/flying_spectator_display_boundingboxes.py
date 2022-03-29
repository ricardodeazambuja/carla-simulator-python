#!/usr/bin/env python

# Based on previous work:
#  Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
#  Barcelona (UAB).
#
#  Copyright (c) 2019 Aptiv
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Show a window with the spectator's view and 2D, 3D BBoxes

"""

TOWN_NAME = 'Town10HD_Opt'
MASTER = False
SIM_FPS = 20

import random

import numpy as np

import pygame

import carla

from carlasyncmode import CarlaSyncMode
from spectatorcontroller import SpectatorController
from carlapygamehelper import CarlaPygameHelper
from carlabboxhelper import CarlaBBOXHelper



def main():

    cbbh = CarlaBBOXHelper()
    VIEW_HEIGHT = cbbh.view_height
    VIEW_WIDTH = cbbh.view_width
    VIEW_FOV = 90

    actor_list = []

    pgh = CarlaPygameHelper(height=cbbh.view_height, width=cbbh.view_width)

    # Connect to the CARLA server
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    if MASTER:
        if world.get_map().name.split('/')[-1] != TOWN_NAME:
            world = client.load_world(TOWN_NAME) #it takes a while to load, so the client timeout needs to afford that.
    
    try:
        camera_transform = carla.Transform(carla.Location(x=2, y=0, z=1.2), carla.Rotation(yaw=0, pitch=-15))
        
        calibration = np.identity(3)
        calibration[0, 2] = VIEW_WIDTH / 2.0
        calibration[1, 2] = VIEW_HEIGHT / 2.0
        calibration[0, 0] = calibration[1, 1] = VIEW_WIDTH / (2.0 * np.tan(VIEW_FOV * np.pi / 360.0))

        camera_rgb_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        camera_rgb_bp.set_attribute('image_size_x', str(VIEW_WIDTH))
        camera_rgb_bp.set_attribute('image_size_y', str(VIEW_HEIGHT))
        camera_rgb_bp.set_attribute('fov', str(VIEW_FOV))

        camera_semantic_bp = world.get_blueprint_library().find('sensor.camera.semantic_segmentation')
        camera_semantic_bp.set_attribute('image_size_x', str(VIEW_WIDTH))
        camera_semantic_bp.set_attribute('image_size_y', str(VIEW_HEIGHT))
        camera_semantic_bp.set_attribute('fov', str(VIEW_FOV))

        sensors = []
        sensors.append({'name':"camera_rgb", 'transform':camera_transform, 'blueprint':camera_rgb_bp})
        sensors.append({'name':"camera_semantic", 'transform':camera_transform, 'blueprint':camera_semantic_bp})

        spec_ctrl = SpectatorController(world, sensors)
        spec_ctrl.sensors['camera_rgb'].calibration = calibration
        actor_list.append(spec_ctrl.sensors['camera_rgb']) # so it will be destroyed at the end
        spec_ctrl.sensors['camera_semantic'].calibration = calibration
        actor_list.append(spec_ctrl.sensors['camera_semantic']) # so it will be destroyed at the end
        sensors_list = [act for act in actor_list]


        spec_ctrl.spectator.set_transform(carla.Transform(carla.Location(z=50), carla.Rotation()))

        # Spawn N vehicles, randomly
        N = 20
        all_spawn_points = list(world.get_map().get_spawn_points())
        random.shuffle(all_spawn_points)
        spawn_points = [i for i in all_spawn_points[:N]]
        all_blueprints = list(world.get_blueprint_library().filter('vehicle.*'))
        # blueprintsWalkers = world.get_blueprint_library().filter("walker.pedestrian.*")
        blueprints = [random.choice(all_blueprints) for i in range(N)]
        cbbh.spawn_actors(world, actor_list, spawn_points=spawn_points, blueprints=blueprints)
        vehicles = world.get_actors().filter('vehicle.*')
        level_objects = world.get_environment_objects(object_type=carla.CityObjectLabel.Vehicles)
        track_objects = list(vehicles) + list(level_objects)

        print("Starting...")
        with CarlaSyncMode(world, sensors_list, fps=SIM_FPS, master=MASTER) as sync_mode:
            print("Loop...")
            while True:
                if pgh.should_quit():
                    return
                pgh.clock.tick()

                spec_ctrl.parse_keys(pgh.clock.get_time())

                # Advance the simulation and wait for the data.
                # The order below follows the sensor_list!
                if sync_mode.master:
                    received_data = sync_mode.tick(timeout=1/SIM_FPS)
                else:
                    received_data = sync_mode.tick(timeout=1)

                snapshot = received_data[0]
                if snapshot == None:
                    print("No snapshot...")
                    continue

                sim_data = {k:d for k,d in zip(spec_ctrl.sensors.keys(),received_data[1:])}

                # image_semseg.convert(carla.ColorConverter.CityScapesPalette)
                fps = round(1.0 / snapshot.timestamp.delta_seconds)
                
                # Process semantic segmentation
                if sim_data['camera_semantic']:
                    array = np.frombuffer(sim_data['camera_semantic'].raw_data, dtype=np.dtype("uint8"))
                    array = np.reshape(array, (sim_data['camera_semantic'].height, sim_data['camera_semantic'].width, 4))
                    array = array[:, :, :3]
                    array = array[:, :, ::-1]
                    array = array.swapaxes(0, 1)
                    masked = np.zeros(array.shape[:2], dtype=np.uint8)
                    
                    # Vehicles (10)
                    mask = array[..., 0] == 10 # https://carla.readthedocs.io/en/0.9.12/ref_sensors/#semantic-segmentation-camera
                    masked[mask] = 1

                # Draw the display.
                if sim_data['camera_rgb']:
                    pgh.draw_image(sim_data['camera_rgb'])

                # Overlay the semantic segmentation
                if sim_data['camera_semantic']:
                    sim_data['camera_semantic'].convert(carla.ColorConverter.CityScapesPalette)
                    pgh.draw_image(sim_data['camera_semantic'], blend=True)

                    bounding_boxes = cbbh.get_bounding_boxes(spec_ctrl.sensors['camera_rgb'], track_objects, max_track_dist=100)

                    bboxes2D = []
                    for bbox in bounding_boxes:
                        bbox[bbox < 0] = 0
                        xmax = int(min(VIEW_WIDTH, bbox[:,0].max()))
                        ymax = int(min(VIEW_HEIGHT, bbox[:,1].max()))
                        xmin = int(bbox[:,0].min())
                        ymin = int(bbox[:,1].min())
                        bboxes2D.append([(xmax, ymax),
                                        (xmin, ymin)])

                    # Filter bboxes that match the mask                
                    MIN_PIXELS_MASK = 100
                    bboxes2D_filtered = []
                    bboxes_filtered = []
                    for bi, bbox in enumerate(bboxes2D):
                        if len(masked):
                            if masked[bbox[1][0]:bbox[0][0], bbox[1][1]:bbox[0][1]].sum() > MIN_PIXELS_MASK:
                                masked[bbox[1][0]:bbox[0][0], bbox[1][1]:bbox[0][1]] = bi+1 # mark where the bbox is
                                                                                            # The "+1" is because 0 is the default value
                                # The list was ordered to put the objects that are closer (therefore probably bigger on the camera plane), last.

                    # It needs to be 2 loops because it will initially fill the masked pixels with indices that intersect
                    for bi, bbox in enumerate(bboxes2D):
                        if len(masked):
                            if (masked[bbox[1][0]:bbox[0][0], bbox[1][1]:bbox[0][1]] == bi+1).sum() > MIN_PIXELS_MASK: # The "+1" is because 0 is the default value
                                bboxes2D_filtered.append(bbox)
                                bboxes_filtered.append(bounding_boxes[bi])

                    cbbh.draw_bounding_boxes(bboxes_filtered, pgh.display, pygame, BB_COLOR=(0,255,0))
                    cbbh.draw_bounding_boxes(bboxes2D_filtered, pgh.display, pygame) # standard color


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
