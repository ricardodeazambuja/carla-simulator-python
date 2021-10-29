#!/usr/bin/env python

# Copyright (c) 2019 Aptiv
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.


#
# Modified for 2D, testing if the object is not "badly" occluded
# get_bounding_boxes now has:
# - max_dist (50) it will consider objects
# - n_samples_per_axis (10) it will random sample for ray casting
# - safety_margin (0.7) used for ray casting to shrink the sampled 3D space
# - max_cast_dist (2.0) because the surfaces are external and the points are anywhere


"""
An example of client-side bounding boxes with basic car controls.

Controls:

    W            : throttle
    S            : brake
    AD           : steer
    Space        : hand-brake

    ESC          : quit
"""

# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys
import math
from time import sleep

import numpy as np

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================

import weakref
import random

try:
    import pygame
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_SPACE
    from pygame.locals import K_a
    from pygame.locals import K_d
    from pygame.locals import K_s
    from pygame.locals import K_w
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')


TOWN_NAME = 'Town10HD_Opt'

import carla

from carlasyncmode import CarlaSyncMode
from spectatorcontroller import SpectatorController
from carlapygamehelper import CarlaPygameHelper


VIEW_WIDTH = 1920//2
VIEW_HEIGHT = 1080//2
VIEW_FOV = 90

# ==============================================================================
# -- ClientSideBoundingBoxes ---------------------------------------------------
# ==============================================================================

class ClientSideBoundingBoxes(object):
    """
    This is a module responsible for creating 3D bounding boxes and drawing them
    client-side on pygame surface.
    """
    @staticmethod
    def draw_bounding_boxes(display, bounding_boxes, BB_COLOR = (248, 64, 24)):
        """
        Draws bounding boxes on pygame display.
        """

        bb_surface = pygame.Surface((VIEW_WIDTH, VIEW_HEIGHT))
        bb_surface.set_colorkey((0, 0, 0))
        for bbox in bounding_boxes:
            if len(bbox) < 8: # 2D bbox
                pygame.draw.rect(bb_surface, BB_COLOR, pygame.Rect(bbox[1][0], 
                                                                   bbox[1][1], 
                                                                   bbox[0][0]-bbox[1][0], 
                                                                   bbox[0][1]-bbox[1][1]),1)
            else: # 3D bbox
                points = [(int(bbox[i, 0]), int(bbox[i, 1])) for i in range(8)]
                # draw lines
                # base
                pygame.draw.line(bb_surface, BB_COLOR, points[0], points[1])
                pygame.draw.line(bb_surface, BB_COLOR, points[1], points[2])
                pygame.draw.line(bb_surface, BB_COLOR, points[2], points[3])
                pygame.draw.line(bb_surface, BB_COLOR, points[3], points[0])
                # top
                pygame.draw.line(bb_surface, BB_COLOR, points[4], points[5])
                pygame.draw.line(bb_surface, BB_COLOR, points[5], points[6])
                pygame.draw.line(bb_surface, BB_COLOR, points[6], points[7])
                pygame.draw.line(bb_surface, BB_COLOR, points[7], points[4])
                # base-top
                pygame.draw.line(bb_surface, BB_COLOR, points[0], points[4])
                pygame.draw.line(bb_surface, BB_COLOR, points[1], points[5])
                pygame.draw.line(bb_surface, BB_COLOR, points[2], points[6])
                pygame.draw.line(bb_surface, BB_COLOR, points[3], points[7])

        display.blit(bb_surface, (0, 0))


    @staticmethod
    def get_bounding_boxes(world, camera, tracking_objects, actor_type=carla.CityObjectLabel.Vehicles, max_dist=100):
        """
        Creates 3D bounding boxes based on carla actor_type and camera.
        """

        filtered_objects = []
        for li, level_object in enumerate(tracking_objects):
            object_type = None
            if type(level_object) == carla.libcarla.Vehicle:
                object_type = "actor"
                level_object_loc = level_object.get_transform().location
            elif type(level_object) == carla.libcarla.EnvironmentObject:
                object_type = "environment"
                level_object_loc = level_object.bounding_box.location
            else:
                print(f"Weird object type: {type(level_object)}")

            camera_loc = camera.get_transform().location
            dist = camera_loc.distance(level_object_loc)
            if dist < max_dist:
                filtered_objects.append((dist, li, object_type))
        
        bounding_boxes = []
        if filtered_objects:
            # Ordered according to the distance (inverse), so the objects that are closer will come last
            filtered_objects = sorted(filtered_objects, key=lambda filtered_objects: filtered_objects[0], reverse=True)

            bounding_boxes = [bbox for bbox in [ClientSideBoundingBoxes.get_bounding_box(tracking_objects[li], object_type, camera) 
                                                                                        for _,li,object_type in filtered_objects]
                                                                                        if len(bbox)]
            # filter out objects that are behind the camera
            bounding_boxes = [bb for bb in bounding_boxes if all(bb[:, 2] > 0)]

        return bounding_boxes


    @staticmethod
    def get_bounding_box(level_object, object_type, camera):
        """
        Returns 3D bounding box for a level_object based on camera view.
        """        
        bb_coords = ClientSideBoundingBoxes._create_bb_points(level_object.bounding_box)
        if object_type == "actor":
            coords_x_y_z = ClientSideBoundingBoxes._vehicle_to_sensor(bb_coords, level_object, camera)[:3, :]
        elif object_type == "environment":
            coords_x_y_z = ClientSideBoundingBoxes._level_object_to_sensor(bb_coords, level_object, camera)[:3, :]
        else:
            print(f"Weird object type: {type(level_object)}")

        coords_y_minus_z_x = np.concatenate([coords_x_y_z[1, :], -coords_x_y_z[2, :], coords_x_y_z[0, :]])
        bbox = np.transpose(np.dot(camera.calibration, coords_y_minus_z_x))
        camera_bbox = np.concatenate([bbox[:, 0] / bbox[:, 2], bbox[:, 1] / bbox[:, 2], bbox[:, 2]], axis=1)
        return camera_bbox

    @staticmethod
    def _create_bb_points(bounding_box):
        """
        Returns 3D bounding box for a level_object.
        """

        coords = np.zeros((8, 4))
        extent = bounding_box.extent
        coords[0, :] = np.array([extent.x, extent.y, -extent.z, 1])
        coords[1, :] = np.array([-extent.x, extent.y, -extent.z, 1])
        coords[2, :] = np.array([-extent.x, -extent.y, -extent.z, 1])
        coords[3, :] = np.array([extent.x, -extent.y, -extent.z, 1])
        coords[4, :] = np.array([extent.x, extent.y, extent.z, 1])
        coords[5, :] = np.array([-extent.x, extent.y, extent.z, 1])
        coords[6, :] = np.array([-extent.x, -extent.y, extent.z, 1])
        coords[7, :] = np.array([extent.x, -extent.y, extent.z, 1])
        return coords

    @staticmethod
    def _level_object_to_sensor(coords, level_object, sensor):
        """
        Transforms coordinates of a level_object bounding box to sensor.
        """

        world_coords = ClientSideBoundingBoxes._level_object_to_world(coords, level_object)
        sensor_coords = ClientSideBoundingBoxes._world_to_sensor(world_coords, sensor)
        return sensor_coords

    @staticmethod
    def _vehicle_to_sensor(cords, vehicle, sensor):
        """
        Transforms coordinates of a vehicle bounding box to sensor.
        """

        world_cord = ClientSideBoundingBoxes._vehicle_to_world(cords, vehicle)
        sensor_cord = ClientSideBoundingBoxes._world_to_sensor(world_cord, sensor)
        return sensor_cord

    @staticmethod
    def _vehicle_to_world(cords, vehicle):
        """
        Transforms coordinates of a vehicle bounding box to world.
        """

        bb_transform = carla.Transform(vehicle.bounding_box.location, vehicle.bounding_box.rotation)
        bb_vehicle_matrix = ClientSideBoundingBoxes.get_matrix(bb_transform.location, bb_transform.rotation)
        vehicle_transform = vehicle.get_transform()
        vehicle_world_matrix = ClientSideBoundingBoxes.get_matrix(vehicle_transform.location, vehicle_transform.rotation)
        bb_world_matrix = np.dot(vehicle_world_matrix, bb_vehicle_matrix)
        world_cords = np.dot(bb_world_matrix, np.transpose(cords))
        return world_cords

    @staticmethod
    def _level_object_to_world(coords, level_object):
        """
        Transforms coordinates of a level_object bounding box to world.
        """

        bb_transform = carla.Transform(level_object.bounding_box.location, level_object.bounding_box.rotation)
        bb_world_matrix = ClientSideBoundingBoxes.get_matrix(bb_transform.location,bb_transform.rotation)
        world_coords = np.dot(bb_world_matrix, np.transpose(coords))
        return world_coords

    @staticmethod
    def _world_to_sensor(coords, sensor):
        """
        Transforms world coordinates to sensor.
        """
        
        transform = sensor.get_transform()
        sensor_world_matrix = ClientSideBoundingBoxes.get_matrix(transform.location, transform.rotation)
        world_sensor_matrix = np.linalg.inv(sensor_world_matrix)
        sensor_coords = np.dot(world_sensor_matrix, coords)
        return sensor_coords

    @staticmethod
    def get_matrix(location, rotation):
        """
        Creates matrix from carla transform.
        """

        # rotation = transform.rotation
        # location = transform.location
        c_y = np.cos(np.radians(rotation.yaw))
        s_y = np.sin(np.radians(rotation.yaw))
        c_r = np.cos(np.radians(rotation.roll))
        s_r = np.sin(np.radians(rotation.roll))
        c_p = np.cos(np.radians(rotation.pitch))
        s_p = np.sin(np.radians(rotation.pitch))
        matrix = np.matrix(np.identity(4))
        matrix[0, 3] = location.x
        matrix[1, 3] = location.y
        matrix[2, 3] = location.z
        matrix[0, 0] = c_p * c_y
        matrix[0, 1] = c_y * s_p * s_r - s_y * c_r
        matrix[0, 2] = -c_y * s_p * c_r - s_y * s_r
        matrix[1, 0] = s_y * c_p
        matrix[1, 1] = s_y * s_p * s_r + c_y * c_r
        matrix[1, 2] = -s_y * s_p * c_r + c_y * s_r
        matrix[2, 0] = s_p
        matrix[2, 1] = -c_p * s_r
        matrix[2, 2] = c_p * c_r
        return matrix


# ==============================================================================
# -- BasicSynchronousClient ----------------------------------------------------
# ==============================================================================


class BasicSynchronousClient(object):
    """
    Basic implementation of a synchronous client.
    """

    def __init__(self):
        self.client = None
        self.world = None
        self.camera = None
        self.car = None

        self.display = None
        self.image = None
        self.image_semantic = None
        self.capture = True
        self.capture_semantic = True
        self.masked = []

        self.actor_list = []

    def camera_blueprint(self, sensor_type="rgb"):
        """
        Returns camera blueprint.
        """

        camera_bp = self.world.get_blueprint_library().find('sensor.camera.'+sensor_type)
        camera_bp.set_attribute('image_size_x', str(VIEW_WIDTH))
        camera_bp.set_attribute('image_size_y', str(VIEW_HEIGHT))
        camera_bp.set_attribute('fov', str(VIEW_FOV))
        return camera_bp

    def set_synchronous_mode(self, synchronous_mode):
        """
        Sets synchronous mode.
        """

        settings = self.world.get_settings()
        settings.synchronous_mode = synchronous_mode
        self.world.apply_settings(settings)

    def setup_car(self):
        """
        Spawns actor-vehicle to be controled.
        """

        car_bp = self.world.get_blueprint_library().filter('vehicle.*')[0]
        location = random.choice(self.world.get_map().get_spawn_points())
        self.car = self.world.spawn_actor(car_bp, location)

    def setup_camera(self):
        """
        Spawns actor-camera to be used to render view.
        Sets calibration for client-side boxes rendering.
        """

        weak_self = weakref.ref(self) # a weak reference will allow the object (instance) to be garbage collected if needed...
        camera_transform = carla.Transform(carla.Location(x=2, y=0, z=1.2), carla.Rotation(yaw=0, pitch=-15))
        calibration = np.identity(3)
        calibration[0, 2] = VIEW_WIDTH / 2.0
        calibration[1, 2] = VIEW_HEIGHT / 2.0
        calibration[0, 0] = calibration[1, 1] = VIEW_WIDTH / (2.0 * np.tan(VIEW_FOV * np.pi / 360.0))

        self.camera = self.world.spawn_actor(self.camera_blueprint(sensor_type='rgb'), camera_transform, attach_to=self.car)
        self.camera.listen(lambda image: weak_self().set_image(weak_self, image))
        self.camera.calibration = calibration

        self.camera_semantic = self.world.spawn_actor(self.camera_blueprint(sensor_type='semantic_segmentation'), camera_transform, attach_to=self.car)
        self.camera_semantic.listen(lambda image: weak_self().set_image_semantic(weak_self, image))
        self.camera_semantic.calibration = calibration
        

    def control(self, car):
        """
        Applies control to main car based on pygame pressed keys.
        Will return True If ESCAPE is hit, otherwise False to end main loop.
        """

        keys = pygame.key.get_pressed()
        if keys[K_ESCAPE]:
            return True

        control = car.get_control()
        control.throttle = 0
        if keys[K_w]:
            control.throttle = 1
            control.reverse = False
        elif keys[K_s]:
            control.throttle = 1
            control.reverse = True
        if keys[K_a]:
            control.steer = max(-1., min(control.steer - 0.05, 0))
        elif keys[K_d]:
            control.steer = min(1., max(control.steer + 0.05, 0))
        else:
            control.steer = 0
        control.hand_brake = keys[K_SPACE]

        car.apply_control(control)
        return False


    @staticmethod
    def set_image(weak_self, img):
        """
        Sets image coming from camera sensor.
        The self.capture flag is a mean of synchronization - once the flag is
        set, next coming image will be stored.
        """

        self = weak_self()
        if self.capture:
            self.image = img
            self.capture = False

    @staticmethod
    def set_image_semantic(weak_self, img):
        """
        Sets image coming from camera sensor.
        The self.capture flag is a mean of synchronization - once the flag is
        set, next coming image will be stored.
        """

        self = weak_self()
        if self.capture_semantic:
            array = np.frombuffer(img.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (img.height, img.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            array = array.swapaxes(0, 1)
            self.masked = np.zeros(array.shape[:2], dtype=np.uint8)
            mask = array[..., 0] == 10 # https://carla.readthedocs.io/en/0.9.12/ref_sensors/#semantic-segmentation-camera
            self.masked[mask] = 1
            self.capture_semantic = False



    def render(self, display):
        """
        Transforms image from camera sensor and blits it to main pygame display.
        """

        if self.image:
            array = np.frombuffer(self.image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (self.image.height, self.image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            array = array.swapaxes(0, 1)
            surface = pygame.surfarray.make_surface(array)
            display.blit(surface, (0, 0))


    def spawn_actors(self, spawn_points=None, blueprints=None):
        available_spawn_points = list(self.world.get_map().get_spawn_points())
        available_blueprints = list(self.world.get_blueprint_library().filter('vehicle.*'))
        
        batch = []
        for i,(spawn_point,blueprint) in enumerate(zip(spawn_points,blueprints)):
            try:
                self.actor_list.append(self.world.spawn_actor(blueprint, spawn_point))
                # self.actor_list[-1].set_simulate_physics(False)
            except Exception as err:
                print(err)
                print(f"{i+1} - Error spawning {blueprint} {(spawn_point in available_spawn_points)} at {spawn_point} {(blueprint in available_blueprints)}!")

    def game_loop(self):
        """
        Main program loop.
        """

        try:
            pygame.init()

            # Connect to the local CARLA server at IP='127.0.0.1' and port=2000
            self.client = carla.Client('127.0.0.1', 2000)
            self.client.set_timeout(2.0)
            self.world = self.client.get_world()
            self.set_synchronous_mode(False) # It will be set to True after the initial setup.

            print(f"CARLA Client Version: {self.client.get_client_version()}")
            print(f"CARLA Server Version: {self.client.get_server_version()}")

            self.setup_car()
            self.setup_camera()
            # The update should happen together with the capture of the camera frame. I notice some bboxes sometimes lag behind.
            # Use the semantic camera to check for things in the view, all in the camera 2D coordinates because it generates bbox quite fast already.


            self.display = pygame.display.set_mode((VIEW_WIDTH, VIEW_HEIGHT), pygame.HWSURFACE | pygame.DOUBLEBUF)
            pygame_clock = pygame.time.Clock()
            
            # Spawn N vehicles, randomly
            N = 20
            all_spawn_points = list(self.world.get_map().get_spawn_points())
            random.shuffle(all_spawn_points)
            spawn_points = [i for i in all_spawn_points[:N]]
            all_blueprints = list(self.world.get_blueprint_library().filter('vehicle.*'))
            blueprints = [random.choice(all_blueprints) for i in range(N)]
            self.spawn_actors(spawn_points=spawn_points, blueprints=blueprints)

            vehicles = self.world.get_actors().filter('vehicle.*')

            level_objects = self.world.get_environment_objects(object_type=carla.CityObjectLabel.Vehicles)

            track_objects = list(vehicles) + list(level_objects)

            self.set_synchronous_mode(True)
            
            while True:
                self.world.tick()

                self.capture = True
                self.capture_semantic = True
                pygame_clock.tick_busy_loop(20)

                self.render(self.display)
                bounding_boxes = ClientSideBoundingBoxes.get_bounding_boxes(self.world, 
                                                                            self.camera,
                                                                            track_objects,
                                                                            max_dist=100)

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
                    if len(self.masked) and (bbox[1][0] != bbox[0][0]) and (bbox[1][1] != bbox[0][1]):
                        if self.masked[bbox[1][0]:bbox[0][0], bbox[1][1]:bbox[0][1]].sum() > MIN_PIXELS_MASK:
                            self.masked[bbox[1][0]:bbox[0][0], bbox[1][1]:bbox[0][1]] = bi+1 # mark where the bbox is
                                                                                             # The "+1" is because 0 is the default value
                            # The list was ordered to put the objects that are closer, last.

                for bi, bbox in enumerate(bboxes2D):
                    if len(self.masked) and (bbox[1][0] != bbox[0][0]) and (bbox[1][1] != bbox[0][1]):
                        if (self.masked[bbox[1][0]:bbox[0][0], bbox[1][1]:bbox[0][1]] == bi+1).sum() > MIN_PIXELS_MASK: # The "+1" is because 0 is the default value
                            bboxes2D_filtered.append(bbox)
                            bboxes_filtered.append(bounding_boxes[bi])

                ClientSideBoundingBoxes.draw_bounding_boxes(self.display, bboxes_filtered, BB_COLOR=(0,255,0))
                ClientSideBoundingBoxes.draw_bounding_boxes(self.display, bboxes2D_filtered) # standard color


                pygame.display.flip()

                pygame.event.pump()
                if self.control(self.car):
                    return

        finally:
            for actor in self.actor_list:
                actor.destroy()

            self.camera.destroy()
            self.car.destroy()

            self.set_synchronous_mode(False)
            pygame.quit()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    """
    Initializes the client-side bounding box demo.
    """

    try:
        client = BasicSynchronousClient()
        client.game_loop()
    finally:
        print('EXIT')


if __name__ == '__main__':
    main()
