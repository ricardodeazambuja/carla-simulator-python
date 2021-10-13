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

import carla

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


import skimage.measure as measure

# from https://arijitray1993.github.io/CARLA_tutorial/
def get_bbox_from_mask(mask):
    label_mask = measure.label(mask)
    props = measure.regionprops(label_mask)
    return [[(prop.bbox[3],prop.bbox[4]),(prop.bbox[0],prop.bbox[1])] for prop in props]



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
    def get_bounding_boxes(world, camera, actor_type=carla.CityObjectLabel.Vehicles, max_dist=50,
                           n_samples_per_axis=10, safety_margin=0.8, max_cast_dist=2.0):
        """
        Creates 3D bounding boxes based on carla actor_type and camera.
        """

        level_objects = world.get_environment_objects(object_type=actor_type)
        bounding_boxes = [bbox for bbox in [ClientSideBoundingBoxes.get_bounding_box(world, level_object, camera, actor_type, max_dist, 
                                                                                     n_samples_per_axis, safety_margin, max_cast_dist) 
                                                                                    for level_object in level_objects]
                                                                                    if len(bbox)]
        # filter out objects that are behind the camera
        bounding_boxes = [bb for bb in bounding_boxes if all(bb[:, 2] > 0)]

        return bounding_boxes

    @staticmethod
    def get_bounding_box(world, level_object, camera, actor_type, max_dist, 
                         n_samples_per_axis, safety_margin, max_cast_dist):
        """
        Returns 3D bounding box for a level_object based on camera view.
        """
        camera_loc = camera.get_transform().location
        level_object_loc = level_object.bounding_box.location
        level_object_loc_xyz = np.array([level_object_loc.x, level_object_loc.y, level_object_loc.z]).T
        level_object_ext = level_object.bounding_box.extent
        level_object_ext_xyz = np.array([level_object_ext.x, level_object_ext.y, level_object_ext.z]).T

        dist = camera_loc.distance(level_object_loc)
        if dist <= max_dist: # avoid processing everything in the level
            sampled_bbox_values = level_object_loc_xyz + (np.random.rand(n_samples_per_axis,3)*2-1)*level_object_ext_xyz*safety_margin
            sampled_locations = [carla.Location(*sample_i) for sample_i in sampled_bbox_values]
            
            ray_cast = []
            for location_i in sampled_locations:
                # for labelled_pnt in world.cast_ray(camera_loc, location_i):
                #     if labelled_pnt.label == actor_type:
                #         dist_cast = labelled_pnt.location.distance(location_i)
                #         if dist_cast < max_cast_dist:
                #             ray_cast.append(True)

                labelled_pnt = world.project_point(camera_loc, 
                                                   carla.Vector3D(location_i.x-camera_loc.x, 
                                                                  location_i.y-camera_loc.y, 
                                                                  location_i.z-camera_loc.z), 
                                                   dist)
                if labelled_pnt:
                    if labelled_pnt.label == actor_type:
                        dist_cast = labelled_pnt.location.distance(location_i)
                        if dist_cast < max_cast_dist:
                            ray_cast.append(True)

            if any(ray_cast):
                bb_cords = ClientSideBoundingBoxes._create_bb_points(level_object.bounding_box)
                cords_x_y_z = ClientSideBoundingBoxes._level_object_to_sensor(bb_cords, level_object, camera)[:3, :]
                cords_y_minus_z_x = np.concatenate([cords_x_y_z[1, :], -cords_x_y_z[2, :], cords_x_y_z[0, :]])
                bbox = np.transpose(np.dot(camera.calibration, cords_y_minus_z_x))
                camera_bbox = np.concatenate([bbox[:, 0] / bbox[:, 2], bbox[:, 1] / bbox[:, 2], bbox[:, 2]], axis=1)
                return camera_bbox
        
        return []

    @staticmethod
    def _create_bb_points(bounding_box):
        """
        Returns 3D bounding box for a level_object.
        """

        cords = np.zeros((8, 4))
        extent = bounding_box.extent
        cords[0, :] = np.array([extent.x, extent.y, -extent.z, 1])
        cords[1, :] = np.array([-extent.x, extent.y, -extent.z, 1])
        cords[2, :] = np.array([-extent.x, -extent.y, -extent.z, 1])
        cords[3, :] = np.array([extent.x, -extent.y, -extent.z, 1])
        cords[4, :] = np.array([extent.x, extent.y, extent.z, 1])
        cords[5, :] = np.array([-extent.x, extent.y, extent.z, 1])
        cords[6, :] = np.array([-extent.x, -extent.y, extent.z, 1])
        cords[7, :] = np.array([extent.x, -extent.y, extent.z, 1])
        return cords

    @staticmethod
    def _level_object_to_sensor(cords, level_object, sensor):
        """
        Transforms coordinates of a level_object bounding box to sensor.
        """

        world_cord = ClientSideBoundingBoxes._level_object_to_world(cords, level_object)
        sensor_cord = ClientSideBoundingBoxes._world_to_sensor(world_cord, sensor)
        return sensor_cord

    @staticmethod
    def _level_object_to_world(cords, level_object):
        """
        Transforms coordinates of a level_object bounding box to world.
        """

        bb_transform = carla.Transform(level_object.bounding_box.location, level_object.bounding_box.rotation)
        bb_world_matrix = ClientSideBoundingBoxes.get_matrix(bb_transform.location,bb_transform.rotation)
        world_cords = np.dot(bb_world_matrix, np.transpose(cords))
        return world_cords

    @staticmethod
    def _world_to_sensor(cords, sensor):
        """
        Transforms world coordinates to sensor.
        """
        
        sensor_world_matrix = ClientSideBoundingBoxes.get_matrix(sensor.get_transform().location,sensor.get_transform().rotation)
        world_sensor_matrix = np.linalg.inv(sensor_world_matrix)
        sensor_cords = np.dot(world_sensor_matrix, cords)
        return sensor_cords

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
        self.capture = True

    def camera_blueprint(self):
        """
        Returns camera blueprint.
        """

        camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(VIEW_WIDTH))
        camera_bp.set_attribute('image_size_y', str(VIEW_HEIGHT))
        camera_bp.set_attribute('fov', str(VIEW_FOV))
        return camera_bp

    def set_synchronous_mode(self, synchronous_mode):
        """
        Sets synchronous mode.
        """
        while not self.world:
            print("Where is the carla server?!?!?")
            sleep(.1) # wait for the simulator

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

        camera_transform = carla.Transform(carla.Location(x=0.5, y=0, z=3), carla.Rotation(yaw=0, pitch=-25))
        self.camera = self.world.spawn_actor(self.camera_blueprint(), camera_transform, attach_to=self.car)
        weak_self = weakref.ref(self)
        self.camera.listen(lambda image: weak_self().set_image(weak_self, image))

        calibration = np.identity(3)
        calibration[0, 2] = VIEW_WIDTH / 2.0
        calibration[1, 2] = VIEW_HEIGHT / 2.0
        calibration[0, 0] = calibration[1, 1] = VIEW_WIDTH / (2.0 * np.tan(VIEW_FOV * np.pi / 360.0))
        self.camera.calibration = calibration

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

    def render(self, display):
        """
        Transforms image from camera sensor and blits it to main pygame display.
        """

        if self.image is not None:
            array = np.frombuffer(self.image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (self.image.height, self.image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
            display.blit(surface, (0, 0))



    def game_loop(self):
        """
        Main program loop.
        """

        try:
            pygame.init()

            self.client = carla.Client('127.0.0.1', 2000)
            self.client.set_timeout(2.0)
            self.world = self.client.get_world()

            self.setup_car()
            self.setup_camera()

            self.display = pygame.display.set_mode((VIEW_WIDTH, VIEW_HEIGHT), pygame.HWSURFACE | pygame.DOUBLEBUF)
            pygame_clock = pygame.time.Clock()

            self.set_synchronous_mode(True)
            vehicles = self.world.get_actors().filter('vehicle.*')

            while True:
                self.world.tick()

                self.capture = True
                pygame_clock.tick_busy_loop(20)

                self.render(self.display)
                bounding_boxes = ClientSideBoundingBoxes.get_bounding_boxes(self.world, 
                                                                            self.camera,
                                                                            actor_type=carla.CityObjectLabel.Vehicles)
                
                print(f"Bounding boxes found: {len(bounding_boxes)}")

                ClientSideBoundingBoxes.draw_bounding_boxes(self.display, bounding_boxes, BB_COLOR=(0,255,0))

                bboxes2D = []
                for bbox in bounding_boxes:
                    bboxes2D.append([(bbox[:,0].max(),bbox[:,1].max()),
                                     (bbox[:,0].min(),bbox[:,1].min())])
                # The bboxes2D will also return values that are OUTSIDE the camera view (negative values).

                ClientSideBoundingBoxes.draw_bounding_boxes(self.display, bboxes2D) # standard color


                pygame.display.flip()

                pygame.event.pump()
                if self.control(self.car):
                    return

        finally:
            self.set_synchronous_mode(False)
            self.camera.destroy()
            self.car.destroy()
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
