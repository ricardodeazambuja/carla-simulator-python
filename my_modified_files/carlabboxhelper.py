# Based on previous work:
#  Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
#  Barcelona (UAB).
#
#  Copyright (c) 2019 Aptiv
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.


import numpy as np

import carla

class CarlaBBOXHelper:
    def __init__(self, view_width=800, view_height=600, view_fov=90) -> None:
        self.view_width = view_width
        self.view_height = view_height
        self.view_fov = view_fov


    def get_bounding_boxes(self, camera, objects_to_track, max_track_dist=100):
        """
        Creates 3D bounding boxes
        """

        filtered_objects = []
        for li, level_object in enumerate(objects_to_track):
            object_type = None
            if type(level_object) == carla.libcarla.Vehicle:
                object_type = "actor_vehicle"
                level_object_loc = level_object.get_transform().location
            elif type(level_object) == carla.libcarla.EnvironmentObject:
                object_type = "environment_obj"
                level_object_loc = level_object.bounding_box.location
            else:
                print(f"Weird object type: {type(level_object)}")

            camera_loc = camera.get_transform().location
            dist = camera_loc.distance(level_object_loc)
            if dist < max_track_dist:
                filtered_objects.append((dist, li, object_type))
        
        bounding_boxes = []
        if filtered_objects:
            # Order according to the distance (inverse), so the objects that are closer will come last
            filtered_objects = sorted(filtered_objects, key=lambda filtered_objects: filtered_objects[0], reverse=True)

            bounding_boxes = [bbox for bbox in [self.get_bounding_box(objects_to_track[li], object_type, camera) 
                                                                                        for _,li,object_type in filtered_objects]
                                                                                        if len(bbox)]
            # Filter out objects that have one or more corners of the 3D bbox behind the camera
            bounding_boxes = [bb for bb in bounding_boxes if all(bb[:, 2] >= 0)]

        return bounding_boxes


    def draw_bounding_boxes(self, bounding_boxes, display, pygame, BB_COLOR = (248, 64, 24)):
        """
        Draws bounding boxes on pygame display.
        """

        bb_surface = pygame.Surface((self.view_width, self.view_height))
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


    def get_bounding_box(self, obj, object_type, camera):
        """
        Returns 3D bounding box for a level_object based on camera view.
        """        
        bb_coords = self._create_bb_points(obj.bounding_box) # the bounding_box gives 3D center point and dimensions, without the points.
        coords_x_y_z = self._object_to_sensor(bb_coords, obj, object_type, camera)[:3, :]

        coords_y_minus_z_x = np.concatenate([coords_x_y_z[1, :], -coords_x_y_z[2, :], coords_x_y_z[0, :]])
        bbox = np.transpose(np.dot(camera.calibration, coords_y_minus_z_x))
        camera_bbox = np.concatenate([bbox[:, 0] / bbox[:, 2], bbox[:, 1] / bbox[:, 2], bbox[:, 2]], axis=1)

        if sum(camera_bbox[:,2]>0) >= 4:
            return camera_bbox
        else:
            return []


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


    def _object_to_sensor(self, coords, obj, obj_type, sensor):
        """
        Transforms coordinates of a level_object bounding box to sensor.
        """

        world_coords = self._object_to_world(coords, obj, obj_type)
        sensor_coords = self._world_to_sensor(world_coords, sensor)
        return sensor_coords


    def _object_to_world(self, coords, obj, obj_type):
        """
        Transforms coordinates of a vehicle bounding box to world.
        """

        bb_transform = carla.Transform(obj.bounding_box.location, obj.bounding_box.rotation)
        bb_init_matrix = self.get_matrix(bb_transform.location, bb_transform.rotation)

        if "actor" in obj_type:
            actor_transform = obj.get_transform()
            actor_world_matrix = self.get_matrix(actor_transform.location, actor_transform.rotation)
            bb_world_matrix = np.dot(actor_world_matrix, bb_init_matrix)
        elif "environment" in obj_type: 
            bb_world_matrix = bb_init_matrix

        world_coords = np.dot(bb_world_matrix, np.transpose(coords))
        return world_coords


    def _world_to_sensor(self, coords, sensor):
        """
        Transforms world coordinates to sensor.
        """
        
        transform = sensor.get_transform()
        sensor_world_matrix = self.get_matrix(transform.location, transform.rotation)
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
        matrix[0, 0] = c_p * c_y
        matrix[0, 1] = c_y * s_p * s_r - s_y * c_r
        matrix[0, 2] = -c_y * s_p * c_r - s_y * s_r
        matrix[1, 0] = s_y * c_p
        matrix[1, 1] = s_y * s_p * s_r + c_y * c_r
        matrix[1, 2] = -s_y * s_p * c_r + c_y * s_r
        matrix[2, 0] = s_p
        matrix[2, 1] = -c_p * s_r
        matrix[2, 2] = c_p * c_r
        # Last column
        matrix[0, 3] = location.x
        matrix[1, 3] = location.y
        matrix[2, 3] = location.z
        # matrix[3, 3] = 1 # already set (identity)
        return matrix

    @staticmethod
    def spawn_actors(world, actor_list, spawn_points=None, blueprints=None):
        available_spawn_points = list(world.get_map().get_spawn_points())
        available_blueprints = list(world.get_blueprint_library().filter('vehicle.*'))
        
        batch = []
        for i,(spawn_point,blueprint) in enumerate(zip(spawn_points,blueprints)):
            try:
                actor_list.append(world.spawn_actor(blueprint, spawn_point))
                # self.actor_list[-1].set_simulate_physics(False)
            except Exception as err:
                print(err)
                print(f"{i+1} - Error spawning {blueprint} {(spawn_point in available_spawn_points)} at {spawn_point} {(blueprint in available_blueprints)}!")
