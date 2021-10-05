#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import random

try:
    import pygame
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')

try:
    import queue
except ImportError:
    import Queue as queue


class CarlaSyncMode(object):
    """
    Context manager to synchronize output from different sensors. Synchronous
    mode is enabled as long as we are inside this context

        with CarlaSyncMode(world, sensors) as sync_mode:
            while True:
                data = sync_mode.tick(timeout=1.0)

    """

    def __init__(self, world, *sensors, **kwargs):
        self.world = world
        self.sensors = sensors
        self.frame = None
        self.delta_seconds = 1.0 / kwargs.get('fps', 20)
        self._queues = []
        self._settings = None

    def __enter__(self):
        self._settings = self.world.get_settings()
        self.frame = self.world.apply_settings(carla.WorldSettings(
            no_rendering_mode=False,
            synchronous_mode=True,
            fixed_delta_seconds=self.delta_seconds))

        def make_queue(register_event):
            q = queue.Queue()
            register_event(q.put)
            self._queues.append(q)

        make_queue(self.world.on_tick)
        for sensor in self.sensors:
            make_queue(sensor.listen)
        return self

    def tick(self, timeout):
        self.frame = self.world.tick()
        data = [self._retrieve_data(q, timeout) for q in self._queues]
        assert all(x.frame == self.frame for x in data)
        return data

    def __exit__(self, *args, **kwargs):
        self.world.apply_settings(self._settings)

    def _retrieve_data(self, sensor_queue, timeout):
        while True:
            data = sensor_queue.get(timeout=timeout)
            if data.frame == self.frame:
                return data


def draw_image(surface, image, blend=False):
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    image_surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    if blend:
        image_surface.set_alpha(100)
    surface.blit(image_surface, (0, 0))


def get_font():
    fonts = [x for x in pygame.font.get_fonts()]
    default_font = 'ubuntumono'
    font = default_font if default_font in fonts else fonts[0]
    font = pygame.font.match_font(font)
    return pygame.font.Font(font, 14)


def should_quit():
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            return True
        elif event.type == pygame.KEYUP:
            if event.key == pygame.K_ESCAPE:
                return True
    return False


def main():
    actor_list = []
    pygame.init()

    display = pygame.display.set_mode(
        (800, 600),
        pygame.HWSURFACE | pygame.DOUBLEBUF)
    font = get_font()
    clock = pygame.time.Clock()

    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    print("Available maps:")
    print('\n'.join(client.get_available_maps()))

    world = client.load_world('Town10HD_Opt') #it takes a while to load, so the client timeout needs to afford that.
    # world = client.get_world()

    env_objs = world.get_environment_objects(carla.CityObjectLabel.Any)
    for o in env_objs:
        print(o.id, o.name, o.type)

    # # Toggle buildings off
    # world.enable_environment_objects(objects_to_toggle, False) # objects_to_toggle => a set of object ids
    # # Toggle buildings on
    # world.enable_environment_objects(objects_to_toggle, True) # objects_to_toggle => a set of object ids

    # map = world.get_map()
    # spawn_points = map.get_spawn_points()

    try:
        m = world.get_map()
        # blueprints = [bp for bp in world.get_blueprint_library().filter('*')]
        # for blueprint in blueprints:
        #     print(blueprint.id)
        #     for attr in blueprint:
        #         print('  - {}'.format(attr))
        # exit(0)

        start_pose = random.choice(m.get_spawn_points()) # selects a random place to start, considering the possible spawn points.
        waypoint = m.get_waypoint(start_pose.location)

        blueprint_library = world.get_blueprint_library()
        vehicle = world.spawn_actor(
            random.choice(blueprint_library.filter('vehicle.*')),
            start_pose)
        color = random.choice(vehicle.get_attribute('color').recommended_values)
        vehicle.set_attribute('color', color)
        actor_list.append(vehicle)

        # Usually, physics would be off (False) allowing the vehicles to go through
        # wall and other vehicles...
        vehicle.set_simulate_physics(False)
        # https://carla.readthedocs.io/en/0.9.12/tuto_G_control_vehicle_physics/
        # physics_control = vehicle.get_physics_control()
        # physics_control.mass = 100000
        # # Apply Vehicle Physics Control for the vehicle
        # vehicle.apply_physics_control(physics_control)
        # print(physics_control)


        # camera_rgb = world.spawn_actor(
        #     blueprint_library.find('sensor.camera.rgb'),
        #     carla.Transform(carla.Location(x=0, y=0, z=300), carla.Rotation(pitch=-90)))
        # actor_list.append(camera_rgb)

        camera_rgb = world.spawn_actor(
            blueprint_library.find('sensor.camera.rgb'),
            carla.Transform(carla.Location(x=-5.5, z=10), carla.Rotation(pitch=-15)),
            attach_to=vehicle)
        actor_list.append(camera_rgb)

        # camera_semseg = world.spawn_actor(
        #     blueprint_library.find('sensor.camera.semantic_segmentation'),
        #     carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15)),
        #     attach_to=vehicle)
        # actor_list.append(camera_semseg)

        # Create a synchronous mode context.
        # with CarlaSyncMode(world, camera_rgb, camera_semseg, fps=30) as sync_mode:
        xpos = 0
        waypoint_counter = 0
        waypoint_counter_max = 50
        with CarlaSyncMode(world, camera_rgb, fps=30) as sync_mode:
            while True:
                if should_quit():
                    return
                clock.tick()

                # Advance the simulation and wait for the data.
                # snapshot, image_rgb, image_semseg = sync_mode.tick(timeout=2.0)
                snapshot, image_rgb = sync_mode.tick(timeout=2.0)

                # Choose the next waypoint and update the car location.
                waypoint = random.choice(waypoint.next(1.5))
                # waypoint_counter += 1
                # if waypoint_counter <= waypoint_counter_max:
                #     waypoint = waypoint.next(1.5)[0]
                # elif waypoint_counter > waypoint_counter_max and waypoint_counter <= waypoint_counter_max*2:
                #     waypoint = waypoint.previous(1.5)[0] # randomly select from a list of waypoints in the opposit direction of the lane
                # else:
                #     waypoint_counter = 0
                vehicle.set_transform(waypoint.transform)
                # camera_rgb.set_transform(carla.Transform(carla.Location(x=xpos, y=10, z=10), carla.Rotation(pitch=-85)))
                # vehicle.set_transform(carla.Transform(carla.Location(x=xpos)))
                # xpos += 0.1

                # image_semseg.convert(carla.ColorConverter.CityScapesPalette)
                fps = round(1.0 / snapshot.timestamp.delta_seconds)

                # Draw the display.
                draw_image(display, image_rgb)
                # draw_image(display, image_semseg, blend=True)
                display.blit(
                    font.render('% 5d FPS (real)' % clock.get_fps(), True, (255, 255, 255)),
                    (8, 10))
                display.blit(
                    font.render('% 5d FPS (simulated)' % fps, True, (255, 255, 255)),
                    (8, 28))
                pygame.display.flip()

    finally:

        print('destroying actors.')
        for actor in actor_list:
            actor.destroy()

        pygame.quit()
        print('done.')


if __name__ == '__main__':

    try:

        main()

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
