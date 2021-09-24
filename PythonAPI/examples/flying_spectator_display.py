#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Show a window with the spectator's view

"""

TOWN_NAME = 'Town10HD_Opt'

import glob
import os
import sys

import random
import math
import queue

import numpy as np

import pygame
from pygame.locals import K_SPACE
from pygame.locals import K_DOWN
from pygame.locals import K_LEFT
from pygame.locals import K_RIGHT
from pygame.locals import K_UP
from pygame.locals import K_q
from pygame.locals import K_e
from pygame.locals import K_w
from pygame.locals import K_s
from pygame.locals import K_a
from pygame.locals import K_d


import carla


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
        self._settings = self.world.get_settings() # save the current world settings
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
        self.world.apply_settings(self._settings) # restore the previous world settings

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

class SpectatorController():
    def __init__(self, world):
        self.spectator = world.get_spectator()
        self.camera_rgb = world.spawn_actor(
            world.get_blueprint_library().find('sensor.camera.rgb'),
            carla.Transform(carla.Location(), carla.Rotation()),
            attach_to=self.spectator)

        self.transform = {'loc':[0,0,0],'rot':[0,0,0]}

    def parse_keys(self, milliseconds):
        """Parses keyboard input when keys are pressed"""
        keys = pygame.key.get_pressed()
        delta = 5e-4 * milliseconds
        if keys[K_SPACE]:
            self.transform['loc'] = [0,0,0]
            self.transform['rot'] = [0,0,0]
        elif keys[K_UP]:
            self.transform['loc'][0] += delta
        elif keys[K_DOWN]:
            self.transform['loc'][0] -= delta
        elif keys[K_w]:
                    self.transform['rot'][1] -= delta
        elif keys[K_s]:
                    self.transform['rot'][1] += delta
        elif keys[K_a] or keys[K_LEFT]:
                    self.transform['rot'][2] -= delta
        elif keys[K_d] or keys[K_RIGHT]:
                    self.transform['rot'][2] += delta

        curr_loc = self.spectator.get_transform().location
        curr_rot = self.spectator.get_transform().rotation

        yaw_rad = math.radians(curr_rot.yaw)
        pitch_rad = math.radians(curr_rot.pitch)

        next_loc = curr_loc + carla.Location(x=self.transform['loc'][0]*math.cos(yaw_rad), 
                                             y=self.transform['loc'][0]*math.sin(yaw_rad), 
                                             z=self.transform['loc'][0]*math.sin(pitch_rad))
        
        next_rot = carla.Rotation(roll=self.transform['rot'][0]+curr_rot.roll,
                                  pitch=max(-89.9, min(89.9, self.transform['rot'][1]+curr_rot.pitch)),
                                  yaw=self.transform['rot'][2]+curr_rot.yaw)
        
        self.spectator.set_transform(carla.Transform(next_loc, next_rot)) # it will continuously apply the transformation

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
    world = client.get_world()

    if world.get_map().name.split('/')[-1] != 'Town10HD_Opt':
        world = client.load_world('Town10HD_Opt') #it takes a while to load, so the client timeout needs to afford that.
    
    try:
        spec_ctrl = SpectatorController(world)
        actor_list.append(spec_ctrl.camera_rgb) # so it will be destroyed at the end

        spec_ctrl.spectator.set_transform(carla.Transform(carla.Location(z=50), carla.Rotation()))

        with CarlaSyncMode(world, spec_ctrl.camera_rgb, fps=20) as sync_mode:
            while True:
                if should_quit():
                    return
                clock.tick()

                spec_ctrl.parse_keys(clock.get_time())

                # Advance the simulation and wait for the data.
                snapshot, image_rgb = sync_mode.tick(timeout=2.0)


                # image_semseg.convert(carla.ColorConverter.CityScapesPalette)
                fps = round(1.0 / snapshot.timestamp.delta_seconds)

                # Draw the display.
                draw_image(display, image_rgb)
                display.blit(
                    font.render('UpKey:Forward, DownKey:Backward, LeftKey:+Yaw, RightKey:-Yaw, W:+Pitch, S:-Pitch, SPACE: Stop, ESC: Exit', True, (255, 255, 255)),
                    (8, 10))
                display.blit(
                    font.render(f'{clock.get_fps()} FPS (real)', True, (255, 255, 255)),
                    (8, 30))
                display.blit(
                    font.render(f'{fps} FPS (simulated)', True, (255, 255, 255)),
                    (8, 50))
                display.blit(
                    font.render(f'{spec_ctrl.spectator.get_transform().location}', True, (255, 255, 255)),
                    (8, 70))
                display.blit(
                    font.render(f'{spec_ctrl.spectator.get_transform().rotation}', True, (255, 255, 255)),
                    (8, 90))
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
