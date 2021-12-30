import math

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


class SpectatorController():
    def __init__(self, world, sensors = [{'name':      None, 
                                          'transform': None, 
                                          'blueprint': None}]):

        self.spectator = world.get_spectator()
        self.sensors = {}

        for sensor in sensors:
            name = sensor['name']
            
            if 'transform' in sensor:
                transform = sensor['transform']
            else:
                transform = None
                
            blueprint = sensor['blueprint']

            if transform:
                self.sensors[name] = world.spawn_actor(
                    blueprint,
                    transform,
                    attach_to=self.spectator)
            else:
                self.sensors[name] = world.spawn_actor(
                    blueprint,
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

        yaw_rad = math.radians(curr_rot.yaw) # math should be faster than numpy for only one value
        pitch_rad = math.radians(curr_rot.pitch)

        next_loc = curr_loc + carla.Location(x=self.transform['loc'][0]*math.cos(yaw_rad), 
                                             y=self.transform['loc'][0]*math.sin(yaw_rad), 
                                             z=self.transform['loc'][0]*math.sin(pitch_rad))
        
        next_rot = carla.Rotation(roll=self.transform['rot'][0]+curr_rot.roll,
                                  pitch=max(-89.9, min(89.9, self.transform['rot'][1]+curr_rot.pitch)),
                                  yaw=self.transform['rot'][2]+curr_rot.yaw)
        
        self.spectator.set_transform(carla.Transform(next_loc, next_rot)) # it will continuously apply the transformation
