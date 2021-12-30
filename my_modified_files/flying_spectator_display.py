#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Show a window with the spectator's view

"""

TOWN_NAME = 'Town10HD_Opt'

import carla

from carlasyncmode import CarlaSyncMode
from spectatorcontroller import SpectatorController
from carlapygamehelper import CarlaPygameHelper

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
        sensors = []
        camera_rgb_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        sensors.append({'name':'camera_rgb','blueprint':camera_rgb_bp})

        camera_semantic_bp = world.get_blueprint_library().find('sensor.camera.semantic_segmentation')
        sensors.append({'name':'camera_semantic','blueprint':camera_semantic_bp})

        spec_ctrl = SpectatorController(world, sensors)
        actor_list.append(spec_ctrl.sensors['camera_rgb']) # so it will be destroyed at the end
        actor_list.append(spec_ctrl.sensors['camera_semantic']) # so it will be destroyed at the end

        spec_ctrl.spectator.set_transform(carla.Transform(carla.Location(z=50), carla.Rotation()))

        sensors_list = actor_list
        with CarlaSyncMode(world, sensors_list, fps=20) as sync_mode:
            while True:
                if pgh.should_quit():
                    return
                pgh.clock.tick()

                spec_ctrl.parse_keys(pgh.clock.get_time())

                # Advance the simulation and wait for the data.
                snapshot, image_rgb, image_semantic = sync_mode.tick(timeout=2.0)


                # image_semseg.convert(carla.ColorConverter.CityScapesPalette)
                fps = round(1.0 / snapshot.timestamp.delta_seconds)

                # Draw the display.
                pgh.draw_image(image_rgb)
                
                # Overlay the semantic segmentation
                image_semantic.convert(carla.ColorConverter.CityScapesPalette)
                pgh.draw_image(image_semantic, blend=True)

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
