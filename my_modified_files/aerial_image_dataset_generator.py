#!/usr/bin/env python

# Based on original work from...
# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.


from time import sleep
import hashlib
import numpy as np
from PIL import Image
from PIL.PngImagePlugin import PngInfo

import carla

from carlasyncmode import CarlaSyncMode
from spectatorcontroller import SpectatorController
from carlapygamehelper import CarlaPygameHelper
 
with open(__file__,"rb") as f:
    bytes = f.read()
    script_hash = hashlib.sha256(bytes).hexdigest()

# Stuff you should edit before running the script...
DATA_DIR = 'samples'
TOWN_NAME = 'Town01'
TOTAL_SAMPLES = 100 # -1 for manual sampling
YAW_LIMITS = (-90, 90)
Z_LIMITS = (50, 100)
SEED = 42


SIM_FPS = 10
MAX_DEPTH_DIST = 1000

CAM_HEIGHT = 480
CAM_WIDTH = 640
FOV = 69

# https://carla.readthedocs.io/en/latest/core_map/#maps-and-navigation
#carla.MapLayer.
# 'All',
#  'Buildings',
#  'Decals',
#  'Foliage',
#  'Ground',
#  'ParkedVehicles',
#  'Particles',
#  'Props',
#  'StreetLights',
#  'Walls'
# Calling a map ending with _Opt allows to enable/disable map layers

# Towns with suggested limits (for z<=100)
MAPS = {
        'Town01':   {'x':(10,380),    'y':(10,430)}, 
        'Town02':   {'x':(20,180),   'y':(100,250)}, 
        'Town03':   {'x':(-120,250), 'y':(-210,190)}, 
        'Town04':   {'x':(50,380),   'y':(-370,170)}, 
        'Town05':   {'x':(-290,490), 'y':(-220,280)}, 
        'Town06':   {'x':(-340,690), 'y':(-170,380)}, 
        'Town07':   {'x':(-190,90), 'y':(-260,110)}, 
        'Town10HD': {'x':(-100,120), 'y':(-60,110)}, 
}


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

    pgh = CarlaPygameHelper(height=CAM_HEIGHT, width=CAM_WIDTH)

    # Connect to the CARLA server
    client = carla.Client('carla-container.local', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    # available_maps = client.get_available_maps()
    # print(f"Available maps: \n{client.get_available_maps()}")
    

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
        camera_rgb_bp.set_attribute('fov', str(FOV)) # OAK-D Lite HFOV
        camera_rgb_bp.set_attribute('fstop', '2.2') # OAK-D Lite FSTOP
        camera_rgb_bp.set_attribute('image_size_x', str(CAM_WIDTH))
        camera_rgb_bp.set_attribute('image_size_y', str(CAM_HEIGHT))
        sensors.append({'name':'camera_rgb','blueprint':camera_rgb_bp,
                        'transform': carla.Transform(carla.Location(), carla.Rotation(pitch=-90.0))})

        # https://carla.readthedocs.io/en/latest/ref_sensors/#sensor.camera.instance_segmentation
        camera_instance_segmentation_bp = world.get_blueprint_library().find('sensor.camera.instance_segmentation')
        camera_instance_segmentation_bp.set_attribute('fov', str(FOV)) # OAK-D Lite HFOV
        camera_instance_segmentation_bp.set_attribute('image_size_x', str(CAM_WIDTH))
        camera_instance_segmentation_bp.set_attribute('image_size_y', str(CAM_HEIGHT))
        sensors.append({'name':'camera_instance_segmentation','blueprint':camera_instance_segmentation_bp,
                        'transform': carla.Transform(carla.Location(), carla.Rotation(pitch=-90.0))})

        # https://carla.readthedocs.io/en/latest/ref_sensors/#depth-camera
        camera_depth_bp = world.get_blueprint_library().find('sensor.camera.depth')
        camera_depth_bp.set_attribute('fov', str(FOV)) # FOV matches the stereo pair
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
                    delta = 10.0
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
                    elif keys[spec_ctrl.K_z]:
                        next_rot.yaw += delta
                    elif keys[spec_ctrl.K_x]:
                        next_rot.yaw -= delta
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

                if TOTAL_SAMPLES>sample_counter:
                    capture_data = True
                    rs.shuffle(weather_presets)
                    weather = getattr(carla.WeatherParameters, weather_presets[0])
                    weather.sun_azimuth_angle = rs.randint(0,360)
                    weather.sun_altitude_angle = rs.randint(1,50)
                    world.set_weather(weather)
                    next_rot.yaw = rs.randint(YAW_LIMITS[0],YAW_LIMITS[1])
                    next_loc.x = rs.randint(MAPS[TOWN_NAME]['x'][0],MAPS[TOWN_NAME]['x'][1])
                    next_loc.y = rs.randint(MAPS[TOWN_NAME]['y'][0],MAPS[TOWN_NAME]['y'][1])
                    next_loc.z = rs.randint(Z_LIMITS[0],Z_LIMITS[1])
                    spec_ctrl.spectator.set_transform(carla.Transform(next_loc, next_rot)) # it will continuously apply the transformation
                    for i in range(10): # Weather seems to take a while to settle...
                        _ = sync_mode.tick(timeout=1/SIM_FPS)
                        sleep(1/SIM_FPS)
                else:
                    spec_ctrl.spectator.set_transform(carla.Transform(next_loc, next_rot)) # it will continuously apply the transformation

                # Advance the simulation and wait for the data.
                _ = sync_mode.tick(timeout=1/SIM_FPS) # Make sure the previous transform was executed
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

                # if sim_data['camera_stereo_right'] and capture_data:
                #     carla_image = sim_data['camera_stereo_right']
                #     bgra_image = np.ndarray(
                #         shape=(carla_image.height, carla_image.width, 4),
                #         dtype=np.uint8, buffer=carla_image.raw_data)[..., :3]
                #     camera_stereo_right = bgra_image[:, :, ::-1]
                #     # Image.fromarray(camera_stereo_right).save("camera_stereo_right.png")

                # if sim_data['camera_stereo_left'] and capture_data:
                #     carla_image = sim_data['camera_stereo_left']
                #     bgra_image = np.ndarray(
                #         shape=(carla_image.height, carla_image.width, 4),
                #         dtype=np.uint8, buffer=carla_image.raw_data)[..., :3]
                #     camera_stereo_left = bgra_image[:, :, ::-1]
                #     # Image.fromarray(camera_stereo_left).save("camera_stereo_left.png")

                # Draw the display.
                if sim_data['camera_rgb']:
                    pgh.draw_image(sim_data['camera_rgb'])
                       
                    # Overlay the semantic segmentation
                    # image_semantic.convert(carla.ColorConverter.CityScapesPalette)
                    # pgh.draw_image(image_semantic, blend=True)
                    save_data = False
                    if capture_data:
                        save_data = any(v != None for v in sim_data.values())

                    msg = 'Up/Down:+/-X, Left/Right:+/-Y, W/S:+/-Z, A/D:Weather, Z/X:+/-YAW, SPACE:Save, ESC:Exit'
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
                            filename = DATA_DIR+"/"+base_name+".png"
                            print(filename)
                            # It will save details to PNG metadata
                            metadata = PngInfo()
                            metadata.add_text("script_hash", script_hash)
                            metadata.add_text("TOTAL_SAMPLES", str(TOTAL_SAMPLES))
                            metadata.add_text("YAW_LIMITS", str(YAW_LIMITS))
                            metadata.add_text("Z_LIMITS", str(Z_LIMITS))
                            metadata.add_text("SEED", str(SEED))
                            metadata.add_text("FOV", str(FOV))
                            metadata.add_text("XYZYAW", f'({x}, {y}, {z}, {yaw})')
                            metadata.add_text("WEATHER_PRESET", weather_presets[0])
                            metadata.add_text("sun_azimuth_angle", f'{weather.sun_azimuth_angle}')
                            metadata.add_text("sun_altitude_angle", f'{weather.sun_altitude_angle}')
                            Image.fromarray(vars()[sname]).save(filename, pnginfo=metadata)

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
