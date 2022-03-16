import traceback
import sys
from time import sleep

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D # https://stackoverflow.com/a/56222305/7658422
from matplotlib.widgets import Button


import numpy as np

import carla


NUMBER_OF_SAMPLES = 5000
MINIMUM_Z = -5

SHOW_PLOT = True


LABEL_COLORS = {
    "NONE": [0, 0, 0],
    "BUILDINGS": [70, 70, 70], 
    "FENCES": [100, 40, 40],
    "OTHER": [55, 90, 80], 
    "PEDESTRIANS": [220, 20, 60],
    "POLES": [153, 153, 153],
    "ROADLINES": [157, 234, 50],
    "ROADS": [128, 64, 128],
    "SIDEWALKS": [244, 35, 232],
    "VEGETATION": [107, 142, 35],
    "VEHICLES": [0, 0, 142],
    "WALLS": [102, 102, 156],
    "TRAFFICSIGNS": [220, 220, 0],
    "SKY": [70, 130, 180],
    "GROUND": [81, 0, 81],
    "BRIDGE": [150, 100, 100],
    "RAILTRACK": [230, 150, 140],
    "GUARDRAIL": [180, 165, 180],
    "TRAFFICLIGHT": [250, 170, 30],
    "STATIC": [110, 190, 160],
    "DYNAMIC": [170, 120, 50],
    "WATER": [45, 60, 150],
    "TERRAIN": [145, 170, 100],
}

button_break = False

def button_click(event, time2exit=False):
    global button_break
    button_break = True
    if time2exit:
        exit(0)
        
def sample_height(world, curr_loc, dist=1005):
    labelled_pnts = (world.cast_ray(curr_loc, carla.Location(x=curr_loc.x,y=curr_loc.y,z=-dist)) or [None]) # it will return an ordered list
    if labelled_pnts:
        for l in labelled_pnts:
            if l:
                if str(l.label) != "NONE":
                    break
    if l:
        return str(l.label).upper(),l.location

def test_axis(world, curr_location, axis='x', step=1, max_trials=10, dist=1000):
    trials = 0
    label = None
    while trials<max_trials:
        world.tick()
        attrib = getattr(curr_location, axis)
        setattr(curr_location, axis, attrib+step)
        labelled_pnts = (world.cast_ray(curr_location, 
                         carla.Location(x=curr_location.x,y=curr_location.y,z=-dist)) or [None]) # it will return an ordered list

        res = labelled_pnts[0]
        if not res:
            trials += 1
        else:
            label = str(res.label)
            location = res.location
            max_value = getattr(location, axis)
    if label:
        return label, max_value


try:
    # setup client/server
    client = carla.Client('localhost', 2000)
    client.set_timeout(10)
    available_maps = client.get_available_maps()

    world = client.load_world(available_maps[0])
    settings = world.get_settings()
    settings.fixed_delta_seconds = 1./10
    settings.synchronous_mode = True
    world.apply_settings(settings)
    world.set_weather(getattr(carla.WeatherParameters, 'ClearNoon'))
    world.tick()

    #
    # Firs move from 0 until it can't find anything at least 5x. Use that as the max. 
    # Revert and do the same to find the min. Do the same for X and Y axis.
    #
    with open(f'elevation_and_semantics.csv','a') as csvfile:
        print('mapID, label, x, y, z', file=csvfile)
        for mapID in available_maps:
            print(mapID)
            world = client.load_world(mapID)
            world.tick()
            
            print("Searching for approx. map limits...")
            # find X limits
            curr_location = carla.Location(x=0.0,y=0.0,z=1000)
            res = test_axis(world, curr_location, axis='x', step=10, max_trials=10)
            if res:
                x_max = res[1]
                print(f"x_max: {x_max}")
            else:
                raise ValueError("No x_max?!?")
            
            curr_location = carla.Location(x=0.0,y=0.0,z=1000)
            res = test_axis(world, curr_location, axis='x', step=-10, max_trials=10)
            if res:
                x_min = res[1]
                print(f"x_min: {x_min}")
            else:
                raise ValueError("No x_min?!?")

            # find Y limits
            curr_location = carla.Location(x=0.0,y=0.0,z=1000)
            res = test_axis(world, curr_location, axis='y', step=10, max_trials=10)
            if res:
                y_max = res[1]
                print(f"y_max: {y_max}")
            else:
                raise ValueError("No y_max?!?")
            
            curr_location = carla.Location(x=0.0,y=0.0,z=1000)
            res = test_axis(world, curr_location, axis='y', step=-10, max_trials=10)
            if res:
                y_min = res[1]
                print(f"y_min: {y_min}")
            else:
                raise ValueError("No y_min?!?")
            
            xy = np.mgrid[x_min:x_max:1, y_min:y_max:1].reshape(2,-1).T
            np.random.shuffle(xy)

            # Start plotting...
            if SHOW_PLOT: 
                button_break = False
                fig = plt.figure()
                ax = plt.axes(projection = '3d')
                ax.view_init(elev=30., azim=50)
                ax.dist = 10

                sampled_pts = {}
                samples_float = {}
                for l in LABEL_COLORS.keys():
                    tmp, = ax.plot([], [], [], linestyle="", marker=".", color='red', markersize=5, label=l)
                    tmp.set_color(np.asarray(LABEL_COLORS[l])/255)
                    sampled_pts[l] = tmp
                    samples_float[l] = []

                # defining button and add its functionality
                axes = plt.axes([0.65, 0.05, 0.1, 0.075])
                bexit = Button(axes, 'Exit', color="red")
                bexit.on_clicked(lambda e: button_click(e, time2exit=True))
                axes = plt.axes([0.8, 0.05, 0.1, 0.075])
                bnext = Button(axes, 'Next', color="orange")
                bnext.on_clicked(button_click)

                ax.set_title(mapID)
                ax.axes.set_xlim3d(left=x_min, right=x_max)
                ax.axes.set_ylim3d(bottom=y_min, top=y_max) 
                ax.axes.set_zlim3d(bottom=-10, top=100)

                ax.legend(loc='right', bbox_to_anchor=(1, 0.5), prop={'size':6})

                plt.ion()
                plt.show()


            samples = 0
            xi_idx = 0
            curr_location = carla.Location(x=0.0,y=0.0,z=1000)
            while samples<NUMBER_OF_SAMPLES and not button_break:
                world.tick()
                x,y = xy[xi_idx]
                xi_idx += 1
                curr_location.x = x
                curr_location.y = y
                res = sample_height(world, curr_location)
                if res:
                    label, loc = res
                    
                    if loc.z < MINIMUM_Z:
                        print(f'[{samples:03d}] - {mapID}, {label}, {loc.x}, {loc.y}, {loc.z} - IGNORED!')
                        continue

                    print('mapID, label, x, y, z', file=csvfile)
                    print(f'{mapID}, {label}, {loc.x}, {loc.y}, {loc.z}', file=csvfile)
                    print(f'[{samples:03d}] - {mapID}, {label}, {loc.x}, {loc.y}, {loc.z}')

                    if SHOW_PLOT:
                        samples_float[label].append([loc.x, loc.y, loc.z])
                        tmp = np.asanyarray(samples_float[label])
                        
                        sampled_pts[label].set_data (tmp[:,0], tmp[:,1])
                        sampled_pts[label].set_3d_properties(tmp[:,2])

                        plt.draw()
                        plt.pause(0.01)

                samples += 1

            if SHOW_PLOT:
                while not button_break:
                    sleep(0.1)
                plt.close()

except Exception as err:
    print(traceback.format_exc())