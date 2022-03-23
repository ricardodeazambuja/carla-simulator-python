import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D # https://stackoverflow.com/a/56222305/7658422
from matplotlib.widgets import Button


MAPID = "/Game/Carla/Maps/Town07_Opt"

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


with open("../elevation_and_semantics.csv") as csvfile:
    lines = csvfile.readlines()
    # mapID, seed, idx, samples, label, x, y, z
    labels = []
    positions = []
    for l in lines[1:]:
        mapID, seed, idx, samples, label, x, y, z = l.strip().split(", ")
        if mapID != MAPID:
            continue
        labels.append(label)
        positions.append([float(x),float(y),float(z)])


fig = plt.figure()
ax = plt.axes(projection = '3d')
ax.view_init(elev=30., azim=50)
ax.dist = 10

postionsbylabel = {k:[] for k in LABEL_COLORS.keys()}
for label, position in zip(labels, positions):
    postionsbylabel[label].append(position)

for l,pos in postionsbylabel.items():
    if pos and l != "NONE":
        pos = np.asanyarray(pos)
        tmp, = ax.plot(pos[:,0], pos[:,1], pos[:,2], linestyle="", marker=".", markersize=5, label=l)
        tmp.set_color(np.asarray(LABEL_COLORS[l])/255)

ax.set_title(MAPID)
ax.legend(loc='right', bbox_to_anchor=(1, 0.5), prop={'size':6})

# ax.axes.set_xlim3d(left=x_min, right=x_max)
# ax.axes.set_ylim3d(bottom=y_min, top=y_max) 
# ax.axes.set_zlim3d(bottom=-10, top=100)

plt.show()