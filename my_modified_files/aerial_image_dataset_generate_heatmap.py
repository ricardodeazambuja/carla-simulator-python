from glob import glob
from ast import literal_eval

from os import getcwd

import numpy as np
from scipy import signal

import matplotlib.pyplot as plt

from PIL import Image, ImageFilter
from PIL.PngImagePlugin import PngInfo

import cv2

DATA_DIR = "./samples/" # images saved using aerial_image_dataset_generator.py
DRONE_RADIUS = 1 # used to generate a kernel for cost map
DILATION = 1 # 1 to do nothing
DRONE_SIZE_MULT = 5 # drone size multiplier, give more weight to places with more free space

# Values (from Gimp) used to mask the water
HSV_LOW = 135, 0, 0
HSV_HIGH = 190, 100, 100


semantic_labels = {
    0: 'Unlabeled',     # Elements that have not been categorized are considered Unlabeled. 
                        # This category is meant to be empty or at least contain elements with no collisions.
    1: 'Building',      # Buildings like houses, skyscrapers,... and the elements attached to them.
                        # E.g. air conditioners, scaffolding, awning or ladders and much more.
    2: 'Fence',         # Barriers, railing, or other upright structures. 
                        # Basically wood or wire assemblies that enclose an area of ground.
    3: 'Other',         # Everything that does not belong to any other category.
    4: 'Pedestrian',    # Humans that walk or ride/drive any kind of vehicle or mobility system.
                        # E.g. bicycles or scooters, skateboards, horses, roller-blades, wheel-chairs, etc.
    5: 'Pole',          # Small mainly vertically oriented pole. If the pole has a horizontal part (often for traffic light poles) this is also considered pole.
                        # E.g. sign pole, traffic light poles.
    6: 'RoadLine',      # The markings on the road.
    7: 'Road',          # Part of ground on which cars usually drive.
                        # E.g. lanes in any directions, and streets.
    8: 'SideWalk',      # Part of ground designated for pedestrians or cyclists. Delimited from the road by some obstacle (such as curbs or poles), not only by markings. This label includes a possibly delimiting curb, traffic islands (the walkable part), and pedestrian zones.
    9: 'Vegetation',    # Trees, hedges, all kinds of vertical vegetation. Ground-level vegetation is considered Terrain.
    10: 'Vehicles',     # Cars, vans, trucks, motorcycles, bikes, buses, trains.
    11: 'Wall',         # Individual standing walls. Not part of a building.
    12: 'TrafficSign',  # Signs installed by the state/city authority, usually for traffic regulation. This category does not include the poles where signs are attached to.
                        # E.g. traffic- signs, parking signs, direction signs...
    13: 'Sky',          # Open sky. Includes clouds and the sun.
    14: 'Ground',       # Any horizontal ground-level structures that does not match any other category. For example areas shared by vehicles and pedestrians, or flat roundabouts delimited from the road by a curb.
    15: 'Bridge',       # Only the structure of the bridge. Fences, people, vehicles, an other elements on top of it are labeled separately.
    16: 'RailTrack',    # All kind of rail tracks that are non-drivable by cars.
                        # E.g. subway and train rail tracks.
    17: 'GuardRail',    # All types of guard rails/crash barriers.
    18: 'TrafficLight', # Traffic light boxes without their poles.
    19: 'Static',       # Elements in the scene and props that are immovable.
                        # E.g. fire hydrants, fixed benches, fountains, bus stops, etc.
    20: 'Dynamic',      # Elements whose position is susceptible to change over time.
                        # E.g. Movable trash bins, buggies, bags, wheelchairs, animals, etc.
    21: 'Water',        # Horizontal water surfaces.
                        # E.g. Lakes, sea, rivers.
    22: 'Terrain',      # Grass, ground-level vegetation, soil or sand. 
                        # These areas are not meant to be driven on. This label includes a possibly delimiting curb.
}

PLACES2LAND = ["Terrain", "Ground", "Other", "SideWalk", "Unlabeled", "Static", "Sky"]

# Everything that is not a good place to land on is an obstacle here
obstacle_ids = [i for i in semantic_labels if semantic_labels[i] not in PLACES2LAND]



# Files filtered according to the sensor type
depth_samples = sorted(glob(DATA_DIR+'*depth*.png'))
rgb_samples = sorted(glob(DATA_DIR+'*rgb*.png'))
instance_segmentation_samples = sorted(glob(DATA_DIR+'*instance*.png'))


prev_sample = None
num_weather_patterns = 0
for ri in rgb_samples:
    key_values = ri.split('/')[-1].split("_")
    curr_sample = int(key_values[0])
    if prev_sample == None:
        prev_sample = curr_sample
        num_weather_patterns += 1
        continue
    elif prev_sample != curr_sample:
        break
    prev_sample = curr_sample
    num_weather_patterns += 1
    
print(f"num_weather_patterns: {num_weather_patterns}") # the whole dataset should be using the same number

final_contour_values = []
for idx,_ in enumerate(instance_segmentation_samples):
    sample_number = idx+1

    # We need to get the first one because it has the standard weather that
    # helps with the water segmentation corrections...
    img_idx_rgb = sample_number*num_weather_patterns - num_weather_patterns
    img_idx = sample_number - 1

    basename = '_'.join(rgb_samples[img_idx_rgb].split('/')[-1].split('.')[0].split('_')[:3])

    filtered_semantics_file = '/'.join(rgb_samples[img_idx_rgb].split('/')[:2]) + '/' + basename + '_filtered_semantics.png'
    heatmap_numpy_file = '/'.join(rgb_samples[img_idx_rgb].split('/')[:2]) + '/' + basename + '_heatmap.npz'
    heatmap_file = '/'.join(rgb_samples[img_idx_rgb].split('/')[:2]) + '/' + basename + '_heatmap.png'

    print(f"Processing sample: {rgb_samples[img_idx_rgb]}")
    img_rgb = Image.open(rgb_samples[img_idx_rgb])

    curr_pos_yaw = literal_eval(img_rgb.text['XYZYAW'])
    fov = literal_eval(img_rgb.text['FOV'])

    view_width = img_rgb.size[0]
    view_height = img_rgb.size[1]

    calibration = np.identity(3)
    calibration[0, 2] = view_width / 2
    calibration[1, 2] = view_height / 2
    calibration[0, 0] = calibration[1, 1] = view_width / (2 * np.tan((fov*np.pi/180)/2))
        
    img = Image.open(instance_segmentation_samples[img_idx])
    only_semantics = np.array(img)[...,2]


    # CARLA has problems with the water label (21) and it labels as water some crazy places
    # This is a mask to fix that.

    # These values are for Town01
    h_gimp2opencv = lambda h: 179*h/360
    s_gimp2opencv = lambda s: 255*s/100
    v_gimp2opencv = lambda h: 255*h/100

    # img_rgb needs to be the first one because the weather is set to be the same
    # and that will help with the hsv color matching...
    frame_HSV = cv2.cvtColor(np.array(img_rgb), cv2.COLOR_RGB2HSV)


    low_H = h_gimp2opencv(HSV_LOW[0])
    low_S = s_gimp2opencv(HSV_LOW[1])
    low_V = v_gimp2opencv(HSV_LOW[2])

    high_H = h_gimp2opencv(HSV_HIGH[0])
    high_S = s_gimp2opencv(HSV_HIGH[1])
    high_V = v_gimp2opencv(HSV_HIGH[2])
    frame_threshold = cv2.inRange(frame_HSV, (low_H, low_S, low_V), (high_H, high_S, high_V))

    erosion_size = 2
    element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * erosion_size + 1, 2 * erosion_size + 1),
                                        (erosion_size, erosion_size))

    frame_threshold = cv2.morphologyEx(frame_threshold, cv2.MORPH_CLOSE, element)
    frame_threshold = cv2.morphologyEx(frame_threshold, cv2.MORPH_OPEN, element)
    not_water = cv2.bitwise_not(frame_threshold)

    labeled_water = np.where(only_semantics==21,1,0).astype('uint8')*255

    final_mask = cv2.bitwise_not(cv2.bitwise_and(labeled_water,not_water))

    # this will make the "not water" to get the label "Unlabeled" (i.e. 0)
    filtered_semantics = cv2.bitwise_and(only_semantics, only_semantics, mask=final_mask)


    img_mono = np.array(filtered_semantics)

    # Slicing the image according to the obstacles
    # to make the dilation easier (?!?)
    img_mono_dilated = np.zeros(img_mono.shape)
    for i in obstacle_ids:
        tmp_img = img_mono.copy()
        # Keep only the current obstacle class (1), 
        # everything else becomes 0
        tmp_img[tmp_img!=i] = 0
        tmp_img[tmp_img==i] = 1
        
        # Apply a dilation
        tmp_img = np.array(Image.fromarray(tmp_img).filter(ImageFilter.MaxFilter(DILATION)))
        #tmp_img = np.array(Image.fromarray(tmp_img).filter(ImageFilter.GaussianBlur(DILATION)))
        
        # save this slice
        img_mono_dilated += tmp_img

    # The dilation will "leak" over different places because the size 
    # of the obstacle in the slice will increase.
    # To keep it binary, everything that is not 0 (place good to land on)
    # will be turned into 1, fusing all obstacles
    img_mono_dilated[img_mono_dilated>0] = 1

    def create_drone_patch(w,h):
        # based on https://stackoverflow.com/a/44874588/7658422
        center = (int(w/2), int(h/2))
        radius = min(center[0], center[1], w-center[0], h-center[1])
        Y, X = np.ogrid[:h, :w]
        dist_from_center = np.sqrt((X - center[0])**2 + (Y-center[1])**2)
        drone_patch = (dist_from_center <= radius) + 0
        return drone_patch

    # Altitude used for sampling (dataset generation)
    _,_,z,_ = curr_pos_yaw

    # Get image plane coordinates for a square with side 2xDRONE_RADIUS at dist=z
    # using an approximating as a pinhole camera
    img_plane = np.dot(calibration,[2*DRONE_RADIUS,2*DRONE_RADIUS,z])*np.array([1/z, 1/z, 1])

    # Rounds it up
    drone_patch_shape = np.ceil(abs(calibration[0:2,-1]-img_plane[[0,1]])).astype(int)

    # Generates the patch 'kernel' (image plane)
    drone_patch = create_drone_patch(*drone_patch_shape)

    print(f"drone_patch_shape: {drone_patch_shape}")


    # Change obstacle value to 0
    img_mono_dilated_inv = np.invert(img_mono_dilated==1)+0

    # Calculate correlation of the drone_patch over the mask.
    # It will blur the landing areas reducing its size to account
    # for the size of the drone.
    corr_res = signal.correlate2d(img_mono_dilated_inv, drone_patch, boundary='symm', mode='same')

    # Normalize it so the good places to land will have value 1
    corr_res = corr_res/corr_res.max()

    # All areas with a value smaller than one mean
    # the drone patch didn't fit perfectly.
    # Therefore, we will threshold to make a binary mask
    corr_res_thrs = corr_res.copy()
    corr_res_thrs[corr_res_thrs<1] = 0

    mask = (corr_res_thrs*255).astype('uint8')
    # https://docs.opencv.org/4.6.0/d9/d8b/tutorial_py_contours_hierarchy.html
    # hierarchy => [Next, Previous, First_Child, Parent], -1 indicates None
    # RETR_TREE => retrieves all the contours and creates a full family hierarchy list
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


    # Filter contours according to area
    # Calculate objective = area - perimeter/area

    filtered_contours = []
    for ci,contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if area > 0:
            perimeter = cv2.arcLength(contour,True)
            #M = cv.moments(contour)
            #cx = int(M['m10']/M['m00'])
            #cy = int(M['m01']/M['m00'])
            
            hull = cv2.convexHull(contour)
            #approx = cv2.approxPolyDP(contour,perimeter*0.1,True)
            perimeter_hull = cv2.arcLength(hull,True)
            #print(perimeter/cv2.arcLength(approx,True), perimeter/cv2.arcLength(hull,True))
            
            # It may include obstacles
            filtered_contours.append([ci,perimeter,area,perimeter_hull])
        else:
            # Although we don't care about contours with area==0,
            # the hierarchy will need the indices to match the original
            filtered_contours.append([ci,None,None,None])

    filtered_contours = np.array(filtered_contours)

    # Option #1
    centre_i = view_height/2
    centre_j = view_width/2

    max_dist = np.sqrt((centre_i)**2+(centre_j)**2)

    # Generate the distance array
    local_indices = np.transpose(np.nonzero(np.ones((view_height,view_width), dtype=float)))
    dists = (max_dist-np.sqrt(((local_indices-[centre_i,centre_j])**2).sum(axis=1)))
    dists -= dists.min()
    dists /= dists.max()
    final_dists = np.zeros((view_height,view_width), dtype=float)
    for i,(j,k) in enumerate(local_indices):
        final_dists[j,k] = dists[i]

    max_objective_all = 0
    max_dist_all = 0
    final_contour = np.zeros((view_height,view_width), dtype='float')
    for i,perimeter,area,p_hull in filtered_contours:
        if perimeter == None:
            # It's one of the area==0, so ignore it
            continue
        ci = int(i) # because the numpy array dtype is float...
        contour = contours[ci]
        h = hierarchy[0][ci]
        # h = [Next, Previous, First_Child, Parent]
        if h[3]!=-1: # skip children if it's an obstacle
            individual_contour = np.zeros((view_height,view_width), dtype='uint8')
            cv2.drawContours(individual_contour, [contour], 0, 1, cv2.FILLED)
            matches = np.logical_and(corr_res_thrs,individual_contour).sum()
            if (matches/area)<0.5: # obstacles won't match the area
                continue
        
        children = []
        children_area = 0
        children_perimeter = 0
        children_p_hull = 0
        if h[2]!=-1: # it has children
            for k,_,_,_ in filtered_contours:
                k = int(k)
                if filtered_contours[k][1]:
                    if hierarchy[0][k][3]==ci: 
                        children.append(k) # children with ci as father
                        _, tmp_perimeter, tmp_area, tmp_p_hull = filtered_contours[k]
                        children_area += tmp_area
                        children_perimeter += tmp_perimeter
                        children_p_hull += tmp_p_hull
        
        # 1. The bigger the area, the easier to land
        # 2. perimeter/area will tell how complex the contour is, and it's better to land on simple ones
        #    However, perimeter/perimeter_hull will penalize the difference between the two perimeters
        complexity_weight = ((p_hull+children_p_hull)/(perimeter + children_perimeter))
        objective = complexity_weight*(area - children_area)
        objective = 0 if objective < 0 else objective
        max_objective_all = objective if objective > max_objective_all else max_objective_all
        
        individual_contour = np.zeros((view_height,view_width), dtype='float')
        cv2.drawContours(individual_contour, [contour], 0, 1, cv2.FILLED)
        if children:
            for k in children:
                # Cuts the children from the parent
                cv2.drawContours(individual_contour, [contours[k]], 0, 0, cv2.FILLED)
        
        # 3. The closer to the drone the pixel inside a contour is, the bigger the objective value
        # Therefore the distance acts as a weight(gain) multiplying the objective value.
        indices = np.transpose(np.nonzero(individual_contour>0))
        # dists = (max_dist-np.sqrt(((indices-[centre_i,centre_j])**2).sum(axis=1)))
        # max_dist_all = dists.max() if dists.max() > max_dist_all else max_dist_all
        for idx,(r,c) in enumerate(indices):
            individual_contour[r,c] = objective # *dists[idx]
        if len(indices):
            final_contour += individual_contour

    drone_patch = create_drone_patch(*(DRONE_SIZE_MULT*drone_patch_shape))
    final_corr = signal.correlate2d(final_contour, drone_patch, boundary='symm', mode='same')
    final_corr -= final_corr.min()
    final_corr /= final_corr.max()
    final_contour *= final_corr

    final_contour *= final_dists

    print(f"final_contour max: {final_contour.max()}, min: {final_contour.min()}")
    final_contour_values.append(final_contour.max())

    print(f"Saving: {filtered_semantics_file}")
    Image.fromarray(filtered_semantics).save(filtered_semantics_file)
    print(f"Saving: {heatmap_numpy_file}")
    np.savez_compressed(heatmap_numpy_file, final_contour)
    print(f"Saving: {heatmap_file}")
    Image.fromarray((255*(final_contour/final_contour.max())).astype('uint8')).save(heatmap_file)

final_contour_values_filename = '/'.join(rgb_samples[0].split('/')[:2]) + '/' + "final_contour_values.npz"
print(f"Almost done... saving: {final_contour_values_filename}")
np.savez_compressed(final_contour_values_filename, final_contour_values)