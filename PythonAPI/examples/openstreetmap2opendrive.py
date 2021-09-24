import glob
import os
import sys

import carla

"""Convert map saved from OpenStreetMaps to OpenDrive
https://carla.readthedocs.io/en/0.9.12/tuto_G_openstreetmap/
"""

# Read the .osm data
f = open("map.osm", 'r')
osm_data = f.read()
f.close()

# Define the desired settings. In this case, default values.
settings = carla.Osm2OdrSettings()
# Set OSM road types to export to OpenDRIVE
settings.set_osm_way_types(["motorway", "motorway_link", "trunk", "trunk_link", "primary", "primary_link", "secondary", "secondary_link", "tertiary", "tertiary_link", "unclassified", "residential"])

settings.generate_traffic_lights = True
# settings.all_junctions_with_traffic_lights = True

# Convert to .xodr
xodr_data = carla.Osm2Odr.convert(osm_data, settings)

# save opendrive file
f = open("map.xodr", 'w')
f.write(xodr_data)
f.close()