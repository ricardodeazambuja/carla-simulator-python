#!/bin/sh

# echo "Starting avahi..."
# sudo avahi-daemon -D

unset SDL_VIDEODRIVER

echo "Starting simulator..."
./CarlaUE4.sh -vulkan -nosound &

echo "Press ENTER to kill it!"
echo
read USELESS

echo
echo "Killing simulator..."
echo

pkill Carla
