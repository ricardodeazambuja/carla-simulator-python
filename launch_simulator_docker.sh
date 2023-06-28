#!/bin/bash

tmp_ips=$(hostname -I) # worst scenario: avahi-publish won't do its work...
tmp_array=($tmp_ips) 
avahi-publish -a -R carla-container.local ${tmp_array[0]} &>/dev/null &
export AVAHI_PUB_PID=$!
trap 'echo Killing avahi-publish with PID $AVAHI_PUB_PID && kill $AVAHI_PUB_PID' EXIT

docker run --rm -it \
--name carla-container \
--network=host \
--user carla \
--gpus 0 \
ricardodeazambuja/carlasim:0.9.13_headless ./launch_headless.sh
