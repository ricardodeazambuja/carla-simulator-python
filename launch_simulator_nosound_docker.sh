#!/bin/bash

docker run --rm --gpus 'all,"capabilities=graphics,utility,display,video,compute"' -it \
--name carla-container --hostname=carla-container \
--user $(id -u):$(id -g) --group-add sudo \
--env=DISPLAY=$DISPLAY --env=QT_X11_NO_MITSHM=1 \
--volume /tmp/.X11-unix:/tmp/.X11-unix \
-p 2000-2002:2000-2002 ricardodeazambuja/carlasim:0.9.13_headless \
./launch_nosound.sh -nosound \