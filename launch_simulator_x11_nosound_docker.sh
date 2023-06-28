#!/bin/bash

tmp_ips=$(hostname -I) # worst scenario: avahi-publish won't do its work...
tmp_array=($tmp_ips) 
avahi-publish -a -R carla-container.local ${tmp_array[0]} &>/dev/null &
export AVAHI_PUB_PID=$!
trap 'echo Killing avahi-publish with PID $AVAHI_PUB_PID && kill $AVAHI_PUB_PID' EXIT

XAUTHORITY=
FILE=$HOME/.Xauthority
if [ -f "$FILE" ]; then
    XAUTHORITY="--mount type=bind,source=/home/$USER/.Xauthority,target=/home/carla/.Xauthority"
else 
    echo "If you are launching this through a remote computer connected using ssh -X, it won't work because you don't have $FILE."
    echo "Just use 'touch ~/.Xauthority' and log out/in and it will be fine."
fi

echo "Launching CARLA... it's a complex town, so it may take a while"

docker run --rm --gpus 'all,"capabilities=graphics,utility,display,video,compute"' -it \
--name carla-container --network host \
--user $(id -u):$(id -g) --group-add sudo \
--env=DISPLAY=$DISPLAY --env=QT_X11_NO_MITSHM=1 \
--volume /tmp/.X11-unix:/tmp/.X11-unix \
$XAUTHORITY \
carlasim:$USER \
./launch_nosound.sh -nosound \