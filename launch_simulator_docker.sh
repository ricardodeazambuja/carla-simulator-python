#!/bin/bash

docker run --rm -it --name carla-container -u carla -p 2000-2002:2000-2002 --gpus 0 ricardodeazambuja/carlasim:0.9.13_headless ./launch_headless.sh
