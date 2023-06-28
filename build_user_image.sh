#!/bin/bash
docker build --platform linux/amd64 -t carlasim:$USER -f Dockerfile.localuser --build-arg UID=$(id -u) --build-arg GID=$(id -g) .