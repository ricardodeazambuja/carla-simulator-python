# ARG CARLA_VERSION=0.9.13
# FROM carlasim/carla:${CARLA_VERSION} as base
FROM carlasim/carla@sha256:2c1a59808792b99233c92dcdab6afb575357b863a00f7ff44b3ae096f648af12
# To find the hash: $ docker images --digests


#
# Just to get rid of the annoying error message...
# See https://answers.unrealengine.com/questions/1039260/suppress-or-fix-xdg-user-dir-not-found.html
USER root

RUN apt-get update
RUN apt-get install -y --no-install-recommends \
  xdg-user-dirs \
  vim \
  sudo \
  avahi-daemon \
  libnss-mdns \
  && apt-get -qq -y autoclean \
  && apt-get -qq -y autoremove \
  && apt-get -qq -y clean \
  && rm -rf /var/lib/apt/lists/*

RUN usermod -aG sudo carla
RUN echo "carla    ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers 

COPY avahi-daemon.conf /etc/avahi/avahi-daemon.conf

USER carla
# Done!

# Download the headless helper script
# ADD --chown=carla:carla https://raw.githubusercontent.com/ricardodeazambuja/carla-simulator-python/main/launch_headless.sh /home/carla/launch_headless.sh
ADD --chown=carla:carla launch_headless.sh /home/carla/launch_headless.sh

# Make the script above executable
RUN chmod +x /home/carla/launch_headless.sh

#
# EXPOSE 2000-2002
# ENTRYPOINT /home/carla/launch_headless.sh
# The two lines above allow you to launch it only using 
# docker run --rm -it ricardodeazambuja/carlasim:0.9.13_headless

# The command below sets the ports visible by the host to a different range (2020-2022) and gives a name to the container (carla-container)
# docker run --rm -it --name carla-container -u carla -p 2000-2002:2020-2022 --gpus 0 ricardodeazambuja/carlasim:0.9.13_headless ./launch_headless.sh