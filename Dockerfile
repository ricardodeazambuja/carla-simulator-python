# ARG CARLA_VERSION=0.9.14
# FROM carlasim/carla:${CARLA_VERSION} as base

# 0.9.13
FROM carlasim/carla@sha256:2c1a59808792b99233c92dcdab6afb575357b863a00f7ff44b3ae096f648af12 

# 0.9.14
# FROM carlasim/carla@sha256:6ea4ed1b07fd017097b155d11501713f99b3a11a99fe695ef5c8615ba95adbe1
# To find the hash: $ docker images --digests


#
# Just to get rid of the annoying error message...
# See https://answers.unrealengine.com/questions/1039260/suppress-or-fix-xdg-user-dir-not-found.html
USER root

# RUN apt-get install -y --no-install-recommends wget && \
#   wget https://developer.download.nvidia.com/compute/cuda/repos/$distro/$arch/cuda-keyring_1.0-1_all.deb && \
#   sudo dpkg -i cuda-keyring_1.0-1_all.deb
RUN  rm /etc/apt/sources.list.d/nvidia-ml.list /etc/apt/sources.list.d/cuda.list && \
  apt-get update && apt-get install -y --no-install-recommends \
  xdg-user-dirs \
  wget \
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

ADD --chown=carla:carla launch_headless.sh /home/carla/launch_headless.sh
ADD --chown=carla:carla launch_nosound.sh /home/carla/launch_nosound.sh

# Make the script above executable
RUN chmod +x /home/carla/launch_headless.sh
RUN chmod +x /home/carla/launch_nosound.sh

# # Download and install additional maps
# RUN wget -O AdditionalMaps_0.9.13.tar.gz https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/AdditionalMaps_0.9.13.tar.gz && \
#     tar -xf AdditionalMaps_0.9.13.tar.gz && \
#     rm AdditionalMaps_0.9.13.tar.gz
#     # Remove Town11 as it doesn't seem to work...


#
# EXPOSE 2000-2002
# ENTRYPOINT /home/carla/launch_headless.sh
# The two lines above allow you to launch it only using 
# docker run --rm -it ricardodeazambuja/carlasim:0.9.13_headless

# The command below sets the ports visible by the host to a different range (2020-2022) and gives a name to the container (carla-container)
# docker run --rm -it --name carla-container -u carla -p 2000-2002:2020-2022 --gpus 0 ricardodeazambuja/carlasim:0.9.13_headless ./launch_headless.sh


#
# The final image will be around 22GB instead of the original 16GB! One strategy to reduce the size is to export the image and reimport it.
# First, we need a container to export the filesystem from:
# $ docker run --rm -it --name carla-container --hostname=carla-container --user carla carlasim:0.9.13_headless bash
# Then export to carla-sim.tar:
# $ docker --output="carla-sim.tar" carla-container
# Import from carla-sim.tar:
# $ cat carla-sim.tar | docker import \
# --change "ENV LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/usr/lib/i386-linux-gnu:/usr/local/nvidia/lib:/usr/local/nvidia/lib64:/usr/local/nvidia/lib:/usr/local/nvidia/lib64" \
# --change "ENV NVIDIA_VISIBLE_DEVICES=all" \
# --change "ENV CUDA_PKG_VERSION=10-1=10.1.243-1" \
# --change "ENV CUDA_VERSION=10.1.243" \
# --change "ENV NVIDIA_DRIVER_CAPABILITIES=compute,graphics,utility" \
# --change "ENV NVIDIA_REQUIRE_CUDA=cuda>=10.1 brand=tesla,driver>=384,driver<385 brand=tesla,driver>=396,driver<397 brand=tesla,driver>=410,driver<411" \
# --change "ENV PATH=/usr/local/nvidia/bin:/usr/local/cuda/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" \
# --change "WORKDIR /home/carla/"  - ricardodeazambuja/carlasim:0.9.13_headless
#
# On problem related to the exporting/importing idea above is the loss of the previous layer information. If this image was reusing other images, that
# information will be lost and it may actually take more disk space after all.
