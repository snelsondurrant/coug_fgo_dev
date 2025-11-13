# Created by Nelson Durrant, Nov 2025
FROM frostlab/holoocean:2.0-runtime-ubuntu22.04-cuda12.4-ros

# Give user 'ue4' sudo permissions
USER root
RUN apt update && apt install sudo \
    && usermod -aG sudo ue4 \
    && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER ue4
WORKDIR /home/ue4

CMD ["bash"]
