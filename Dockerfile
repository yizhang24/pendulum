FROM osrf/ros:kilted-desktop-full

RUN apt update && apt install -y openssh-server
RUN mkdir /var/run/sshd
RUN echo "root:root" | chpasswd
RUN sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config
RUN sed -i 's/#PasswordAuthentication yes/PasswordAuthentication yes/' /etc/ssh/sshd_config
EXPOSE 22

RUN apt install -y ros-$ROS_DISTRO-foxglove-bridge

SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/kilted/setup.bash" >> ~/.bashrc

WORKDIR /root/project
CMD ["/bin/bash", "-c", "/usr/sbin/sshd -D"]
