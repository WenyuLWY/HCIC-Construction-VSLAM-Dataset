FROM osrf/ros:noetic-desktop-full

ARG DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

WORKDIR /
#base
RUN apt-get update && \
    apt-get install -y lsb-release curl wget vim python3-catkin-tools

# ros workspace
RUN mkdir -p /root/catkin_ws/src && \
    cd /root/catkin_ws && \
    bash -c "source /opt/ros/noetic/setup.bash && catkin build" && \
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
    
# ouster
RUN apt-get update && apt-get install -y \
    git \
    ros-noetic-pcl-ros \
    ros-noetic-rviz \
    libspdlog-dev  
RUN cd /root/catkin_ws/src && \
    git clone --recurse-submodules https://github.com/ouster-lidar/ouster-ros.git && \
    cd /root/catkin_ws && \
    bash -c "source /opt/ros/noetic/setup.bash && catkin build"
# RUN apt-get update && apt-get install -y \
#     net-tools iproute2 iputils-ping

# start
RUN echo "#!/bin/bash" > /root/start.sh && \
    echo "cd /root/catkin_ws" >> /root/start.sh && \
    echo "source /opt/ros/noetic/setup.bash" >> /root/start.sh && \
    echo "catkin build" >> /root/start.sh && \ 
    echo "source ~/catkin_ws/devel/setup.bash" >> /root/start.sh && \
    echo "exec \"\$@\"" >> /root/start.sh && \
    chmod +x /root/start.sh
WORKDIR /root
ENTRYPOINT ["/root/start.sh"]
CMD ["/bin/bash"]

