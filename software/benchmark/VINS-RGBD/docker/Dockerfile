FROM osrf/ros:melodic-desktop-full

ARG DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

WORKDIR /
#base
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA && \
    apt-get update && \
    apt-get install -y lsb-release curl wget vim python3-catkin-tools

# ros workspace
RUN mkdir -p /root/catkin_ws/src && \
    cd /root/catkin_ws && \
    bash -c "source /opt/ros/melodic/setup.bash && catkin build" && \
    echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc && \
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
    
RUN git clone https://github.com/WenyuLWY/Sophus.git && \
    cd Sophus && \
    mkdir build && \
    cd build && \
    cmake .. && \
    sudo make install
RUN apt-get update && apt-get install -y libceres-dev

RUN apt-get update && apt-get install -y \
    ros-melodic-imu-tools ros-melodic-rtabmap*
    
RUN echo "#!/bin/bash" > /root/start.sh && \
    echo "cd /root/catkin_ws" >> /root/start.sh && \
    echo "source /opt/ros/melodic/setup.bash" >> /root/start.sh && \
    echo "catkin build" >> /root/start.sh && \ 
    echo "source ~/catkin_ws/devel/setup.bash" >> /root/start.sh && \
    echo "exec \"\$@\"" >> /root/start.sh && \
    chmod +x /root/start.sh
WORKDIR /root
ENTRYPOINT ["/root/start.sh"]
CMD ["/bin/bash"]
