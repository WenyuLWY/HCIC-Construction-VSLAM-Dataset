FROM osrf/ros:melodic-desktop-full

ARG DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

WORKDIR /root
#base
# RUN apt-get update && \
#     apt-get install -y lsb-release curl wget vim gnupg2 build-essential python3-catkin-tools  python3-pip  git

#after install ros-melodic images
RUN apt-get update && \
    apt-get install -y  curl wget git python3-catkin-tools python3-pip libglew-dev

    
RUN git clone https://gitee.com/wenyu-li/Pangolin.git && \
    cd Pangolin && \
    git checkout v0.6 && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j && \
    make install && \
    ldconfig
    
# ros workspace
RUN mkdir -p /root/catkin_ws/src && \
    cd /root/catkin_ws && \
    bash -c "source /opt/ros/melodic/setup.bash && catkin build" && \
    echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc && \
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

RUN apt-get update && apt-get install -y  vim
RUN pip3 install rosdepc && \
    rosdepc init  && \
    rosdepc update
    
RUN echo "export ROS_PACKAGE_PATH=/root/catkin_ws/src/ORB_SLAM3-RGBD-Inertial/orbslam3:/opt/ros/melodic/share:/root/catkin_ws/src/ORB_SLAM3-RGBD-Inertial/Examples/ROS" >> ~/.bashrc

# start
RUN echo "#!/bin/bash" > /root/start.sh && \
    echo "cd /root/catkin_ws/src/ORB_SLAM3-RGBD-Inertial" >> /root/start.sh && \
    echo "chmod +x build.sh" >> /root/start.sh && \ 
    echo "chmod +x build_ros.sh" >> /root/start.sh && \  
    echo "./build.sh" >> /root/start.sh && \
    echo "cd /root/catkin_ws" >> /root/start.sh && \
    echo "source /opt/ros/melodic/setup.bash" >> /root/start.sh && \
    echo "catkin build orbslam3" >> /root/start.sh && \ 
    echo "source ~/catkin_ws/devel/setup.bash" >> /root/start.sh && \
    echo "exec \"\$@\"" >> /root/start.sh && \
    chmod +x /root/start.sh
WORKDIR /root
ENTRYPOINT ["/root/start.sh"]
CMD ["/bin/bash"]

