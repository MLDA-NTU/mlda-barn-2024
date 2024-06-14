FROM ros:melodic

RUN mkdir -p /jackal_ws/src
WORKDIR /jackal_ws/src

RUN apt install git


RUN git clone https://github.com/jackal/jackal.git --branch melodic-devel
RUN git clone https://github.com/jackal/jackal_simulator.git --branch melodic-devel
RUN git clone https://github.com/jackal/jackal_desktop.git --branch melodic-devel
RUN git clone https://github.com/MLDA-NTU/mlda-barn-2024.git --branch Hard-Deadline-dev
# RUN git clone https://gitlab.kuleuven.be/u0144428/free_space_motion_tube.git --branch barn2023

WORKDIR /jackal_ws/src/mlda-barn-2024/free_space_motion_tube
RUN mkdir -p build
WORKDIR /jackal_ws/src/mlda-barn-2024/free_space_motion_tube/build
RUN cmake ..
RUN make -j8 && make install

RUN apt-get update && apt-get install -y python3-pip build-essential
RUN pip3 install --upgrade pip
RUN pip3 install defusedxml rospkg netifaces numpy jupyter scipy matplotlib casadi

# Install ROS components
RUN apt-get install -y ros-melodic-desktop-full ros-melodic-gmapping \
    ros-melodic-robot-localization ros-melodic-joint-state-publisher-gui ros-melodic-navigation \
    ros-melodic-hector-gazebo-plugins ros-melodic-velodyne-description ros-melodic-rosdoc-lite \
    ros-melodic-twist-mux ros-melodic-sick-tim ros-melodic-teleop-twist-joy ros-melodic-teleop-twist-keyboard ros-melodic-pointgrey-camera-description \
    ros-melodic-interactive-marker-twist-server ros-melodic-lms1xx ros-melodic-laser-pipeline ros-melodic-controller-manager ros-melodic-hector-mapping

WORKDIR /jackal_ws

# Add start script
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN echo "if [ ! -d /jackal_ws/devel ]; then ">> ~/.bashrc
RUN echo "  cd /jackal_ws" >> ~/.bashrc
RUN echo "  catkin_make" >> ~/.bashrc
RUN echo "else" >> ~/.bashrc
RUN echo "  source /jackal_ws/devel/setup.bash;" >> ~/.bashrc
RUN echo "fi" >> ~/.bashrc
RUN echo "alias python=python3" >> ~/.bashrc
RUN echo "cd /jackal_ws/src/" >> ~/.bashrc


