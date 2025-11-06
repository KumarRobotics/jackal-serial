FROM dtcpronto/ros-jazzy:base

# install bridge deps
RUN sudo apt-get update \
 && sudo apt-get install -y libzmq3-dev

# install teleop deps
RUN sudo apt-get install -y \
 ros-jazzy-serial-driver \
 ros-jazzy-asio-cmake-module

# add yourself to necessary groups
RUN sudo adduser $USER dialout
RUN sudo adduser $USER tty 
RUN sudo adduser $USER plugdev

RUN sudo apt update \
 && sudo apt install -y \
 libserial-dev \
 ros-jazzy-xacro \
 ros-jazzy-lms1xx \
 ros-jazzy-diff-drive-controller \
 ros-jazzy-controller-manager \
 ros-jazzy-joint-state-broadcaster

COPY ./jackal-bridge ws/src/jackal-bridge
COPY ./jackal-teleop ws/src/jackal-teleop
COPY ./jackal-launch ws/src/jackal-launch
COPY ./jackal-control ws/src/jackal-control
COPY ./jackal-serial ws/src/jackal-serial
COPY ./jackal-description ws/src/jackal-description

COPY ./entrypoint.bash entrypoint.bash

RUN cd ws \
 && /bin/bash -c 'source /opt/ros/jazzy/setup.bash && colcon build'

ENTRYPOINT ["./entrypoint.bash"]
