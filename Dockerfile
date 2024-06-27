FROM monkey/humble:no-gpu

# Install packages 
RUN apt-get update \
    && apt-get install -y \
    python3-pip \
    cmake \
    # ros-humble-behaviortree-cpp \
    && pip install python-can[serial] \
    && rm -rf /var/lib/apt/lists/*

# Create a non-root user
ARG USERNAME=monkey
USER $USERNAME

ARG HOME_DIR=/home/${USERNAME}

# Clone BTCPP and BTROS2 and build dep_ws
WORKDIR ${HOME_DIR}/dep_ws/src
RUN git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git
RUN git clone https://github.com/BehaviorTree/BehaviorTree.ROS2.git -b humble

# Renaming BehaviorTree.CPP --> behaviortree_cpp (so BTROS2 can find the package)
RUN mv BehaviorTree.CPP behaviortree_cpp

# Building BTCPP using cmake and make
WORKDIR ${HOME_DIR}/dep_ws/src/behaviortree_cpp
RUN mkdir build ; cd build
RUN cmake ..
    && make
    && sudo install make

# Reverting back the BTROS2 that does not use generate_parameters_library
WORKDIR ${HOME_DIR}/dep_ws/src/BehaviorTree.ROS2
RUN git reset --hard 374edcf

# Source and build BTROS2
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build

# WORKDIR ${HOME_DIR}/dep_ws/src
# RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
#     && cd ${HOME_DIR}/dep_ws/ \
#     && colcon build

# Copy and build arm_ws
RUN mkdir /home/${USERNAME}/arm_ws/

COPY . /home/${USERNAME}/arm_ws/src

RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && . ${HOME_DIR}/dep_ws/install/setup.sh \
    && cd /home/${USERNAME}/arm_ws/ \
    && colcon build

USER root

# Set up .bashrc scripts
RUN echo "source /home/${USERNAME}/arm_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc

WORKDIR /home/${USERNAME}

CMD ["bash"]
