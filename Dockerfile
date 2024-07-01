FROM monkey/humble:no-gpu

# Install packages 
RUN apt-get update \
    && apt-get install -y \
    python3-pip \
    cmake \
    libzmq3-dev \
    libboost-dev \
    ros-humble-behaviortree-cpp \
    && pip install python-can[serial] \
    && rm -rf /var/lib/apt/lists/*

# Create a non-root user
# ARG USERNAME=monkey
# USER $USERNAME

ARG HOME_DIR=/root

# Clone BTCPP and BTROS2 and build dep_ws
WORKDIR ${HOME_DIR}/dep_ws/src
RUN git clone https://github.com/BehaviorTree/BehaviorTree.ROS2.git -b humble

# == ---------------------------------------------------------- == #
# == NO NEED THIS PART BECAUSE HAVE ros-humble-behaviortree-cpp == #

# RUN git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git
# RUN mv BehaviorTree.CPP behaviortree_cpp

# Building BTCPP using cmake and make
# WORKDIR ${HOME_DIR}/dep_ws/src/behaviortree_cpp
# RUN mkdir build

# WORKDIR ${HOME_DIR}/dep_ws/src/behaviortree_cpp/build
# RUN cmake .. \
#     && make \
#     && sudo make install

# == ---------------------------------------------------------- == #

# Reverting back the BTROS2 that does not use generate_parameters_library
WORKDIR ${HOME_DIR}/dep_ws/src/BehaviorTree.ROS2
RUN git reset --hard ce923e1 \
    && . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build
    # Set up .bashrc scripts
RUN echo "source /root/dep_ws/src/BehaviorTree.ROS2/install/setup.bash" >> /root/.bashrc

# Copy and build arm_ws
RUN mkdir /root/arm_ws/
COPY monkey_arm /root/arm_ws/src

WORKDIR ${HOME_DIR}/arm_ws/src

# WORKDIR ${HOME_DIR}/arm_ws/src

RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && . /root/dep_ws/src/BehaviorTree.ROS2/install/setup.sh \
    && colcon build
   
# USER root

# Set up .bashrc scripts
RUN echo "source /root/arm_ws/src/install/setup.bash" >> /root/.bashrc

WORKDIR /root

CMD ["bash"]