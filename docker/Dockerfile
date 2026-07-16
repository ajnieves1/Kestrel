FROM ros:jazzy

# Pin ArduPilot so image builds reproduce, resolved per plan A23
ARG ARDUPILOT_TAG=Copter-4.6.3

# Core ROS, MAVROS, Gazebo bridge, vision and build tooling
RUN apt-get update && apt-get install -y \
    ros-jazzy-desktop \
    ros-jazzy-mavros \
    ros-jazzy-mavros-extras \
    ros-jazzy-ros-gz \
    ros-jazzy-vision-msgs \
    ros-jazzy-cv-bridge \
    python3-opencv \
    python3-pip \
    git sudo wget curl lsb-release gnupg \
    rapidjson-dev \
    && rm -rf /var/lib/apt/lists/*

# Add the OSRF apt repository, libgz-sim8-dev and gz-harmonic both live there
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gz-stable.list \
    && apt-get update \
    && apt-get install -y gz-harmonic libgz-sim8-dev \
    && rm -rf /var/lib/apt/lists/*

# MAVROS connects but misbehaves without the geographiclib datasets
RUN /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh

# Give the stock ubuntu user passwordless sudo, uid 1000 matches the host user
RUN echo "ubuntu ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/ubuntu \
    && chmod 0440 /etc/sudoers.d/ubuntu

USER ubuntu
WORKDIR /home/ubuntu

# Docker never exports USER for RUN shells, ArduPilot prereqs script needs it
ENV USER=ubuntu

# Clone and build ArduPilot SITL once so first launch is fast
RUN git clone --branch ${ARDUPILOT_TAG} --depth 1 --recurse-submodules \
    https://github.com/ArduPilot/ardupilot.git
RUN cd ardupilot && Tools/environment_install/install-prereqs-ubuntu.sh -y

# The prereqs script builds a venv holding future, pymavlink, and friends,
# a plain RUN shell starts fresh so waf needs the venv active to see them
RUN cd ardupilot \
    && . /home/ubuntu/venv-ardupilot/bin/activate \
    && ./waf configure --board sitl \
    && ./waf copter

# The plugin needs GStreamer headers for its camera streaming code
RUN sudo apt-get update \
    && sudo apt-get install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    && sudo rm -rf /var/lib/apt/lists/*

# Clone and build the ArduPilot Gazebo plugin against Harmonic
RUN git clone https://github.com/ArduPilot/ardupilot_gazebo.git && \
    cd ardupilot_gazebo && \
    GZ_VERSION=harmonic cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo && \
    cmake --build build -j4

# Vision inference, LLM report writer SDKs, and MAVLink tooling
RUN sudo pip3 install --break-system-packages onnxruntime anthropic openai google-genai pymavlink MAVProxy pytest-timeout

ENV PATH=/home/ubuntu/ardupilot/Tools/autotest:${PATH}
ENV GZ_SIM_SYSTEM_PLUGIN_PATH=/home/ubuntu/ardupilot_gazebo/build
ENV GZ_SIM_RESOURCE_PATH=/home/ubuntu/ardupilot_gazebo/models:/home/ubuntu/ardupilot_gazebo/worlds:/ws/src/kestrel/src/kestrel/models:/ws/src/kestrel/src/kestrel/worlds
WORKDIR /ws

# Source ROS and the workspace overlay before running the container command
COPY docker/entrypoint.sh /entrypoint.sh
RUN sudo chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
