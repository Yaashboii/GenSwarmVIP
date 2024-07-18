FROM ros:noetic-ros-core-focal

# Install Python dependencies
RUN apt-get update && \
    apt-get install -y \
    python3-pip \
    python3-rosdep \
    python3-rospkg \
    wget \
    git \
    && rm -rf /var/lib/apt/lists/*

# RUN pip install

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc


# Set the default command when the container starts
CMD ["/bin/bash"]
