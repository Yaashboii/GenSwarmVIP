FROM huabench/code-llm:base

ARG PYTHON_VERSION=3.10
ENV DEBIAN_FRONTEND=noninteractive

# Switch the working directory to /src
WORKDIR /catkin_ws

# Create a conda environment with specified Python version
RUN conda create -y --name py$(echo $PYTHON_VERSION | sed 's/\.//g') python=${PYTHON_VERSION}

# Activate the conda environment
SHELL ["/bin/bash", "-c"]
RUN echo "conda activate py$(echo $PYTHON_VERSION | sed 's/\.//g')" >> ~/.bashrc

# Install Python packages from requirements.txt
COPY requirements.txt requirements.txt

RUN apt-get update && apt-get install -y \
    swig \
    python3-empy \
    libgl1-mesa-glx \
    iputils-ping \
    apt-transport-https \
    ca-certificates \
    curl \
    software-properties-common \
    lsb-release \
    openssh-client \
    ansible \
    sshpass \
    && rm -rf /var/lib/apt/lists/*

# Install Docker (optional if you want Docker inside the container)
RUN curl -fsSL https://get.docker.com | bash

# Install Docker Compose (latest stable version)
RUN curl -L "https://github.com/docker/compose/releases/download/1.29.2/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose \
    && chmod +x /usr/local/bin/docker-compose

# Verify Docker and Docker Compose installation
RUN docker --version && docker-compose --version

RUN /bin/bash -c "source activate py$(echo $PYTHON_VERSION | sed 's/\.//g') \
                  && pip3 install --no-cache-dir -r requirements.txt"

RUN printf '[defaults]\nhost_key_checking = False\n' > /etc/ansible/ansible.cfg

# Set the default command when the container starts
CMD ["/bin/bash"]
