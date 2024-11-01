FROM huabench/code-llm:base

ARG PYTHON_VERSION=3.10

# Switch the working directory to /src
WORKDIR /catkin_ws

# Create a conda environment with specified Python version
RUN conda create -y --name py$(echo $PYTHON_VERSION | sed 's/\.//g') python=${PYTHON_VERSION}

# Activate the conda environment
SHELL ["/bin/bash", "-c"]
RUN echo "conda activate py$(echo $PYTHON_VERSION | sed 's/\.//g')" >> ~/.bashrc

# Install Python packages from requirements.txt
COPY requirements.txt requirements.txt

RUN apt-get update && apt-get install -y swig python3-empy libgl1-mesa-glx

RUN /bin/bash -c "source activate py$(echo $PYTHON_VERSION | sed 's/\.//g') \
                  && pip3 install --no-cache-dir -r requirements.txt"

# Set the default command when the container starts
CMD ["/bin/bash"]
