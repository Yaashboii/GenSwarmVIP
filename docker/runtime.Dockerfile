FROM huabench/code-llm:base

ARG PYTHON_VERSION=3.10

# Switch the working directory to /src
WORKDIR /src

# Create a conda environment with specified Python version
RUN conda create -y --name py$(echo $PYTHON_VERSION | sed 's/\.//g') python=${PYTHON_VERSION}

# Activate the conda environment
SHELL ["/bin/bash", "-c"]
RUN echo "conda activate py$(echo $PYTHON_VERSION | sed 's/\.//g')" >> ~/.bashrc

# Install Python packages from requirements.txt
COPY requirements.txt requirements.txt
RUN /bin/bash -c "source activate py$(echo $PYTHON_VERSION | sed 's/\.//g') \
                  && pip3 install --no-cache-dir -r requirements.txt \
                  && pip3 install rospkg"

# Set the default command when the container starts
CMD ["/bin/bash"]
