# Use Python 3.10 slim as base
FROM python:3.10-slim

# Set working directory
WORKDIR /app

# Ensure workspace folder exists inside container
RUN mkdir -p /app/workspace

# Install system dependencies needed for some packages
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        build-essential \
        swig \
        python3-dev \
        libglib2.0-0 \
        libgl1-mesa-dev \
        && rm -rf /var/lib/apt/lists/*

# Copy requirements file
COPY requirements.txt .

# Upgrade pip and install Python dependencies
RUN pip install --upgrade pip
RUN pip install --no-cache-dir -r requirements.txt

# Copy the rest of the project into the container
COPY . .

# Set default command (optional)
CMD ["python3"]


