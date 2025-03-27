# Use an official Ubuntu base image with Python support
FROM ubuntu:22.04

# Set non-interactive mode for APT to avoid prompts
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies and Clang-Tidy
RUN apt-get update && apt-get install -y \
    clang \
    clang-tidy \
    clang-tools \
    python3-pip \
    wget \
    build-essential \
    libfftw3-dev \
    libeigen3-dev \
    nlohmann-json3-dev \
    && rm -rf /var/lib/apt/lists/*

# Make 'python' point to 'python3'
RUN ln -s /usr/bin/python3 /usr/bin/python

# Install CMake 3.29.7
RUN wget https://github.com/Kitware/CMake/releases/download/v3.29.7/cmake-3.29.7-linux-x86_64.sh && \
    chmod +x cmake-3.29.7-linux-x86_64.sh && \
    ./cmake-3.29.7-linux-x86_64.sh --skip-license --prefix=/usr/local && \
    rm cmake-3.29.7-linux-x86_64.sh

# Install ONNX Runtime
RUN wget https://github.com/microsoft/onnxruntime/releases/download/v1.14.1/onnxruntime-linux-x64-1.14.1.tgz && \
    tar -xzf onnxruntime-linux-x64-1.14.1.tgz && \
    cp -r onnxruntime-linux-x64-1.14.1/include/* /usr/local/include/ && \
    cp -r onnxruntime-linux-x64-1.14.1/lib/* /usr/local/lib/ && \
    rm -rf onnxruntime-linux-x64-1.14.1.tgz onnxruntime-linux-x64-1.14.1

# Manually install run-clang-tidy.py
RUN wget https://raw.githubusercontent.com/llvm/llvm-project/release/14.x/clang-tools-extra/clang-tidy/tool/run-clang-tidy.py -O /usr/local/bin/run-clang-tidy.py && \
    chmod +x /usr/local/bin/run-clang-tidy.py


# Set working directory
WORKDIR /app

# Copy relevant files
COPY simulator_program/requirements.txt .
COPY simulator_program/datalogger_simulator.py .
COPY simulator_program/utils.py .
COPY simulator_program/firmware_config/ firmware_config/
COPY integration_testing/ integration_testing/
COPY README.md .

# Install Python dependencies
RUN pip install --no-cache-dir -r requirements.txt