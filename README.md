<div style="display: flex; align-items: flex-start;">
  <div style="flex: 2;">
    
## Table of Contents

1. [Overview](#overview)  
2. [Repository Structure](#repository-structure)  
3. [listener_program](#listener_program)  
   - [Installing Dependencies](#installing-dependencies)  
     - [Installing Dependencies on Ubuntu/Debian](#installing-dependencies-on-ubuntudebian)  
     - [Installing Dependencies on macOS](#installing-dependencies-on-macos)  
   - [Build Program (Ubuntu/Debian & macOS)](#build-program-ubuntudebian--macos)  
4. [simulator_program](#simulator_program)

  </div>
  <div style="flex: 1; text-align: center; padding-left: 20px;">
    <img src="supplemental/images/freeWILLIlogo.png" alt="Free WILLI Logo" width="80%">
  </div>
</div>
# Overview
### FreeWILLI: Free software for Whale Identification and Localization with Low-power Implementation:

The FreeWILLI project aims to deliver high-performance C++ software for soft real-time, low-power underwater passive acoustic array data processing. It supports automated multi-target tracking from small volumetric four-channel hydrophone arrays and offers integrated functionality for neural network inference. Designed for efficiency and flexibility, the system enables advanced acoustic analyses on resource-constrained devices.

This repo consists of two programs:

1. **listener_program**: A multi-target tracking and ML application for receiving and processing acoustic data.

2. **simulator_program**: A utility for simulating a four-channel data logger by sending pre-recorded data as UDP packets for testing and development purposes

## Repository Structure
The repository is organized as follows:

- .github/workflows/: Contains GitHub Actions workflows for continuous integration and deployment. 
- .vscode/: Configuration files for Visual Studio Code, including workspace settings and extensions.
- analysis/: Scripts and tools for analyzing and visualizing data outputs from HarpListen.
- listener_program/: Source code and resources for the HarpListen program.
- simulator_program/: Source code and resources for the DataLogger Simulator program. 
- supplemental/: Additional files, documentation, or supporting scripts.
- .gitignore: Specifies files and directories to be ignored by Git.
- .gitmodules: Configuration for managing Git submodules.
- CONTRIBUTING.md: Guidelines for contributing to the project.

## listener_program

### Dependencies

- **CMake**: A build system generator used to configure and build the project across multiple platforms.
- **FFTW3**: Required for performing fast Fourier transforms.
- **Eigen**: Used for advanced linear algebra operations, such as matrix manipulations and decompositions.
- **nlohmann-json**: A JSON library for parsing and managing configuration files.
- **ONNX Runtime**: Used for running machine learning models in the program.

### Installing Dependencies on Ubuntu/Debian
1. Example Installing CMake 3.29.7 on Linux x86
Use wget to download the precompiled binary from the official CMake website:
```bash
wget https://github.com/Kitware/CMake/releases/download/v3.29.7/cmake-3.29.7-linux-x86_64.sh
```

Make the downloaded file executable and run it to install CMake:
```bash
chmod +x cmake-3.29.7-linux-x86_64.sh
sudo ./cmake-3.29.7-linux-x86_64.sh --skip-license --prefix=/usr/local
```

Confirm the installed version of CMake:
```bash
cmake --version
```

2. Install the required libraries using apt:
```bash
sudo apt-get update
sudo apt-get install -y build-essential libfftw3-dev libeigen3-dev nlohmann-json3-dev
```

3. Download and install ONNX Runtime
```bash
wget https://github.com/microsoft/onnxruntime/releases/download/v1.14.1/onnxruntime-linux-x64-1.14.1.tgz
tar -xzf onnxruntime-linux-x64-1.14.1.tgz
sudo cp -r onnxruntime-linux-x64-1.14.1/include/* /usr/local/include/
sudo cp -r onnxruntime-linux-x64-1.14.1/lib/* /usr/local/lib/
```

### Installing Dependencies on macOS
1. Example Installing CMake 3.29.7 on macOS
Use curl to download the precompiled binary from the official CMake website:
```bash
curl -L -o cmake-3.29.7-macos-universal.dmg https://github.com/Kitware/CMake/releases/download/v3.29.7/cmake-3.29.7-macos-universal.dmg
```

Mount the .dmg file to access the installer:
```bash
hdiutil attach cmake-3.29.7-macos-universal.dmg
```

Open the mounted volume and drag the CMake application to your preferred location (e.g., /Applications

Add CMake to PATH
```bash
sudo "/Applications/CMake.app/Contents/bin/cmake" /usr/local/bin/cmake
```

Confirm the installed version of CMake:
```bash
cmake --version
```


1. Install the required libraries using brew:
```bash
brew install fftw eigen nlohmann-json
```

2. Download and install ONNX Runtime
```bash
curl -L -o onnxruntime-osx-arm64-1.19.2.tgz https://github.com/microsoft/onnxruntime/releases/download/v1.19.2/onnxruntime-osx-arm64-1.19.2.tgz
tar -xzf onnxruntime-osx-arm64-1.19.2.tgz
sudo mkdir -p /usr/local/include/onnxruntime /usr/local/lib
sudo cp -r onnxruntime-osx-arm64-1.19.2/include/* /usr/local/include/onnxruntime/
sudo cp -r onnxruntime-osx-arm64-1.19.2/lib/* /usr/local/lib/
```

### Build program (Ubuntu/Debian & macOS)

1. Clone the repository:

``` bash
git clone https://github.com/JosephLWalker96/Embedded_miniHarp.git
cd Embedded_miniHarp
```

2. Initialize and Update Submodules:
Run the following commands from the root of the repository:

```bash
git submodule init
git submodule update --init --recursive
```

3. Build the program:
```bash
cd listener_program/
mkdir out/ && cd out
cmake ..
make -j$(nproc)
```

4. Run the binary:
```bash
cd ..
./bin/HarpListen config_files/volumetric.json 50000
```
# Datalogger Simulator

A Python-based simulator (datalogger_simulator.py) that streams simulated data packets over UDP. The simulator mimics the behavior of firmware-based data logging systems. It can read .npy files, process their contents (e.g., apply channel offsets for TDOA testing, scale the data, etc.), and send out structured UDP packets for testing HarpListener program.

## File Structure
```bash
.
├── datalogger_simulator.py   # The main simulation script
├── firmware_config           # Firmware-specific configuration files
│   ├── firmware_1240.py
│   └── firmware_1550.py
├── simulator_data            # Directory containing .npy data files
├── utils.py                  # Shared utility functions (e.g., data manipulation, concurrency helpers)
└── requirements.txt          # Python dependencies needed to run the simulator
```

- **datalogger_simulator.py**
The main entry point for the simulator. It handles:
1. Command-line argument parsing (to customize data streaming IP, port, etc.).
2. Loading data from .npy files.
3. (Optionally) applying channel duplication, shifting, stretching, glitches, and more.
4. Sending the processed data over UDP.

- **firmware_config/**
1. Contains firmware-specific configurations for different firmware versions (e.g., firmware_1240.py, firmware_1550.py).
2. These Python files define constants used by the simulator (packet size, sample rate, etc.), so the simulator can mimic each firmware version’s data format.

- **simulator_data/**
Holds .npy files to be streamed by the simulator. You can store multiple .npy data files here to simulate longer runs or multiple scenarios.

- **utils.py**
Contains helper functions (e.g., reading binary data, applying channel shifts, data scaling, sleeping, etc.) that are shared by the simulator.

- **requirements.txt**
Defines the Python dependencies needed to run this simulator (e.g., NumPy, psutil, argparse, etc.).

## Program Description
The simulator is designed to emulate a real-time data capture and logging system:

1. **Firmware Emulation:**
Selects firmware configuration (e.g., firmware_1240.py) based on user arguments to determine packet structure and timing intervals.

2. **Data Streaming:**:
Reads .npy data files (which contain raw sample data).
Preprocesses the data (e.g., interleaving, scaling, time-stamping).
Sends packets over UDP to a specified IP and port.

3. **Optional Features:**

- **Looping**: Repeatedly stream the same data files in a loop.
- **TDOA Simulation**: Applies channel offsets to simulate time-difference-of-arrival.
- **Glitches**: Intentionally modifies timestamps or data packets to test receiver robustness.
- **IMU Data**: Appends IMU packets if enabled, to simulate additional sensor data.

## How to Run
1. **Install Dependencies**
From the same directory, install required dependencies:
```bash
pip install -r requirements.txt
```

2. **Prepare Data Files**

Place your .npy data files in the simulator_data/ directory (or wherever you prefer).
If using IMU functionality, ensure you have the correct IMU data files referenced in the script (adjust path in the arguments or within utils.py if needed).

3. **Run the Simulator**
Execute datalogger_simulator.py with your desired arguments.
This example will load data from simulator_data/track132_5minchunks/ and organize the data according to the firmware version 1240. The data is sent to to port 1045 at IP address 192.168.100.235 

```bash
python datalogger_simulator.py \
    --ip 192.168.100.235 \
    --port 1045 \
    --data_dir simulator_data/track132_5minchunks/ \
    --fw 1240 \
```
## Notes
**High Priority**: The simulator can set high priority for the process (using SetHighPriority in utils.py) if your system supports it.
**Multiprocessing**: Uses multiple processes to preload the next .npy file while the current one is streaming. This helps achieve smoother, more real-time data streaming.

# simulator_program

### Installation and build for DataLogger Simulator
1. Install dependencies:
```bash
cd simulator_program/
conda create --name freewilli python=3.9
conda activate freewilli
pip install -r requirements.txt
```

2. Download data:

By default, the program reads in data from folder **simulator_data/track132_5minchunks/**

[Download](https://drive.google.com/drive/folders/1v8sgYyQATcsUkzAI6AcUaiMpq5Wi37y1?usp=sharing) the track132_5minchunks/ folder and place it in the simulator_data/ directory.

3. Run example:
```bash
cd simulator_program/
```
Assuming you are running the simulator on the same machine as the listener program, run the following:
```bash
python datalogger_simulator.py --ip self --fw 1240 --port 1045
```

Otherwise, specify the correct IP address (--ip) and port (--port).
