# HarpListen

## Description

**HarpListen** is a real-time program designed to receive and process UDP packets efficiently. The program sets up a UDP listener that captures packets from a specified IP address and port, storing the received data in a shared buffer for subsequent processing. By leveraging multi-threading, HarpListen ensures that data reception and processing occur concurrently, enabling efficient real-time handling of incoming data.

### Key Components

- **UDP Listener**: Captures incoming UDP packets, stores the received data in a buffer, and prints statistics about the received packets.
- **Data Processor**: Retrieves data from the buffer, applies filters, and performs analysis to detect and estimate specific signal characteristics.
- **Session and Experiment Classes**: Manage session-specific and experiment-specific details, including configuration settings, data buffers, and synchronization mechanisms.
- **Multi-threading**: Employs separate threads for listening to UDP packets and processing data, ensuring that both tasks are handled efficiently and in real-time.

### Features

- Dynamic configuration based on specified firmware versions, with adjustments to processing parameters as needed.
- Logging of processed data to an output file for further analysis.
- Automatic error recovery by restarting the listener and processing threads to maintain continuous operation.

### Requirements

- **FFTW3**: Required for performing fast Fourier transforms (FFTs).
- **Eigen**: Used for linear algebra operations, such as matrix manipulations and decompositions.

## Example Usage

To run the program with a datalogger simulator:

```bash
./HarpListen 192.168.7.2 1045 1240 2500 2