# Use an official Python 3.9 image
FROM python:3.9-slim

# Set default port (can be overridden at build time)
#ARG PORT=1045

# Set the working directory in the container
WORKDIR /app

# Copy only the necessary files (excluding the dataset)
COPY requirements.txt .
COPY datalogger_simulator.py .
COPY utils.py .
COPY firmware_config/ firmware_config/
COPY README.md .

# Install dependencies
RUN pip install --no-cache-dir -r requirements.txt

# Expose the port (informational only)
#EXPOSE ${PORT}

# Default command to run the simulator
#CMD ["python", "datalogger_simulator.py", "--ip", "self", "--fw", "1240", "--port", "1045"]
