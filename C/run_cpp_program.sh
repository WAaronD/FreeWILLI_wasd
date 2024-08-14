#!/bin/bash

# Example to run program ./run_cpp_program.sh self 1045 2500 120 0
if [ "$#" -ne 5 ]; then
  echo "Usage: $0 <UDP_IP> <UDP_PORT> <DETECTION_THRESHOLD> <RUN_TIME_SEC> <SLEEP_TIME_SEC>"
  echo "Example: ./run_cpp_program.sh self 1045 2500 120 0"
  exit 1
fi

UDP_IP=$1
UDP_PORT=$2
DETECTION_THRESHOLD=$3
RUN_TIME_SEC=$4
SLEEP_TIME_SEC=$5
FIRMWARE_VERSION=1240
CHECK_TIME=$((RUN_TIME_SEC + 2))

# Program executable
PROGRAM="./bin/HarpListen"

DIR="deployment_files"

# Check if the directory exists
if [ -d "$DIR" ]; then
  # If it exists, clear its contents
  rm -rf "${DIR:?}"/*
else
  # If it does not exist, create the directory
  mkdir "$DIR"
fi

# Define the error log file as a variable
ERROR_LOG="$DIR/error_log.txt"

# Function to handle cleanup upon receiving SIGINT
cleanup() {
  echo "Caught SIGINT signal! Terminating..."
  if [ -n "$PID" ] && kill -0 $PID ; then
    kill $PID
  fi
  exit 0
}

# Trap SIGINT signal (Ctrl-C)
trap cleanup SIGINT

while true; do
  # Run the compiled C++ program in the background
  #"$PROGRAM" "$UDP_IP" $UDP_PORT $FIRMWARE_VERSION $DETECTION_THRESHOLD $RUN_TIME_SEC > /dev/null 2>> "$ERROR_LOG" &
  "$PROGRAM" "$UDP_IP" $UDP_PORT $FIRMWARE_VERSION $DETECTION_THRESHOLD $RUN_TIME_SEC > /dev/null 2>> "$ERROR_LOG" &
  
  # Get the PID of the last background command
  PID=$!

  sleep 1 # allow program to initialize variables and begin execution before checking it 
  
  if kill -0 $PID ; then
    echo "Program Running..."
  else
    echo "Program failed to start..."
  fi

  # Allow the program to run for the specified time in seconds
  sleep $CHECK_TIME
  
  # Check if the program is still running and kill it if necessary
  if kill -0 $PID 2>/dev/null; then
    echo "Program did not exit within the specified time, forcefully terminating..."
    kill -SIGKILL $PID
  fi

  # Wait for the program to terminate and get its exit status
  wait $PID
  EXIT_STATUS=$?

  if [ $EXIT_STATUS -eq 0 ]; then
    echo "Program exited successfully with status 0."
  else
    echo "Program exited with status $EXIT_STATUS."
  fi  

  # Wait for the specified sleep time in seconds before restarting
  sleep $SLEEP_TIME_SEC
done
