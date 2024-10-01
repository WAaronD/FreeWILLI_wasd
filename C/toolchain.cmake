# toolchain.cmake

# Specify the target system name and architecture
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# Specify the compilers for cross-compilation
set(CMAKE_C_COMPILER /usr/bin/aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER /usr/bin/aarch64-linux-gnu-g++)

# Specify the sysroot
set(CMAKE_SYSROOT /home/harp/rpi-sysroot)

# Set the sysroot and root path
set(CMAKE_FIND_ROOT_PATH ${CMAKE_SYSROOT})

# Ensure that CMake searches within the sysroot for libraries and include files
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

