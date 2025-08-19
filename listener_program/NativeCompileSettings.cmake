# ===========================
# Detect System Information
# ===========================
if(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    set(MACOSX TRUE)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    file(READ "/etc/os-release" OS_RELEASE_CONTENTS)
    if(OS_RELEASE_CONTENTS MATCHES "Ubuntu")
        set(UBUNTU TRUE)
    elseif(OS_RELEASE_CONTENTS MATCHES "Raspbian|Debian")
        set(RASPBIAN_DEBIAN TRUE)
        
        # Check for Raspberry Pi models
        file(READ "/proc/cpuinfo" CPUINFO_CONTENTS)
        if (CPUINFO_CONTENTS MATCHES "Raspberry Pi 4")
            set(RASPBERRY_PI_4 TRUE)
        elseif (CPUINFO_CONTENTS MATCHES "Raspberry Pi Zero")
            set(RASPBERRY_PI_ZERO TRUE)
        endif()
    endif()
else()
    message(FATAL_ERROR "Unsupported operating system.")
endif()

# ===========================
# Find FFTW3f (Single Precision)
# ===========================
find_library(FFTWF_LIBRARIES NAMES fftw3f REQUIRED)
find_path(FFTWF_INCLUDE_DIRS NAMES fftw3.h PATH_SUFFIXES include REQUIRED)

if(NOT FFTWF_LIBRARIES OR NOT FFTWF_INCLUDE_DIRS)
    message(WARNING "FFTW3f (single precision) library not found! Skipping.")
else()
    message(STATUS "FFTW3f Library: ${FFTWF_LIBRARIES}")
    message(STATUS "FFTW3f Include Directory: ${FFTWF_INCLUDE_DIRS}")

    list(APPEND THIRD_PARTY_LIBRARIES ${FFTWF_LIBRARIES})
    list(APPEND THIRD_PARTY_INCLUDE_DIRS ${FFTWF_INCLUDE_DIRS})
endif()

# ===========================
# Find ONNX Runtime
# ===========================
if(MACOSX)
    set(ONNXRUNTIME_INCLUDE_DIR "/usr/local/include/onnxruntime")
    set(ONNXRUNTIME_LIBRARY_DIR "/usr/local/lib")
elseif(UBUNTU OR RASPBIAN_DEBIAN)
    set(ONNXRUNTIME_INCLUDE_DIR "/usr/include/onnxruntime")
    set(ONNXRUNTIME_LIBRARY_DIR "/usr/lib")
endif()

find_library(ONNXRUNTIME_LIBRARIES NAMES onnxruntime HINTS ${ONNXRUNTIME_LIBRARY_DIR} REQUIRED)
find_path(ONNXRUNTIME_INCLUDE_DIRS NAMES onnxruntime_cxx_api.h HINTS ${ONNXRUNTIME_INCLUDE_DIR} REQUIRED)

if(NOT ONNXRUNTIME_LIBRARIES OR NOT ONNXRUNTIME_INCLUDE_DIRS)
    message(FATAL_ERROR "ONNX Runtime not found! Please check your installation paths.")
endif()

message(STATUS "ONNX Runtime Library: ${ONNXRUNTIME_LIBRARIES}")
message(STATUS "ONNX Runtime Include Directory: ${ONNXRUNTIME_INCLUDE_DIRS}")

list(APPEND THIRD_PARTY_LIBRARIES ${ONNXRUNTIME_LIBRARIES})
list(APPEND THIRD_PARTY_INCLUDE_DIRS ${ONNXRUNTIME_INCLUDE_DIRS})