# cmake -DCMAKE_C_COMPILER=/usr/bin/gcc-12 -DCMAKE_CXX_COMPILER=/usr/bin/g++-12 ..
# cmake -DCMAKE_BUILD_TYPE=Debug ..

#cmake -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake -DRASPBERRY_PI_ZERO=TRUE ..

cmake_minimum_required(VERSION 3.29.7)
project(HarpListen VERSION 1.0)

# Check if we are cross-compiling
if(CMAKE_CROSSCOMPILING)
    message(STATUS "Cross-compiling for ARM (Raspberry Pi Zero 2W)")

    # Set Raspberry Pi 64-bit specific flags
    add_compile_options(-mcpu=cortex-a53+crypto -march=armv8-a -ftree-vectorize -O3 -ffast-math)

    set(ENABLE_TEST FALSE)  # Disable tests for cross-compiling
    set(ENABLE_BENCHMARK FALSE)

    # Ensure CMake searches the correct paths within the sysroot
    set(CMAKE_PREFIX_PATH ${CMAKE_SYSROOT}/usr ${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu ${CMAKE_SYSROOT}/usr/include)
    set(CMAKE_LIBRARY_PATH ${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu ${CMAKE_SYSROOT}/lib/aarch64-linux-gnu)
else()
    # If not cross-compiling, set the compilers if needed
    if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
        set(CMAKE_C_COMPILER /usr/bin/gcc-12)
        set(CMAKE_CXX_COMPILER /usr/bin/g++-12)
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
        set(CMAKE_C_COMPILER /usr/bin/gcc)
        set(CMAKE_CXX_COMPILER /usr/bin/g++)
    endif()
endif()

# Set the C++ standard
set(CMAKE_CXX_STANDARD 20)

# Add a check for empty CMAKE_SYSTEM_NAME
if (NOT CMAKE_SYSTEM_NAME)
    message(FATAL_ERROR "CMAKE_SYSTEM_NAME is not set. Please check your CMake configuration.")
endif()
message(STATUS "CMAKE_SYSTEM_NAME: ${CMAKE_SYSTEM_NAME}")

# GLOBAL VARIABLES FOR SETTINGS
set(ENABLE_TEST FALSE)
set(ENABLE_AUTO_TEST FALSE)
set(ENABLE_BENCHMARK FALSE)

# Check for a build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Initialize third-party libraries and include directories
set(THIRD_PARTY_LIBRARIES "")
set(THIRD_PARTY_INCLUDE_DIRS "")

# Include and link directories
include_directories(${THIRD_PARTY_INCLUDE_DIRS})
link_directories(${CMAKE_LIBRARY_PATH})

# Find and append third-party libraries
if(CMAKE_CROSSCOMPILING)
    # Find OpenBLAS
    find_library(OPENBLAS_LIBRARIES 
    NAMES openblas openblasp-r0.3.21
    PATHS ${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/ 
        ${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/openblas-pthread/
    NO_DEFAULT_PATH)

    if (OPENBLAS_LIBRARIES)
    list(APPEND THIRD_PARTY_LIBRARIES ${OPENBLAS_LIBRARIES})
    message(STATUS "OpenBLAS libraries (cross-compiling): ${OPENBLAS_LIBRARIES}")
    else()
    message(FATAL_ERROR "OpenBLAS libraries not found!")
    endif()
    
    # Find FFTW3
    find_library(FFTW_LIBRARIES NAMES fftw3 REQUIRED PATHS ${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/ NO_DEFAULT_PATH)
    find_path(FFTW_INCLUDE_DIRS NAMES fftw3.h PATHS ${CMAKE_SYSROOT}/usr/include NO_DEFAULT_PATH)
    if (FFTW_LIBRARIES)
        list(APPEND THIRD_PARTY_LIBRARIES ${FFTW_LIBRARIES})
        message(STATUS "FFTW libraries (cross-compiling): ${FFTW_LIBRARIES}")
    endif ()
    if (FFTW_INCLUDE_DIRS)
        list(APPEND THIRD_PARTY_INCLUDE_DIRS ${FFTW_INCLUDE_DIRS})
        message(STATUS "FFTW include dirs (cross-compiling): ${FFTW_INCLUDE_DIRS}")
    endif()
    
    # Find ONNX Runtime library
    find_library(ONNXRUNTIME_LIBRARIES 
                NAMES onnxruntime 
                REQUIRED 
                PATHS ${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/ 
                    ${CMAKE_SYSROOT}/usr/local/lib 
                NO_DEFAULT_PATH)

    # Find ONNX Runtime include directory
    find_path(ONNXRUNTIME_INCLUDE_DIRS 
            NAMES onnxruntime_cxx_api.h 
            PATHS ${CMAKE_SYSROOT}/usr/include 
                    ${CMAKE_SYSROOT}/usr/local/include 
            NO_DEFAULT_PATH)

    # Add to THIRD_PARTY_LIBRARIES and THIRD_PARTY_INCLUDE_DIRS
    if (ONNXRUNTIME_LIBRARIES AND ONNXRUNTIME_INCLUDE_DIRS)
        list(APPEND THIRD_PARTY_LIBRARIES ${ONNXRUNTIME_LIBRARIES})
        list(APPEND THIRD_PARTY_INCLUDE_DIRS ${ONNXRUNTIME_INCLUDE_DIRS})
        
        message(STATUS "ONNX Runtime libraries (cross-compiling): ${ONNXRUNTIME_LIBRARIES}")
        message(STATUS "ONNX Runtime include dirs (cross-compiling): ${ONNXRUNTIME_INCLUDE_DIRS}")
    else()
        message(FATAL_ERROR "ONNX Runtime libraries or include directories not found!")
    endif()

    # Add ONNX Runtime include directories to the target
    #target_include_directories(MainLib PRIVATE ${ONNXRUNTIME_INCLUDE_DIRS})
    #target_link_libraries(MainLib PRIVATE ${ONNXRUNTIME_LIBRARIES})

    # Find FFTW3f for single precision
    find_library(FFTWF_LIBRARIES NAMES fftw3f REQUIRED PATHS ${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/ NO_DEFAULT_PATH)
    if (FFTWF_LIBRARIES)
        list(APPEND THIRD_PARTY_LIBRARIES ${FFTWF_LIBRARIES})
        message(STATUS "FFTWf libraries (cross-compiling): ${FFTWF_LIBRARIES}")
    endif ()
    if (FFTWF_INCLUDE_DIRS)
        list(APPEND THIRD_PARTY_INCLUDE_DIRS ${FFTWF_INCLUDE_DIRS})
        message(STATUS "FFTWf include dirs (cross-compiling): ${FFTWF_INCLUDE_DIRS}")
    endif()
endif()

# Eigen3
find_package (Eigen3 REQUIRED NO_MODULE)
if (EIGEN3_LIBRARIES)
    list(APPEND THIRD_PARTY_LIBRARIES ${EIGEN3_LIBRARIES})
    message(STATUS "Eigen3 libraries: ${EIGEN3_LIBRARIES}")
endif ()
if (EIGEN3_INCLUDE_DIR)
    list(APPEND THIRD_PARTY_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
    message(STATUS "Eigen3 include dir: ${EIGEN3_INCLUDE_DIR}")
endif()

# Set compiler flags based on OS
# Check for OS
if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set(MACOSX TRUE)
elseif (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    execute_process(
        COMMAND lsb_release -is
        OUTPUT_VARIABLE LSB_RELEASE_ID
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    message(STATUS "LSB_RELEASE_ID: ${LSB_RELEASE_ID}")
    if (LSB_RELEASE_ID STREQUAL "Ubuntu")
        set(UBUNTU TRUE)
    elseif (LSB_RELEASE_ID STREQUAL "Raspbian" OR LSB_RELEASE_ID STREQUAL "Debian")
        set(RASPBIAN_DEBIAN TRUE)

        # Check for specific Raspberry Pi models
        file(READ "/proc/cpuinfo" CPUINFO_CONTENTS)
        if (CPUINFO_CONTENTS MATCHES "Raspberry Pi 4")
            set(RASPBERRY_PI_4 TRUE)
        elseif (CPUINFO_CONTENTS MATCHES "Raspberry Pi Zero")
            set(RASPBERRY_PI_ZERO TRUE)
        endif()
    endif()
else()
    message(FATAL_ERROR "Unsupported operating system or Raspberry Pi model.")
endif()

if(MACOSX)
    add_compile_options(-mcpu=apple-m1)
    set(ENABLE_TEST TRUE)
    set(ENABLE_BENCHMARK TRUE)
elseif(UBUNTU)
    #add_compile_options(-march=native)
    set(ENABLE_TEST TRUE)
    set(ENABLE_BENCHMARK TRUE)
    if(NOT CMAKE_CROSSCOMPILING)
        set(ENABLE_AUTO_TEST TRUE)
    endif()
elseif(RASPBERRY_PI_4)
    #add_compile_options(-march=native)
    set(ENABLE_TEST TRUE)
    set(ENABLE_BENCHMARK TRUE)
elseif(RASPBERRY_PI_ZERO)
    #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mcpu=cortex-a53 -O3 -flto=auto")
    #add_compile_options(-march=native)
    if(TEST_ON_RASPBERRY_PI_ZERO)
        set(ENABLE_TEST TRUE)
    endif ()
    set(ENABLE_BENCHMARK TRUE)
else()
    add_compile_options(-march=native)
    set(ENABLE_TEST TRUE)
    set(ENABLE_BENCHMARK TRUE)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    if (MACOSX)
        add_compile_options(-O0 -g -Wall -fno-inline)
        add_link_options(-fsanitize=address)
    else ()
        #add_compile_options(-O0 -g -Wall -fno-inline -fsanitize=address)
        #add_link_options(-fsanitize=address)
        add_compile_options(-O0 -g -Wall -fno-inline)
        #add_link_options(-fsanitize=address)
    endif ()
else ()
    # Enable Optimization flags
    add_compile_options(-O3 -flto=auto)
    # Vectorization
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm|aarch64")
            message(STATUS "ARM processor does not support AVX vectorization.")
            if(MACOSX)
                message(STATUS "Apple M-series chip detected")
#                add_compile_options(-march=arm64)
                # add_compile_options(--fast-math) # may mess up the results
            else ()
                message(STATUS "Non-M-series ARM chip detected: enabling neon vectorization")
                #add_compile_options(-mfpu=neon)
            endif ()

        else()
            message(STATUS "x86 chip detected: enabling AVX vectorization.")
            add_compile_options(-mavx -mavx2 -mfma -O3)
        endif()
    elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
        add_compile_options(/arch:AVX2 /O2)
    endif()
endif ()

# Add the source directory
add_subdirectory(${PROJECT_SOURCE_DIR}/src)
# Add the test and benchmark directories
if (ENABLE_TEST)
    # Set the build options to skip Google Mock
    set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
    set(INSTALL_GMOCK OFF CACHE BOOL "" FORCE)
    set(BUILD_GTEST ON CACHE BOOL "" FORCE)
    set(INSTALL_GTEST ON CACHE BOOL "" FORCE)
    # current googletest version is on release-1.10.0
    add_subdirectory(${PROJECT_SOURCE_DIR}/libs/googletest)
    add_subdirectory(${PROJECT_SOURCE_DIR}/tests)
endif()
#set(ENABLE_BENCHMARK FALSE)
if(ENABLE_BENCHMARK)
    set(BENCHMARK_ENABLE_TESTING off)
    # current benchmark version is on v1.8.5
    add_subdirectory(${PROJECT_SOURCE_DIR}/libs/benchmark)
    add_subdirectory(${PROJECT_SOURCE_DIR}/benchmark)
endif()


# Set the output directory for the executable based on mode
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    message(STATUS "Building tests in Debug mode.")

    # the following line sets a variable `DEBUG` that
    # can be accessed in all cpp files in the project
    target_compile_definitions(HarpListen PRIVATE DEBUG)
    target_compile_definitions(UnitTests PRIVATE DEBUG)
    target_compile_definitions(Benchmark PRIVATE DEBUG)

    # Set the output directory for the executable
    set_property(TARGET HarpListen PROPERTY OUTPUT_NAME HarpListenDebug)
    set_target_properties(HarpListen PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/debug
    )
    if (ENABLE_TEST)
        set_property(TARGET UnitTests PROPERTY OUTPUT_NAME UnitTestsDebug)
        set_target_properties(UnitTests PROPERTIES
                RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/debug
        )
    endif ()
    if (ENABLE_BENCHMARK)
        set_property(TARGET Benchmark PROPERTY OUTPUT_NAME BenchmarkDebug)
        set_target_properties(Benchmark PROPERTIES
                RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/debug
        )
    endif ()
#    add_compile_options(-Og -g -Wall -Wextra -fsanitize=address)
else ()
    message(STATUS "Building tests in Release mode.")
    # Set the output directory for the executable
    set_target_properties(HarpListen PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin
    )
endif ()

# Print the compile flags
get_target_property(target_options HarpListen COMPILE_OPTIONS)
message(STATUS "Compile options for HarpListen: ${target_options}")

if (ENABLE_TEST)
    get_target_property(target_options UnitTests COMPILE_OPTIONS)
    message(STATUS "Compile options for UnitTests: ${target_options}")
endif ()

if(ENABLE_BENCHMARK)
    get_target_property(target_options Benchmark COMPILE_OPTIONS)
    message(STATUS "Compile options for Benchmark: ${target_options}")
endif ()

