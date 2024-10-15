cmake_minimum_required(VERSION 3.0)
project(3D_TEST)
set(CMAKE_CXX_STANDARD 11)

# To help it find the EAsy3D package
set(Easy3D_DIR "/home/fahim/Downloads/Easy3D/Release")

# Find the required packages and libraries
find_package(Easy3D COMPONENTS viewer REQUIRED)


# Include directories for the additional headers
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}) # This will include the current source directory





# Add your CPP program source file
add_executable(UAVSim simulator.cpp aircraft.cpp)

# Link libraries to your target
target_link_libraries(UAVSim easy3d::viewer)

#For keyboard ncurses
# Add this line in your CMakeLists.txt
target_link_libraries(UAVSim ncurses)


find_package(Python3 REQUIRED COMPONENTS Development)
target_link_libraries(UAVSim Python3::Python)

# Additional include directories and compile options
target_include_directories(UAVSim PRIVATE /usr/include/python3.6) # Replace this with the path to the Python include directory
target_compile_options(UAVSim PRIVATE -std=c++17) # Set C++ standard to C++17
# Add the path to the Eigen include directory
target_include_directories(UAVSim PRIVATE /usr/local/include/Eigen)
# If matplotlibcpp requires the Python include directory
target_include_directories(UAVSim PRIVATE /usr/include/python3.6)

# Add the path to the directory containing "matplotlibcpp.h"
target_include_directories(UAVSim PRIVATE /home/fahim/Downloads/Git/UAV-Simulator)