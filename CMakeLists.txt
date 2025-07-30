cmake_minimum_required(VERSION 3.0)
project(UAVSim)
set(CMAKE_CXX_STANDARD 17)

# === Easy3D ===
set(Easy3D_DIR "/home/fahim/Libraries/Easy3D/Release")
find_package(Easy3D COMPONENTS viewer REQUIRED)

# === Python & NumPy ===
find_package(Python3 REQUIRED COMPONENTS Development)

# === NLOPT ======
find_package(NLopt REQUIRED)


# === ImGui ===
set(IMGUI_DIR /home/fahim/Libraries/Easy3D/3rd_party/imgui)
set(IMGUI_BACKENDS_DIR ${IMGUI_DIR}/backends)
set(IMPLOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/implot")

set(IMGUI_SOURCES
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_BACKENDS_DIR}/imgui_impl_glfw.cpp
    ${IMGUI_BACKENDS_DIR}/imgui_impl_opengl3.cpp
    ${IMPLOT_DIR}/implot.cpp
    ${IMPLOT_DIR}/implot_items.cpp
)


# === NLOPT ======
find_library(NLOPT_LIB nlopt)
if(NLOPT_LIB)
    message(STATUS "Found NLopt: ${NLOPT_LIB}")
else()
    message(FATAL_ERROR "NLopt library not found")
endif()

# Add this anywhere before `add_executable`
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "/usr/local/lib")



include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${IMGUI_DIR}
    ${IMGUI_BACKENDS_DIR}
    ${IMPLOT_DIR}
    ${Python3_INCLUDE_DIRS}
    /usr/include/eigen3
    /usr/include/python3.6
    /home/fahim/.local/lib/python3.12/site-packages/numpy/_core/include
    /home/fahim/Coding/Git/UAV-Simulator
    /home/fahim/Coding/Git/UAV-Simulator/include
    /usr/include
)





# === Executable ===
add_executable(UAVSim
    src/main.cpp
    src/aircraft/aircraft.cpp
    src/rendering/world.cpp
    src/rendering/simviewer.cpp
    src/gnc/waypoint_list.cpp
    src/gnc/line_segment.cpp
    src/gnc/path.cpp
    src/gnc/mission_manager.cpp
    src/gnc/gnc.cpp
    src/rendering/viewer_imgui.cpp
    ${IMGUI_SOURCES}
)

# === Linking ===
target_link_libraries(UAVSim
    PUBLIC
    ${NLOPT_LIB}
    easy3d::viewer
    Python3::Python
    glfw      # Needed for ImGui backend
    GL        # OpenGL
    pthread   # Threading
    dl        # Dynamic linking
    nlopt     # For NLopt library linkin
)

# === Compiler flags ===
target_compile_options(UAVSim PRIVATE -std=c++17 -Wattributes -Wunused-variable)
