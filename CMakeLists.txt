cmake_minimum_required(VERSION 3.0)
project(UAVSim)
set(CMAKE_CXX_STANDARD 17)

# === Easy3D ===
set(Easy3D_DIR "/home/fahim/Libraries/Easy3D/Release")
find_package(Easy3D COMPONENTS viewer REQUIRED)

# === Python & NumPy ===
find_package(Python3 REQUIRED COMPONENTS Development)

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
)

# === Matplot++ ===
add_subdirectory(matplotplusplus)


# === Executable ===
add_executable(UAVSim
    simulator.cpp
    aircraft.cpp
    world.cpp
    plotter.cpp
    simviewer.cpp
    waypoint_list.cpp
    line_segment.cpp
    path.cpp
    mission_manager.cpp
    gnc.cpp
    viewer_imgui.cpp
    ${IMGUI_SOURCES}
)

# === Linking ===
target_link_libraries(UAVSim
    PUBLIC
    matplot
    easy3d::viewer
    Python3::Python
    glfw      # Needed for ImGui backend
    GL        # OpenGL
    pthread   # Threading
    dl        # Dynamic linking
)

# === Compiler flags ===
target_compile_options(UAVSim PRIVATE -std=c++17 -Wattributes)
