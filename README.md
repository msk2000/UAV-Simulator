# UAV-Simulator Project

Demo footage illustrating path and waypoint generation, real-time state plotting, slider based aircraft control:


https://github.com/user-attachments/assets/8f9a8d6c-32c5-4173-bad0-4a8b659f6ae0



A texture mapped terrain and control demo footage:



https://github.com/user-attachments/assets/9578d49b-d618-4981-91b6-300246770b43



A stall demo footage (old):



https://github.com/user-attachments/assets/97ee1782-feb5-4569-809a-e0d4cac45b3d





## Overview

This repository contains a custom-built UAV simulation framework that models the dynamics and control of a small fixed‑wing aircraft.  
It integrates:

- **Nonlinear flight dynamics** derived from standard aircraft equations of motion.
- **Trim calculation routines** to establish equilibrium states for specific flight conditions.
- **Linearization methods** that generate transfer‑function and state‑space representations for control design and analysis.
- **3D rendering and visualization** of the aircraft geometry, body axes, and simulation outputs.

The project serves as a foundation for experimenting with guidance, navigation, and control (GNC) algorithms, while providing an educational platform for testing new ideas in flight dynamics.

📖 **Project Documentation (Doxygen):**  
[https://msk2000.github.io/UAV-Simulator](https://msk2000.github.io/UAV-Simulator)

---

## Current Status

**Active Development:**  
The simulator has been rebuilt after an earlier data loss, and a working baseline is already running.  
At present you can:

- Load an aircraft configuration and compute trim for given airspeed and flight‑path parameters.
- Generate preliminary state‑space and transfer‑function models around the trim condition.
- Run the dynamics in a custom C++ simulation loop with live 3D visualization.

Soon you should be able to provide a path to the UAV and watch it generate waypoints and use an autopilot to navigate itself to the destination autonomously :D

## Recent Changelog:

<div style="max-height: 100px; overflow-y: scroll; border: 1px solid #ccc; padding: 10px; margin: 10px 0;">

*26th July 2025*
- **Documentation:** Doxygen workflow set up. Documentation is now automatically generated and published online at [https://msk2000.github.io/UAV-Simulator](https://msk2000.github.io/UAV-Simulator).

*25th July 2025*
- **Trim:** Implemented automated trim point calculation and corresponding control settings and sim initialisation.
- **Memory Leaks:** Fixed some memory leak bugs.
- **Waypoint Navigation:** Work on waypoint navigation started. A sample path (in red) and waypoints(yellow) can be seen in the most recent update video.
- **ImGUI/ImPlot:** Live plotting of state variables integrated into simulation (see recent video). A control panel for elevator,rudder, aileron and throttle input has been added to the gui.

*6th July 2025*

- **Terrain Texture Mapping:** Implemented texture mapping on the STL terrain using UV coordinates and a texture image. Normalized and tiled the UVs for consistent texture scaling.
- **Texture Wrap Mode:** Set texture wrap mode to `REPEAT` to allow seamless tiling across the terrain surface.
- **Coordinate Adjustment:** Corrected UV mapping to use the X and Y axes (`pos[0]`, `pos[1]`) instead of X and Z, since Z is the up-axis in this simulator.
- **Shadow Rendering:** Attempted basic hard shadow support for the aircraft model using Easy3D's `Shadow` class. **Pending Implementation**.
- **HUD Implementation:** Integrated a Heads-Up Display (HUD) in the viewer using Easy3D's `TextRenderer`. The HUD overlays real-time flight data from the aircraft onto the screen during simulation.
- **Viewer Improvements:** Introduced a new `TestViewer` class derived from Easy3D’s `Viewer`. This class handles aircraft HUD rendering, user input for aircraft control, and shadow rendering(incomplete).

*28th October 2024*
- **Runga-Kutta 4:** Changed the simplistic solver to RK4 based integrator for much better estimation of state parameters.

*20th October 2024*
- **Modularization:** Refactored the **forces and moments** function from a monolithic structure to a more modular design for improved clarity and maintainability.
- **Dynamics Function:** Transitioned the **dynamics** function to a modular structure, facilitating easier updates and modifications.
- **Animation Cleanup:** Cleaned up the **animation** function by introducing an **update** function that manages all necessary updates to the simulation before rendering.
- **Coordinate Frame:** Added a **body frame coordinate system** that is always visible and coincident with the aircraft, providing a consistent reference for the simulation.

*16th October 2024*
- **Visualisation Method:** Changed the simple vertex/face based rendering to surface mesh based one allowing for import of any 3d model into the simulator.
- **Changes to Aircraft Class:** Removed multiple depricated functionds and added new ones to help with the new render system.

</div>


## Features

- **Aircraft Dynamics:** Models the physical characteristics and flight dynamics of an aircraft.
- **State Tracking:** Tracks various state variables including position, velocity, and orientation.
- **Control Inputs:** Handles different control inputs such as throttle, aileron, elevator, and rudder.
- **3D Rendering:** Uses Easy3D for rendering and visualizing aircraft dynamics in a 3D environment.
- **Graphing:** Includes basic functionality for plotting and analyzing aircraft state data.

## Dependencies

This project relies on the following libraries:
- [Eigen](https://eigen.tuxfamily.org/dox/GettingStarted.html): For linear algebra operations.
- [matplotlibcpp](https://github.com/lava/matplotlibcpp): A C++ wrapper for Matplotlib to plot graphs.
- [Easy3D](https://github.com/easy3d/easy3d): For 3D visualization and rendering.

## Building the Project [NOT RECOMMENDED FOR NOW]

To build and run this project, follow these steps:

1. **Clone the Repository:**
    ```bash
    git clone https://github.com/msk2000/UAV-Simulator.git
    cd aircraft-simulation
    ```

2. **Install Dependencies:**
   Ensure you have the necessary libraries installed. You may need to follow the installation instructions for [Eigen](https://eigen.tuxfamily.org/dox/GettingStarted.html), [matplotlibcpp](https://github.com/lava/matplotlibcpp), and [Easy3D](https://github.com/easy3d/easy3d).

3. **Compile the Code:**
    ```bash
    g++ -o aircraft_simulation main.cpp -I/path/to/eigen -I/path/to/matplotlibcpp -I/path/to/easy3d -lX11 -lGL -lGLU
    ```

4. **Run the Simulation:**
    ```bash
    ./aircraft_simulation
    ```

## Known Issues

- **Data Loss:** Recent updates to the project have been lost. The current code is a recovery of an older, less developed version.
- **Incomplete Features:** The project is still under development, and some features are not fully implemented or tested.

## Future Work

- **Feature Completion:** Completing and improving the simulation features.
- **Enhanced Rendering:** Adding more detailed and interactive 3D visualizations.
- **Improved Data Handling:** Recovering and integrating the latest data to enhance the simulation accuracy.
- **Incorporate QT libraries:** This is intended to replace the current dependency on matplotlib.

# Class Members for Aircraft Simulation

The following table details the key parameters used in the simulation. These parameters represent the physical properties, control inputs, and initial conditions of the aircraft. This table will help you understand both the class members and the data.txt file that the simulator uses to load a given UAV configuration.

| #  | Parameter        | Description                                                                                                       |
|----|------------------|-------------------------------------------------------------------------------------------------------------------|
| 1  | `mass`         | Aircraft mass in kilograms.                                                                                       |
| 2  | `Jx`           | Moment of inertia about the x-axis (roll) in kg·m².                                                               |
| 3  | `Jy`           | Moment of inertia about the y-axis (pitch) in kg·m².                                                              |
| 4  | `Jz`           | Moment of inertia about the z-axis (yaw) in kg·m².                                                                |
| 5  | `Jxz`          | Product of inertia for coupling between the x and z axes.                                                         |
| 6  | `S_wing`       | Wing surface area in square meters.                                                                               |
| 7  | `b`            | Wingspan in meters.                                                                                               |
| 8  | `c`            | Mean aerodynamic chord length in meters.                                                                          |
| 9  | `e`            | Oswald efficiency factor.                                                                                         |
| 10 | `g`            | Gravitational acceleration constant (9.81 m/s²).                                                                  |
| 11 | `rho`          | Air density in kg/m³.                                                                                             |
| 12 | `k_motor`      | Motor constant representing the relationship between motor speed and thrust.                                      |
| 13 | `S_prop`       | Propeller surface area in square meters.                                                                          |
| 14 | `k_T_P`        | Motor torque constant (to be defined based on propeller efficiency).                                               |
| 15 | `k_Omega`      | Propeller rotational speed constant (to be defined based on efficiency).                                           |
| 16 | `C_L_0`        | Zero-lift coefficient of lift.                                                                                     |
| 17 | `C_L_alpha`    | Lift curve slope, relating angle of attack to lift.                                                               |
| 18 | `C_L_q`        | Lift coefficient due to pitch rate.                                                                               |
| 19 | `C_L_delta_e`  | Lift coefficient due to elevator deflection.                                                                      |
| 20 | `C_D_0`        | Zero-lift drag coefficient (parasitic drag).                                                                      |
| 21 | `C_D_alpha`    | Drag coefficient as a function of angle of attack.                                                                |
| 22 | `C_D_p`        | Drag coefficient due to roll rate.                                                                                |
| 23 | `C_D_q`        | Drag coefficient due to pitch rate.                                                                               |
| 24 | `C_D_delta_e`  | Drag coefficient due to elevator deflection.                                                                      |
| 25 | `C_m_0`        | Zero-lift moment coefficient.                                                                                     |
| 26 | `C_m_alpha`    | Moment coefficient due to angle of attack.                                                                        |
| 27 | `C_m_q`        | Moment coefficient due to pitch rate.                                                                             |
| 28 | `C_m_delta_e`  | Moment coefficient due to elevator deflection.                                                                    |
| 29 | `C_Y_0`        | Zero-sideslip yaw force coefficient.                                                                              |
| 30 | `C_Y_beta`     | Yaw force coefficient due to sideslip angle (lateral stability).                                                   |
| 31 | `C_Y_p`        | Yaw force coefficient due to roll rate.                                                                           |
| 32 | `C_Y_r`        | Yaw force coefficient due to yaw rate.                                                                            |
| 33 | `C_Y_delta_a`  | Yaw force coefficient due to aileron deflection.                                                                  |
| 34 | `C_Y_delta_r`  | Yaw force coefficient due to rudder deflection.                                                                   |
| 35 | `C_ell_0`      | Roll moment coefficient at zero sidesli                                                                         |
| 36 | `C_ell_beta`   | Roll moment coefficient due to sideslip angle.                                                                    |
| 37 | `C_ell_p`      | Roll moment coefficient due to roll rate.                                                                         |
| 38 | `C_ell_r`      | Roll moment coefficient due to yaw rate.                                                                          |
| 39 | `C_ell_delta_a`| Roll moment coefficient due to aileron deflection.                                                                |
| 40 | `C_ell_delta_r`| Roll moment coefficient due to rudder deflection.                                                                 |
| 41 | `C_n_0`        | Zero-yaw moment coefficient.                                                                                      |
| 42 | `C_n_beta`     | Yaw moment coefficient due to sideslip angle (lateral stability).                                                  |
| 43 | `C_n_p`        | Yaw moment coefficient due to roll rate.                                                                          |
| 44 | `C_n_r`        | Yaw moment coefficient due to yaw rate.                                                                           |
| 45 | `C_n_delta_a`  | Yaw moment coefficient due to aileron deflection.                                                                 |
| 46 | `C_n_delta_r`  | Yaw moment coefficient due to rudder deflection.                                                                  |
| 47 | `C_prop`       | Propeller thrust coefficient.                                                                                     |
| 48 | `M`            | Transition Rate used for Stall Characteristic modelling                                                          |
| 49 | `epsilon`      | Downwash gradient coefficient.                                                                                    |
| 50 | `alpha0`       | Zero-lift angle of attack.                                                                                        |
| 51 | `pn0`          | Initial position in the North direction (in meters).                                                              |
| 52 | `pe0`          | Initial position in the East direction (in meters).                                                               |
| 53 | `pd0`          | Initial position in the Down direction (negative altitude, in meters).                                             |
| 54 | `u0`           | Initial velocity along the body x-axis (forward velocity).                                                         |
| 55 | `v0`           | Initial velocity along the body y-axis (side-slip velocity).                                                       |
| 56 | `w0`           | Initial velocity along the body z-axis (vertical velocity).                                                        |
| 57 | `phi0`         | Initial roll angle (in radians).                                                                                  |
| 58 | `theta0`       | Initial pitch angle (in radians).                                                                                 |
| 59 | `psi0`         | Initial yaw angle (in radians).                                                                                   |
| 60 | `p0`           | Initial roll rate (body frame).                                                                                   |
| 61 | `q0`           | Initial pitch rate (body frame).                                                                                  |
| 62 | `r0`           | Initial yaw rate (body frame).                                                                                    |
| 63 | `delta_t`        | Throttle input (ranging from 0 to 1).                                                                             |
| 64 | `delta_a`        | Aileron deflection (control surface input for roll control).                                                       |
| 65 | `delta_e`        | Elevator deflection (control surface input for pitch control).                                                     |
| 66 | `delta_r`        | Rudder deflection (control surface input for yaw control).                                                         |
| 67 | `delta_t_max`    | Maximum throttle setting.                                                                                         |
| 68 | `delta_t_min`    | Minimum throttle setting.                                                                                         |
| 69 | `delta_a_max`    | Maximum aileron deflection.                                                                                       |
| 70 | `delta_a_min`    | Minimum aileron deflection.                                                                                       |
| 71 | `delta_e_max`    | Maximum elevator deflection.                                                                                      |
| 72 | `delta_e_min`    | Minimum elevator deflection.                                                                                      |
| 73 | `delta_r_max`    | Maximum rudder deflection.                                                                                        |
| 74 | `delta_r_min`    | Minimum rudder deflection.                                                                                        |


### Usage

Each of these parameters is used to model the dynamics of the aircraft in simulation, ensuring that the response to control inputs and environmental factors is accurate. For more details, refer to the implementation in `simulation.h`. Additional details about how to obtain these parameters for a given UAV will be provided in the future.

  

## Contributing

Contributions to the project are welcome. If you'd like to help improve the project, please fork the repository and submit a pull request with your changes.





