# UAV-Simulator Project



https://github.com/user-attachments/assets/81536b87-a34f-4bdb-8080-3e1eada20210



## Overview

This project is a fun and educational initiative aimed at simulating small scale aircraft dynamics and control. The project leverages various libraries for 3D rendering and mathematical operations to model and visualize aircraft behavior based on state variables and control inputs.

## Current Status

**Incomplete:** The project is currently in an incomplete state due to data loss. The most recent source code was unfortunately lost, but a primitive, underdeveloped version has been recovered and will be uploaded here.

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
  

## Contributing

Contributions to the project are welcome. If you'd like to help improve the project, please fork the repository and submit a pull request with your changes.





