// This is the main program from which you setup/run the simulation
#include <iostream>
#include <simulator.h>
#include <easy3d/viewer/viewer.h>
#include <easy3d/renderer/camera.h>
#include <easy3d/core/types.h>
#include <easy3d/util/resource.h>
#include <easy3d/util/initializer.h>



int main() 
{
    // Simulation parameters
    int steps = 10;  
    double dt = 0.00001;
    int vehicle_count = 1;
    std::string fname = "../data.txt";

    //Instantiate a UAV
    Aircraft drone(fname,vehicle_count);
     
    drone.steps = steps;
    drone.dt = dt;

    // Keyboard control
    drone.initKeyboard();
    

    // initialize Easy3D.
    easy3d::initialize(true);
    
    // Create the default Easy3D viewer. Note: a viewer must be created before creating any drawables.
    easy3d::Viewer viewer("UAV Simulator");

    // Load a 3D model (OBJ, PLY, STL, etc.)
    drone.file_name = "/home/fahim/Downloads/Git/UAV-Simulator/scaled_uav6.stl";  // Replace with your actual model path
    drone.mesh = easy3d::SurfaceMeshIO::load(drone.file_name);

    // Draw the aircraft and 3D graphs
   
    drone.renderAircraft(viewer);
    drone.createGridDrawable(viewer);

    // Make sure everything is within the visible region of the viewer.
    viewer.fit_screen();
    viewer.set_animation(true); // -> True here will create a log file in /build
    
    // Animation function
    viewer.animation_func_ = [&](easy3d::Viewer* v) -> bool 
    {
        return drone.animate(v,dt);
    };

    

      
    return viewer.run();
    
}

