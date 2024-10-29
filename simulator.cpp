// This is the main program from which you setup/run the simulation
#include <iostream>
#include <aircraft.h>
#include <world.h>
#include <plotter.h>
#include <easy3d/viewer/viewer.h>
#include <easy3d/renderer/camera.h>
#include <easy3d/core/types.h>
#include <easy3d/util/resource.h>
#include <easy3d/util/initializer.h>



int main() 
{
    // Simulation parameters
    int steps = 10;  
    double dt = 0.0001;//0.00001;
    int vehicle_count = 1;
    std::string fname = "../data.txt";

    //Instantiate a UAV
    Aircraft drone(fname,vehicle_count);
    //Initiate the world/environment
    World world;
    // Initialise a plot
    //Plotter plotter(drone);
     
    drone.steps = steps;
    drone.dt = dt;

    // Keyboard control
    drone.initKeyboard();
    

    // initialize Easy3D.
    easy3d::initialize(true);
    
    // Create the default Easy3D viewer. Note: a viewer must be created before creating any drawables.
    easy3d::Viewer viewer("UAV Simulator");
    // Flip the camera view by setting the opposite direction
    viewer.camera()->setViewDirection(easy3d::vec3(-1, 0, 0)); // Flips to the opposite side along X-axis

    // Load a 3D model (OBJ, PLY, STL, etc.)
    drone.file_name = "/home/fahim/Downloads/Git/UAV-Simulator/y_for_z_up_uav.stl";  // Replace with your actual model path
    drone.mesh = easy3d::SurfaceMeshIO::load(drone.file_name);

    // Draw the aircraft and 3D graphs and world
    world.createGround(viewer);
    drone.renderAircraft(viewer);
    drone.createGridDrawable(viewer);
    drone.createAxesDrawable(viewer);

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

