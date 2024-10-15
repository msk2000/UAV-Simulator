#include <iostream>
#include <simulator.h>
#include <easy3d/viewer/viewer.h>
#include "matplotlibcpp.h"
#include <easy3d/renderer/camera.h>
#include <easy3d/core/types.h>
#include <easy3d/util/resource.h>
#include <easy3d/util/initializer.h>



#define _USE_MATH_DEFINES

namespace plt = matplotlibcpp;



int main() 
{
    // Simulation parameters
    int steps = 10;  
    double dt = 0.00001;
    int vehicle_count = 1;
    std::string fname = "../data.txt";

    //Instantiate a UAV
    Aircraft obj(fname,vehicle_count);
    Aircraft::State state = obj.get_state(); 
    obj.steps = steps;
    obj.dt = dt;

    // Keyboard control
    obj.initKeyboard();
    
    // Initialize UAV geometry
    obj.initializeVertices();
    obj.initializePreviousState();
    obj.initializeVerticesIndices();
    
    
    // initialize Easy3D.
    easy3d::initialize(true);
    
    // Create the default Easy3D viewer. Note: a viewer must be created before creating any drawables.
    easy3d::Viewer viewer("UAV Simulator");

    // Draw the aircraft and 3D graphs
    obj.createAircraftDrawable(viewer);
    obj.createGridDrawable(viewer);

    // Make sure everything is within the visible region of the viewer.
    viewer.fit_screen();
    viewer.set_animation(true); // -> True here will create a log file in /build
    
    // Animation function
    viewer.animation_func_ = [&](easy3d::Viewer* v) -> bool 
    {
        return obj.animate(v, state, dt);
    };

      
    return viewer.run();
    
}

