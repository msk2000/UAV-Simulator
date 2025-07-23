// This is the main program from which you setup/run the simulation
#include <iostream>
#include <aircraft.h>
#include <world.h>
#include <simviewer.h>
#include <mission_manager.h>
#include <waypoint_list.h>
#include <gnc.h>
#include <easy3d/viewer/viewer.h>
#include <easy3d/renderer/camera.h>
#include <easy3d/core/types.h>
#include <easy3d/util/resource.h>
#include <easy3d/util/initializer.h>




int main()
{
    // Simulation parameters
    int steps = 10;
    double dt = 0.01;//0.0001;
    int vehicle_count = 1;
    std::string fname = "../data.txt";

    Aircraft drone(fname, vehicle_count);
    drone.steps = steps;
    drone.dt = dt;

    World world;
    //======================= GNC: TRIM & LINEAR INCLUDED ====
    GNC gnc;
    double trim_airspeed = drone.u;    // m/s
    double trim_flightpath = 0.0;   // radians
    double trim_turn_radius = std::numeric_limits<double>::infinity();  // or a large number like 1e9



    if (gnc.computeTrim(drone, trim_airspeed, trim_flightpath,trim_turn_radius))
    {
        std::cout << "Trim computed successfully." << std::endl;

    if (gnc.linearizeAtTrim(drone))
    {
        std::cout << "Linearization successful." << std::endl;
    }
    }
    else
    {
        std::cerr << "Trim computation failed." << std::endl;
    }

    //=======================================================


    easy3d::initialize(true);

    // Use SimViewer instead of Viewer
    SimViewer viewer("UAV Simulator");
    viewer.setAircraft(&drone);  // pass the UAV to the viewer
   // viewer.set_background_color(easy3d::vec4(0.0f, 0.0f, 0.0f, 1.0f));

    // Load and render the model
    drone.file_name = "/home/fahim/Coding/Git/UAV-Simulator/y_for_z_up_uav2_2000.stl";
    drone.mesh = easy3d::SurfaceMeshIO::load(drone.file_name);

    world.createTerrainWithTexture(viewer);
    world.createGridDrawable(viewer);
    drone.renderAircraft(viewer);
    drone.createAxesDrawable(viewer);

    //============= MISSION PLAN & GNC ================
    MissionManager mission;
    Path path = mission.generateSquareCircuit(0, 0, 2800, drone.pd);
    WaypointList waypoints(path.generateWaypoints(150.0f));
    gnc.setWaypoints(waypoints);

    // Render the mission path and waypoints
    path.draw(viewer);
    waypoints.draw(viewer);

    //============= Main Loop ===================

    viewer.set_animation(true);

    viewer.animation_func_ = [&](easy3d::Viewer* v) -> bool
    {
        drone.clock++;
        return drone.animate(v, dt);
    };

    return viewer.run();
}


