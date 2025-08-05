// This is the main program from which you setup/run the simulation
#include <iostream>
#include <aircraft/aircraft.h>
#include <rendering/world.h>
#include <rendering/simviewer.h>
#include <gnc/mission_manager.h>
#include <gnc/waypoint_list.h>
#include <gnc/gnc.h>
#include <easy3d/viewer/viewer.h>
#include <easy3d/renderer/camera.h>
#include <easy3d/core/types.h>
#include <easy3d/util/resource.h>
#include <easy3d/util/initializer.h>




int main()
{
    // Simulation parameters
    int steps = 10;
    double dt = 0.02;//0.0001;
    int vehicle_count = 1;
    std::string fname = "../data/data.txt";

    Aircraft drone(fname, vehicle_count);
    drone.steps = steps;
    drone.dt = dt;

    World world;
    //======================= GNC: TRIM & LINEAR INCLUDED ====
    GNC gnc;
    double trim_airspeed = 17.0;    // m/s
    double trim_flightpath = 0.0;   // radians
    double trim_turn_radius = std::numeric_limits<double>::infinity();  // or a large number like 1e9



    if (gnc.computeTrim(drone, trim_airspeed, trim_flightpath,trim_turn_radius))
    {
        std::cout << "Trim computed successfully." << std::endl;
         // --- overwrite trimmed state with desired starting position ---
        /*drone.X[0] = 0.0;     // pn (North)
        drone.X[1] = 0.0;     // pe (East)
        drone.X[2] = 2000.0;  // pd (Down)
        //Overwrite state & controls with trimmed values
        /*drone.X[3] = drone.u = trim_airspeed * std::cos(drone.alpha);
        drone.X[4] = drone.v = trim_airspeed * std::sin(drone.beta);
        drone.X[5] = drone.w = 0.0;
        drone.X[9] = drone.phi;
        drone.X[10] = drone.theta = drone.alpha + trim_flightpath;
        drone.X[11] = 0.0; // yaw*/

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
    drone.file_name = "/home/fahim/Coding/Git/UAV-Simulator/assets/y_for_z_up_uav2_2000.stl";
    drone.mesh.reset(easy3d::SurfaceMeshIO::load(drone.file_name));//drone.mesh = easy3d::SurfaceMeshIO::load(drone.file_name);

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


