//Implementation file for GNC 
#include "gnc.h"
#include <iostream>

void GNC::setWaypoints(const WaypointList& waypoints) 
{
    waypoints_ = waypoints;
}

void GNC::update(const Aircraft& aircraft, float dt) 
{
    if (waypoints_.missionComplete()) return;

    easy3d::vec3 current_pos(aircraft.X[0], aircraft.X[1], aircraft.X[2]); // X[0-2] = pn,pe,pd
    easy3d::vec3 target = waypoints_.currentWaypoint();

    // Here we compute heading/altitude errors and apply PID controllers
    std::cout << "Navigating to waypoint at " << target.x << ", " << target.y << ", " << target.z << std::endl;

    // Once close enough to the waypoint, advance
    if ((target - current_pos).norm() < 5.0f)
    {
        waypoints_.advanceToNext();
    }
}
