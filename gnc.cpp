/**
 * @file gnc.cpp
 * @brief Implementation of the Guidance, Navigation, and Control (GNC) system.
 */
#include "gnc.h"
#include <iostream>

/**
 * @brief Set the waypoints for the GNC system.
 *
 * @param waypoints The list of waypoints to navigate.
 */
void GNC::setWaypoints(const WaypointList& waypoints) 
{
    waypoints_ = waypoints;
}

/**
 * @brief Update the GNC system with the current aircraft state.
 *
 * This function processes the aircraft's current position and determines
 * navigation commands based on the active waypoint. If the aircraft is close
 * enough to the current waypoint, it automatically advances to the next.
 *
 * @param aircraft Current aircraft state.
 * @param dt Time step duration in seconds.
 */
void GNC::update(const Aircraft& aircraft, float dt) 
{
    if (waypoints_.missionComplete()) return;
    // Extract current aircraft position: North, East, Down coordinates
    easy3d::vec3 current_pos(aircraft.X[0], aircraft.X[1], aircraft.X[2]); // X[0-2] = pn,pe,pd
    // Get the current target waypoint
    easy3d::vec3 target = waypoints_.currentWaypoint();

    // Debug output for navigation target
    std::cout << "Navigating to waypoint at " << target.x << ", " << target.y << ", " << target.z << std::endl;

    // // If within 5 meters of the waypoint, advance to next waypoint
    if ((target - current_pos).norm() < 5.0f)
    {
        waypoints_.advanceToNext();
    }
}
