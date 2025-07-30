/**
 * @file gnc.h
 * @brief Header file for the Guidance, Navigation, and Control (GNC) system.
 */
#ifndef GNC_H
#define GNC_H

#include "waypoint_list.h"
#include "aircraft/aircraft.h"
#include <easy3d/core/vec.h>
#include "common.h"

/**
 * @class GNC
 * @brief Guidance, Navigation, and Control system for the aircraft.
 *
 * This class manages the navigation through a list of waypoints and updates control
 * commands based on the current state of the aircraft.
 */
class GNC 
{
public:
    GNC();
    /**
     * @brief Set the waypoints for the navigation system.
     * @param waypoints The list of waypoints to follow.
     */
    void setWaypoints(const WaypointList& waypoints);
    /**
     * @brief Update the GNC system state.
     *
     * This function should be called every simulation step and will process
     * the aircraft state to produce new control commands.
     *
     * @param aircraft The current state of the aircraft.
     * @param dt Time step in seconds.
     */
    void update(const Aircraft& aircraft, float dt);


    // New additions: For Trim, linearisation etc
    bool computeTrim(Aircraft& aircraft, double Va, double gamma, double R);
    static double trimObjective(const std::vector<double>& angles, std::vector<double>& grad, void* data);
    static Eigen::VectorXd computeXdot(Aircraft& ac);
    bool linearizeAtTrim(const Aircraft& aircraft);

    struct TrimData
    {
        double Va;
        double gamma;
        double R;
        double phi_fixed;
        double beta_fixed;
        Aircraft* aircraft;
    };



private:
    WaypointList waypoints_;    ///< Current list of waypoints for navigation
};

#endif // GNC_H
