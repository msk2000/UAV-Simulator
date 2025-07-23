/**
 * @file gnc.cpp
 * @brief Implementation of the Guidance, Navigation, and Control (GNC) system.
 */
#include "gnc.h"
#include <iostream>
#include <nlopt.hpp>


GNC::GNC() {}


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



bool GNC::computeTrim(Aircraft& aircraft, double Va, double gamma, double R)
{
    std::cout << "Computing trim for Va = " << Va << ", γ = " << gamma << ", R = " << R << std::endl;

    // Initial guess: alpha, phi, beta
    std::vector<double> x0 = {0.05, 0.0, 0.0}; // alpha, phi, beta

    TrimData data{Va, gamma, R, &aircraft};

    nlopt::opt opt(nlopt::LN_COBYLA, 3);
    opt.set_lower_bounds({-0.5, -M_PI/4, -0.5});
    opt.set_upper_bounds({ 0.5,  M_PI/4,  0.5});
    opt.set_min_objective(trimObjective, &data);
    opt.set_xtol_rel(1e-4);

    double minf;
    nlopt::result result = opt.optimize(x0, minf);

    if (result < 0) {
        std::cerr << "Trim optimization failed!" << std::endl;
        return false;
    }

    std::cout << "Trim succeeded with α = " << x0[0] << ", ϕ = " << x0[1] << ", β = " << x0[2] << std::endl;
    std::cout << "Trim control inputs:\n";
    std::cout << "  Elevator deflection δₑ = " << aircraft.delta_e << " rad\n";
    std::cout << "  Aileron deflection δₐ = " << aircraft.delta_a << " rad\n";
    std::cout << "  Rudder deflection  δᵣ = " << aircraft.delta_r << " rad\n";
    std::cout << "  Throttle setting   δₜ = " << aircraft.delta_t << " (0–1)\n";

    std::cout << "Min residual: " << minf << std::endl;
    return true;
}

double GNC::trimObjective(const std::vector<double>& angles, std::vector<double>& grad, void* data)
{
    auto* trim = static_cast<TrimData*>(data);
    Aircraft& ac = *trim->aircraft;
    double Va = trim->Va;
    double gamma = trim->gamma;
    double R = trim->R;

    double alpha = angles[0];
    double phi   = angles[1];
    double beta  = angles[2];

    // Step 3: Compute x*
    double u = Va * std::cos(alpha) * std::cos(beta);
    double v = Va * std::sin(beta);
    double w = Va * std::sin(alpha) * std::cos(beta);
    double theta = alpha + gamma;

    double p = -Va / R * std::sin(theta);
    double q = Va / R * std::sin(phi) * std::cos(theta);
    double r = Va / R * std::cos(phi) * std::cos(theta);

    // Set aircraft state x*
    ac.X[3] = u; ac.X[4] = v; ac.X[5] = w;
    ac.X[6] = p; ac.X[7] = q; ac.X[8] = r;
    ac.X[9] = phi;  // ϕ
    ac.X[10] = theta; // θ
    ac.X[11] = 0.0;   // ψ = yaw (irrelevant for trim)

    // Step 4: Set guess for control inputs (e.g., elevator, aileron, rudder, throttle)
    // For now, lez do zero inputs and later add another optimization layer
    ac.delta_e = 0.0;
    ac.delta_a = 0.0;
    ac.delta_r = 0.0;
    ac.delta_t = 0.5;

    // Step 5: Evaluate f(x,u)
    ac.calculate_forces();
    ac.calculate_moments();

    // Use the 6 DoF Newton-Euler equations to compute x_dot
    Eigen::VectorXd x_dot = computeXdot(ac);  /// TODO  need to implement this better ?
    // Step 6: Compute cost ||x_dot||
    double cost = x_dot.squaredNorm(); // or select subset of terms you want to minimize

    return cost;
}

Eigen::VectorXd GNC::computeXdot(const Aircraft& ac)
{
    Eigen::VectorXd x_dot(12);
    // fill in translational, rotational derivatives based on Newton-Euler equations
    // use: fx, fy, fz, ell, m, n, current state X, mass, inertia, etc.
    return x_dot;
}



bool GNC::linearizeAtTrim(const Aircraft& aircraft)
{
    std::cout << "Linearizing at trim point..." << std::endl;

    // Use aircraft's equations to compute Jacobians:
    // A = ∂f/∂x at (x*, u*), B = ∂f/∂u at (x*, u*)
    // Use finite differences or symbolic derivation

    return true;
}
