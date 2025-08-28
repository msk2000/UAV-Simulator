/**
 * @file gnc.cpp
 * @brief Implementation of the Guidance, Navigation, and Control (GNC) system.
 */
#include "gnc/gnc.h"
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

    // ---- Fix β and compute φ as per book ----
    double beta_fixed = 0.0; // no sideslip
    double phi_fixed = 0.0;
    if (std::isfinite(R) && std::abs(R) > 1e-6)
    {
        phi_fixed = std::atan((Va * Va * std::cos(gamma)) / (aircraft.g * R));
    }

    // Only alpha is optimized
    std::vector<double> x0 = {0.05}; // initial alpha guess

    trimData = {Va, gamma, R, phi_fixed, beta_fixed, &aircraft};
    TrimData& data = trimData; // use the member for NLopt too


    nlopt::opt opt(nlopt::LN_COBYLA, 1);
    opt.set_lower_bounds({-0.5});
    opt.set_upper_bounds({0.5});
    opt.set_min_objective(trimObjective, &data);
    opt.set_xtol_rel(1e-4);

    double minf;
    nlopt::result result = opt.optimize(x0, minf);

    if (result < 0)
    {
        std::cerr << "Trim optimization failed!" << std::endl;
        return false;
    }

    // Update aircraft trim values
    aircraft.alpha = x0[0];
    aircraft.phi   = phi_fixed;
    aircraft.beta  = beta_fixed;

    double theta = aircraft.alpha + gamma;

    // Build trim state vector (18 elements)
    aircraft.X =
    {
        -3000.0,                          // pn
        -1350.0,                          // pe
        2020.0,                       // pd
        Va * std::cos(aircraft.alpha) * std::cos(aircraft.beta), // u
        Va * std::sin(aircraft.beta), // v
        0.0,                          // w = 0 for trim in body frame (Eq. 5.21)
        0.0,                          // p
        0.0,                          // q
        0.0,                          // r
        aircraft.phi,                 // phi
        theta,                        // theta
        0.0,                          // psi
        0.0, 0.0, 0.0,                 // fx, fy, fz
        0.0, 0.0, 0.0                  // ell, m, n
    };

    // Compute and store trim control inputs
    auto ci = aircraft.computeTrimControls(
        aircraft.alpha,
        aircraft.beta,
        aircraft.phi,
        0.0, 0.0, 0.0,    // p, q, r
        theta,
        Va,
        R
    );

    aircraft.delta_e = ci.delta_e;
    aircraft.delta_a = ci.delta_a;
    aircraft.delta_r = ci.delta_r;
    aircraft.delta_t = ci.delta_t;

    // Debug print
    std::cout << "Trim succeeded with α = " << aircraft.alpha
              << ", ϕ = " << aircraft.phi
              << ", β = " << aircraft.beta << std::endl;
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

    // Optimization variable: only alpha
    double alpha = angles[0];
    // Fixed from TrimData
    double phi = trim->phi_fixed;
    double beta = trim->beta_fixed;

    // ===== Compute steady-state values =====
    double u = Va * std::cos(alpha) * std::cos(beta);
    double v = Va * std::sin(beta);
    double w = Va * std::sin(alpha) * std::cos(beta);
    double theta = alpha + gamma;

    // Angular rates for the specified trim (Eq. 5.19)
    double p = -Va / R * std::sin(theta);
    double q =  Va / R * std::sin(phi) * std::cos(theta);
    double r =  Va / R * std::cos(phi) * std::cos(theta);

    // Set aircraft state
    ac.X[3] = u;
    ac.X[4] = v;
    ac.X[5] = w;
    ac.X[6] = p;
    ac.X[7] = q;
    ac.X[8] = r;
    ac.X[9]  = phi;
    ac.X[10] = theta;
    ac.X[11] = 0.0;

    // ===== Initial control guesses =====
    ac.delta_e = 0.0;
    ac.delta_a = 0.0;
    ac.delta_r = 0.0;
    ac.delta_t = 0.5;

    // Evaluate forces/moments
    ac.calculate_forces();
    ac.calculate_moments();

    // Compute control inputs for trim
    Aircraft::ControlInputs trim_controls = ac.computeTrimControls(alpha, beta, phi, p, q, r, theta, Va, R);
    ac.delta_e = trim_controls.delta_e;
    ac.delta_a = trim_controls.delta_a;
    ac.delta_r = trim_controls.delta_r;
    ac.delta_t = trim_controls.delta_t;

    // Recompute forces/moments
    ac.calculate_forces();
    ac.calculate_moments();

    // ===== Compute state derivatives =====
    Eigen::VectorXd x_dot = computeXdot(ac);

    // ===== Desired derivatives =====
    double h_dot_desired = Va * std::sin(gamma);
    double psi_dot_desired = 0.0;
    if (std::isfinite(R) && std::abs(R) > 1e-6) {
        psi_dot_desired = Va / (R * std::cos(gamma));
    }

    // ===== Compute residuals =====
    double du     = x_dot[3];
    double dv     = x_dot[4];
    double dw     = x_dot[5];
    double dp     = x_dot[6];
    double dq     = x_dot[7];
    double dr     = x_dot[8];
    double dphi   = x_dot[9];
    double dtheta = x_dot[10];
    double dh = (-x_dot[2]) - h_dot_desired;//double dh     = x_dot[2] - h_dot_desired;
    double dpsi   = x_dot[11] - psi_dot_desired;

    double cost = du*du + dv*dv + dw*dw
                + dp*dp + dq*dq + dr*dr
                + dphi*dphi + dtheta*dtheta
                + dh*dh + dpsi*dpsi;

    std::cout << "du dv dw dp dq dr dphi dtheta dh dpsi: "
              << du << " " << dv << " " << dw << " "
              << dp << " " << dq << " " << dr << " "
              << dphi << " " << dtheta << " "
              << dh << " " << dpsi << std::endl;

    return cost;
}




Eigen::VectorXd GNC::computeXdot(Aircraft& ac)
{
    Eigen::VectorXd x_dot(12);
     // Extract states
    double pn    = ac.X[0];
    double pe    = ac.X[1];
    double pd    = ac.X[2];
    double u     = ac.X[3];
    double v     = ac.X[4];
    double w     = ac.X[5];
    double p     = ac.X[6];
    double q     = ac.X[7];
    double r     = ac.X[8];
    double phi   = ac.X[9];
    double theta = ac.X[10];
    double psi   = ac.X[11];

    // Shortcuts
    double fx = ac.fx;
    double fy = ac.fy;
    double fz = ac.fz;
    double ell = ac.ell;
    double m   = ac.m;
    double n   = ac.n;
    double mass = ac.mass;

    // Inertia terms
    double Gamma_1 = ac.Gamma_1;
    double Gamma_2 = ac.Gamma_2;
    double Gamma_3 = ac.Gamma_3;
    double Gamma_4 = ac.Gamma_4;
    double Gamma_5 = ac.Gamma_5;
    double Gamma_6 = ac.Gamma_6;
    double Gamma_7 = ac.Gamma_7;
    double Gamma_8 = ac.Gamma_8;
    double Jy = ac.Jy;

    // Compute derivatives using Aircraft class functions
    x_dot[0] = ac.calculate_pn_dot(u, v, w, phi, theta, psi); // \dot{pn}
    x_dot[1] = ac.calculate_pe_dot(u, v, w, phi, theta, psi); // \dot{pe}
    x_dot[2] = ac.calculate_pd_dot(u, v, w, phi, theta);      // \dot{pd}

    x_dot[3] = ac.calculate_u_dot(v, w, q, r, fx, mass);      // \dot{u}
    x_dot[4] = ac.calculate_v_dot(u, w, p, fy, mass);         // \dot{v}
    x_dot[5] = ac.calculate_w_dot(u, v, p, q, fz, mass);      // \dot{w}

    x_dot[6] = ac.calculate_p_dot(p, q, r, ell, n, Gamma_1, Gamma_2, Gamma_3, Gamma_4); // \dot{p}
    x_dot[7] = ac.calculate_q_dot(p, r, m, Jy, Gamma_5, Gamma_6);                       // \dot{q}
    x_dot[8] = ac.calculate_r_dot(p, q, r, ell, n, Gamma_1, Gamma_4, Gamma_7, Gamma_8); // \dot{r}

    x_dot[9]  = ac.calculate_phi_dot(p, q, r, phi, theta);       // \dot{phi}
    x_dot[10] = ac.calculate_theta_dot(q, r, phi);               // \dot{theta}
    x_dot[11] = ac.calculate_psi_dot(q, r, phi, theta);          // \dot{psi}
                                                                 //
   return x_dot;
}



bool GNC::linearizeAtTrim(Aircraft& drone)
{
    Eigen::VectorXd Xtrim = Eigen::Map<const Eigen::VectorXd>(drone.X.data(), drone.X.size());// current state vector
    Eigen::VectorXd Utrim(4);
    Utrim << drone.delta_e,
             drone.delta_a,
             drone.delta_r,
             drone.delta_t;



    computeLinearModel(drone, Xtrim, Utrim,
                       A, B,
                       A_lat, B_lat,
                       A_lon, B_lon);

    std::cout << "\n=== FULL A ===\n" << A
              << "\n=== FULL B ===\n" << B
              << "\n=== LATERAL A ===\n" << A_lat
              << "\n=== LATERAL B ===\n" << B_lat
              << "\n=== LONGITUDINAL A ===\n" << A_lon
              << "\n=== LONGITUDINAL B ===\n" << B_lon
              << std::endl;

    return true;
}

void GNC::computeLinearModel(Aircraft& aircraft,
                             const Eigen::VectorXd& Xtrim,
                             const Eigen::VectorXd& Utrim,
                             Eigen::MatrixXd& A_full,
                             Eigen::MatrixXd& B_full,
                             Eigen::MatrixXd& A_lat,
                             Eigen::MatrixXd& B_lat,
                             Eigen::MatrixXd& A_lon,
                             Eigen::MatrixXd& B_lon)
{
    const double eps = 1e-5;
    const int n_states = 12;
    const int n_inputs = 4;

    Eigen::VectorXd f0 = aircraft.computeStateDot(Xtrim, Utrim);

    A_full.setZero(n_states, n_states);
    B_full.setZero(n_states, n_inputs);

    // --- Full A ---
    for (int j = 0; j < n_states; ++j) {
        Eigen::VectorXd Xpert = Xtrim;
        Xpert[j] += eps;
        Eigen::VectorXd fpert = aircraft.computeStateDot(Xpert, Utrim);
        A_full.col(j) = (fpert - f0) / eps;
    }

    // --- Full B ---
    for (int j = 0; j < n_inputs; ++j) {
        Eigen::VectorXd Upert = Utrim;
        Upert[j] += eps;
        Eigen::VectorXd fpert = aircraft.computeStateDot(Xtrim, Upert);
        B_full.col(j) = (fpert - f0) / eps;
    }

    // ===== Reduced-order extraction =====
    double u0 = Xtrim[3];
    double v0 = Xtrim[4];
    double w0 = Xtrim[5];
    double Va0 = std::sqrt(u0*u0 + v0*v0 + w0*w0);

    // Lateral indices in full state: beta, p, r, phi, psi
    std::vector<int> lat_state_idx = {4, 6, 8, 9, 11}; // v -> beta computed separately
    std::vector<int> lat_input_idx = {1, 2};           // delta_a, delta_r

    // Longitudinal indices: u, w, q, theta, h
    std::vector<int> lon_state_idx = {3, 5, 7, 10, 2}; // h from pd
    std::vector<int> lon_input_idx = {0, 3};           // delta_e, delta_t

    // Build A_lat, B_lat
    A_lat.resize(lat_state_idx.size(), lat_state_idx.size());
    B_lat.resize(lat_state_idx.size(), lat_input_idx.size());
    for (size_t i = 0; i < lat_state_idx.size(); ++i)
    {
        for (size_t j = 0; j < lat_state_idx.size(); ++j)
        {
            if (j == 0) { // beta column: convert v_dot to beta_dot
                A_lat(i, j) = A_full(lat_state_idx[i], 4) / Va0;
            } else {
                A_lat(i, j) = A_full(lat_state_idx[i], lat_state_idx[j]);
            }
        }
        for (size_t j = 0; j < lat_input_idx.size(); ++j)
        {
            B_lat(i, j) = B_full(lat_state_idx[i], lat_input_idx[j]);
        }
    }

    // Build A_lon, B_lon
    A_lon.resize(lon_state_idx.size(), lon_state_idx.size());
    B_lon.resize(lon_state_idx.size(), lon_input_idx.size());
    for (size_t i = 0; i < lon_state_idx.size(); ++i)
    {
        for (size_t j = 0; j < lon_state_idx.size(); ++j)
        {
            A_lon(i, j) = A_full(lon_state_idx[i], lon_state_idx[j]);
        }
        for (size_t j = 0; j < lon_input_idx.size(); ++j)
        {
            B_lon(i, j) = B_full(lon_state_idx[i], lon_input_idx[j]);
        }
    }


}




