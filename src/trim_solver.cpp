#include "trim_solver.h"
#include <cmath>
#include <iostream>

TrimSolver::TrimSolver(Aircraft& aircraft)
    : aircraft_(aircraft), Va_(0), gamma_(0), R_(0), alpha_(0), beta_(0), phi_(0) {}

void TrimSolver::setTrimTargets(double Va, double gamma, double R) 
{
    Va_ = Va;
    gamma_ = gamma;
    R_ = R;
}

void TrimSolver::solve() 
{
    gradientDescent();
}

Eigen::VectorXd TrimSolver::getTrimmedState() const 
{
    return x_trim_;
}

Eigen::VectorXd TrimSolver::getTrimmedInput() const 
{
    return u_trim_;
}

Eigen::VectorXd TrimSolver::computeTrimState(double alpha, double beta, double phi) 
{
    Eigen::VectorXd x(12);

    // Velocity components
    double u = Va_ * cos(beta) * cos(alpha);
    double v = Va_ * sin(beta);
    double w = Va_ * cos(beta) * sin(alpha);

    // Orientation
    double theta = alpha + gamma_;
    double psi = 0.0;

    // Angular rates (from coordinated turn rate for steady turn)
    double g = aircraft_.g;
    double q = 0.0;
    double r = Va_ / R_ * cos(phi);
    double p = q * sin(theta) + r * tan(theta) * cos(phi); // Approximate for coordinated turn

    x << 0, 0, 0,  // Position (pn, pe, pd)
         u, v, w,  // Body velocities
         phi, theta, psi, // Orientation
         p, q, r;   // Angular rates

    return x;
}

std::vector<double> TrimSolver::compute_trim_inputs(const std::vector<double>& x_trim_guess) {
    double best_cost = std::numeric_limits<double>::max();
    std::vector<double> best_u = {0.0, 0.0, 0.0, 0.5}; // elevator, aileron, rudder, throttle

    // Simple brute-force over a coarse range
    for (double delta_e = -0.3; delta_e <= 0.3; delta_e += 0.05) {
        for (double delta_a = -0.3; delta_a <= 0.3; delta_a += 0.05) {
            for (double delta_r = -0.3; delta_r <= 0.3; delta_r += 0.05) {
                for (double delta_t = 0.0; delta_t <= 1.0; delta_t += 0.05) {
                    // Set aircraft state
                    Aircraft ac;
                    ac.setState(x_trim_guess);
                    ac.delta_e = delta_e;
                    ac.delta_a = delta_a;
                    ac.delta_r = delta_r;
                    ac.delta_t = delta_t;

                    // Calculate forces & moments
                    ac.calculate_forces();
                    ac.calculate_moments();

                    // Compute derivatives from forces/moments
                    std::vector<double> x_dot = ac.compute_state_derivatives();

                    // Cost = norm of state derivatives
                    double cost = 0.0;
                    for (double d : x_dot) cost += d * d;

                    if (cost < best_cost) {
                        best_cost = cost;
                        best_u = {delta_e, delta_a, delta_r, delta_t};
                    }
                }
            }
        }
    }

    return best_u;
}

Eigen::VectorXd TrimSolver::dynamics(const Eigen::VectorXd& x, const Eigen::VectorXd& u) 
{
    // Forward wrapper to aircraft model
    // Need to implement this to evaluate the full f(x, u)
    Eigen::VectorXd x_dot(12);
    // map x,u into Aircraft class, call existing dynamics functions
    return x_dot;
}

double TrimSolver::cost(const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
    Eigen::VectorXd x_dot = dynamics(x, u);
    return x_dot.squaredNorm();  // J = ||f(x,u)||^2
}

void TrimSolver::gradientDescent() 
{
    double eps = 1e-5;
    double lr = 1e-2;
    int max_iter = 100;

    alpha_ = 0.0;
    beta_ = 0.0;
    phi_ = 0.0;

    for (int i = 0; i < max_iter; ++i) {
        Eigen::VectorXd x = computeTrimState(alpha_, beta_, phi_);
        Eigen::VectorXd u = computeTrimInput(x);
        double J = cost(x, u);

        // Finite difference gradients
        double dJ_dalpha = (cost(computeTrimState(alpha_ + eps, beta_, phi_), computeTrimInput(x)) - J) / eps;
        double dJ_dbeta  = (cost(computeTrimState(alpha_, beta_ + eps, phi_), computeTrimInput(x)) - J) / eps;
        double dJ_dphi   = (cost(computeTrimState(alpha_, beta_, phi_ + eps), computeTrimInput(x)) - J) / eps;

        // Gradient descent update
        alpha_ -= lr * dJ_dalpha;
        beta_  -= lr * dJ_dbeta;
        phi_   -= lr * dJ_dphi;

        if (J < 1e-6) break;
    }

    x_trim_ = computeTrimState(alpha_, beta_, phi_);
    u_trim_ = computeTrimInput(x_trim_);
}


