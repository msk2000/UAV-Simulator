#pragma once

#include <ct/core/core.h>  // Include CT core functionalities

/**
 * @brief Adapter class to wrap a custom Aircraft dynamics model into the
 * Control Toolbox (CT) ControlledSystem interface.
 *
 * This allows the aircraft model to be used with CT's optimization and control
 * solvers, such as for trajectory optimization or LQR/MPC.
 */
class UAVSystemAdapter : public ct::core::ControlledSystem<12, 4>
{
public:
    /**
     * @brief Constructor for UAVSystemAdapter.
     *
     * @param aircraft_ptr Pointer to the aircraft model instance.
     */
    UAVSystemAdapter(Aircraft* aircraft_ptr)
        : ct::core::ControlledSystem<12, 4>(), aircraft(aircraft_ptr)
    {
    }

    /**
     * @brief Compute the state derivative given the current state and control input.
     *
     * This function overrides CT's pure virtual method and allows the CT solver
     * to simulate dynamics based on your custom Aircraft model.
     *
     * @param state      The current system state vector (size 12).
     * @param t          The current time (in seconds).
     * @param control    The current control input vector (size 4).
     * @param derivative The output state derivative to be filled (size 12).
     */
    void computeControlledDynamics(
        const ct::core::StateVector<12>& state,
        const double& t,
        const ct::core::ControlVector<4>& control,
        ct::core::StateVector<12>& derivative) override
    {
        // Convert Eigen::Matrix to std::vector for Aircraft model
        std::vector<double> x_vec(state.data(), state.data() + state.size());
        std::vector<double> u_vec(control.data(), control.data() + control.size());

        // Set the state and control in the aircraft model
        aircraft->set_state(x_vec);
        aircraft->set_control(u_vec);

        // Compute the derivative using the Aircraft model's dynamics
        std::vector<double> f_vec = aircraft->compute_f_euler(x_vec);

        // Copy the computed derivative back into the CT vector
        for (int i = 0; i < 12; ++i)
            derivative(i) = f_vec[i];
    }

    virtual UAVSystemAdapter* clone() const override
    {
        return new UAVSystemAdapter(*this);
    }


private:
    Aircraft* aircraft;  ///< Pointer to the aircraft model used for dynamics
};
