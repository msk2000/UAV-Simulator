/**
 * @file aircraft.h
 * @brief Declaration of the Aircraft class for the UAV Simulator.
 *
 * The Aircraft class models a fixed‑wing UAV. It loads parameters from file,
 * calculates aerodynamic forces and moments, integrates the dynamics, and
 * updates its 3D geometry for rendering in the Easy3D viewer.
 */

#ifndef AIRCRAFT_H
#define AIRCRAFT_H

#include <memory> // For std::unique_ptr
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <easy3d/viewer/viewer.h>
#include <easy3d/renderer/drawable_triangles.h>
#include <easy3d/renderer/drawable_lines.h>
#include <easy3d/renderer/vertex_array_object.h>
#include <easy3d/core/surface_mesh.h>
#include <easy3d/fileio/surface_mesh_io.h>
#include <fstream>
#include <cmath>


#define _USE_MATH_DEFINES

/**
 * @class Aircraft
 * @brief Represents a fixed‑wing UAV including its dynamics and 3D rendering.
 *
 * Public members expose state variables and parameters, while methods handle:
 * - Loading aircraft parameters from file
 * - Computing forces, moments, and trim
 * - Integrating dynamics (RK4)
 * - Rendering the aircraft and axes in Easy3D
 * - Updating the aircraft per simulation step
 */
class Aircraft 
{
public:

    // ===== Public state variables (documented briefly for clarity) =====
    double clock;    ///< Simulation clock [s]
    double pn, pe, pd; ///< Position in NED frame [m]
    double u, v, w;    ///< Body-axis velocities [m/s]
    double phi, theta, psi; ///< Euler angles [rad]
    double p, q, r;    ///< Angular rates [rad/s]
    double V_m;        ///< Airspeed [m/s]
    double alpha, beta;///< AoA and sideslip
    double fx, fy, fz; ///< Body-axis forces [N]
    double ell, m, n;  ///< Body-axis moments [Nm]
    
    // More
    std::vector<double> velocity_b;
    //Rates
    double pn_dot;
    double pe_dot;
    double pd_dot;
    double u_dot;
    double v_dot;
    double w_dot; 
    double phi_dot;
    double theta_dot;
    double psi_dot;
    double p_dot;
    double q_dot;
    double r_dot;

    int steps;
    double dt;
    int id;
    //Physical
    double mass;
    double Jx;
    double Jy;
    double Jz;
    double Jxz;
    double wing_area;
    double wing_chord;
    double wing_span;
    double wing_aspect_ratio = (wing_span*wing_span)/wing_area;
    double e; // oswalds efficiency factor
    double g; // gravity
    double rho; // density of air
    //Electric propulsion 
    double k_motor; //motor constant
    double prop_area; //s_[rop]
    double prop_thrust_coef; // constant determined by experiment k_t_p
    double prop_efficiency; // e
    double prop_omega; //angular speed k_omega

    //Inertial constants

    double Gamma;
    double Gamma_1;
    double Gamma_2;
    double Gamma_3;
    double Gamma_4;
    double Gamma_5;
    double Gamma_6;
    double Gamma_7;
    double Gamma_8;

    // ===== Roll/Yaw combined coefficients after applying Gamma transforms =====
    double C_p_0;
    double C_p_beta;
    double C_p_p;
    double C_p_r;
    double C_p_delta_a;
    double C_p_delta_r;

    double C_r_0;
    double C_r_beta;
    double C_r_p;
    double C_r_r;
    double C_r_delta_a;
    double C_r_delta_r;

    //Forces&Moments function:
    double Cd_of_alpha;
      
        // sigma(alpha)
        double sigma_num;
        double sigma_den;
        double sigma_of_alpha;
    
        // Cl of flat plate
        double Cl_flat_plate;
        // Linear Cl
        double Cl_linear;
        // Combined Cl
        double Cl_of_alpha;

        // Coefficients for X and Z directions
        double CxAlpha;
        double CxqAlpha;
        double CxdeltaeAlpha;
        double CzAlpha;
        double CzqAlpha;
        double CzdeltaeAlpha;

        // Sources of Forces
        Eigen::Vector3d force_g; // Component: Gravity
        double force_aero1;

        Eigen::Vector3d force_aero2;
        Eigen::Vector3d force_aero;

        double force_prop1;
        Eigen::Vector3d force_prop2;
        Eigen::Vector3d force_prop;

        Eigen::Vector3d Force;

    

        // Moment/Torque calculations

        double Aero_t1 = force_aero1;
        
        Eigen::Vector3d Aero_t2;
        Eigen::Vector3d Aero_torque;
        Eigen::Vector3d Prop_torque;
        Eigen::Vector3d Torque;


    //Stability & Control derivatives

    double C_L_0;
    double C_L_alpha;
    double C_L_q;
    double C_L_delta_e;

    double C_D_0;
    double C_D_alpha;
    double C_D_p;
    double C_D_q;
    double C_D_delta_e;

    double C_m_0;
    double C_m_alpha;
    double C_m_q;
    double C_m_delta_e;

    double C_Y_0;
    double C_Y_beta;
    double C_Y_p;
    double C_Y_r;
    double C_Y_delta_a;
    double C_Y_delta_r;

    double C_ell_0;
    double C_ell_beta;
    double C_ell_p;
    double C_ell_r;
    double C_ell_delta_a;
    double C_ell_delta_r;

    double C_n_0;
    double C_n_beta;
    double C_n_p;
    double C_n_r;
    double C_n_delta_a;
    double C_n_delta_r;

    double C_prop;
    double trans_rate;
    double epsilon;
    double alpha0;
    double beta0;

    // Initial state: To set the initial condition
    double pn_0;
    double pe_0;
    double pd_0;

    double u_0; //body axis velocity
    double v_0; //body axis velocity
    double w_0; //body axis velocity

    double phi_0; 
    double theta_0;
    double psi_0;

    double p_0;
    double q_0;
    double r_0;

    double delta_t; //throttle
    double delta_a; //aileron
    double delta_e; //elevator
    double delta_r; //rudder
    // aircraft specific maximum control inputs
    double delta_t_max; 
    double delta_t_min;
    double delta_a_max;
    double delta_a_min;
    double delta_e_max;
    double delta_e_min;
    double delta_r_max;
    double delta_r_min;

    // Geometry and init
    float old_roll;
    float old_pitch;
    float old_yaw;
    
    float old_pn;
    float old_pe;
    float old_pd; 

    std::vector<double> vertices_x;
    std::vector<double> vertices_y; 
    std::vector<double> vertices_z;
    int vertices_size; 
    

    // For scaling the thing
    int aircraft_scale = 20;

    std::vector<std::vector<int>> faces;
    std::vector<easy3d::vec3> vertices_aircraft;
    std::vector<unsigned int> indices;
    
    std::vector<easy3d::vec3> dummy_points;//DUMMY var for points
    std::vector<easy3d::vec3> &points;

    easy3d::TrianglesDrawable* aircraft;


    easy3d::LinesDrawable* axesDrawable;
    std::vector<easy3d::vec3> axes_vertices;
    std::vector<easy3d::vec3> original_vertices; // to stop compound updates
    std::vector<easy3d::vec3> axes_vertices_original;

    
    //surface mesh
    std::string file_name;

    std::unique_ptr<easy3d::SurfaceMesh> mesh;// easy3d::SurfaceMesh* mesh;

    /**
     * @brief Construct a new Aircraft, load parameters, and initialize states.
     * @param fname Path to aircraft parameter file.
     * @param vehicle_count Reference to vehicle counter.
     */
    Aircraft(const std::string& fname, int& vehicle_count);
     /**
     * @brief Destructor for Aircraft.
     */
    ~Aircraft();
    
    
    
    /**
     * @brief Load an aircraft parameter file.
     * @param filePath File path of parameter file.
     * @param vehicle_count Reference to vehicle counter.
     */
    void load_a_plane(const std::string& filePath, int& vehicle_count);
    
    
    
    
    //State vector
    std::vector <double> X = {pn,pe,pd,u,v,w,p,q,r,phi,theta,psi,fx,fy,fz,ell,m,n};
    
    


    /**
     * @brief Update aircraft states, geometry, and axes per simulation step.
     * @param vertices Pointer to aircraft vertex buffer.
     * @param axesVertices Pointer to axes vertex buffer.
     * @param dt Time step in seconds.
     * @return Updated aircraft vertex pointer.
     */
    easy3d::vec3* update_aircraft(easy3d::vec3* vertices,easy3d::vec3* axesVertices, double& dt);


    
    /**
     * @brief Render the aircraft model into the viewer.
     * @param viewer Reference to Easy3D viewer.
     */
    void renderAircraft(easy3d::Viewer& viewer); // NEW
     /**
     * @brief Create and render local coordinate axes.
     * @param viewer Reference to Easy3D viewer.
     */
    void createAxesDrawable(easy3d::Viewer& viewer);
    /**
     * @brief Animate the aircraft for one frame.
     * @param viewer Pointer to Easy3D viewer.
     * @param dt Time step in seconds.
     * @return True if animation update succeeded.
     */
    bool animate(easy3d::Viewer* viewer, double dt);
    

     /**
     * @brief Render HUD overlay of state variables.
     * @param tr Text renderer for HUD.
     * @param viewer Pointer to Easy3D viewer.
     */
    void render_HUD(easy3d::TextRenderer& tr, easy3d::Viewer* viewer) const;
    /**
     * @brief Print current state variables to console for debugging.
     */
    void printState();

    /**
     * @brief Get the drawable for shading or viewer operations.
     * @return Aircraft mesh as TrianglesDrawable.
     */
    easy3d::TrianglesDrawable* getAircraftDrawable() const
    {
        return aircraft;
    }
    
    //==================== FRIENDSHIP with GNC ==================
    friend class GNC;

    struct ControlInputs
    {
        double delta_e; // elevator
        double delta_a; // aileron
        double delta_r; // rudder
        double delta_t; // throttle
    };
     /**
     * @brief Compute control inputs that achieve trim for given flight conditions.
     * @param alpha Angle of attack.
     * @param beta Side slip angle.
     * @param phi Roll angle.
     * @param p Roll rate.
     * @param q Pitch rate.
     * @param r Yaw rate.
     * @param theta Pitch angle.
     * @param Va Airspeed.
     * @param R Turn radius.
     * @return Structure containing trim control inputs.
     */
    ControlInputs computeTrimControls(
    double alpha, double beta, double phi,
    double p, double q, double r,
    double theta, double Va, double R);


    private:

    /// Perform force calculations (gravity, aero, prop).
    void calculate_forces();
    /// Calculate body-frame velocity and angles.
    void calculate_body_frame_velocity_and_angles();
    /// Calculate lift and drag coefficients.
    void calculate_lift_drag_coefficients();
    /// Calculate aerodynamic and propulsive moments.
    void calculate_moments();

    /// Apply rotation to aircraft mesh vertices.
    void rotate(easy3d::vec3* vertices);
    /// Apply rotation to axes vertices.
    void rotate_axes(easy3d::vec3* axesVertices);
    /// Apply translation to aircraft mesh vertices.
    void translate(easy3d::vec3* vertices);
    /// Apply translation to axes vertices.
    void translate_axes(easy3d::vec3* axesVertices);

    /// Integrate dynamics using Runge-Kutta 4th order
    void RK4(std::vector<double>& X,  double dt); 
    // ===== Derivative helper functions =====
    // Function to calculate pn_dot
    double calculate_pn_dot(double& u, double& v, double& w, double& phi, double& theta, double& psi);
    //Function to calculate pe_dot
    double calculate_pe_dot(double& u, double& v, double& w, double& phi, double& theta, double& psi);
    // Function to calculate pd_dot
    double calculate_pd_dot(double& u, double& v, double& w, double& phi, double& theta);
    // Function to calculate phi_dot
    double calculate_phi_dot(double& p, double& q, double& r, double& phi, double& theta);
    // Function to calculate theta_dot
    double calculate_theta_dot(double& q, double& r,double& phi);
    // Function to calculate psi_dot
    double calculate_psi_dot(double& q, double& r, double& phi, double& theta);
    // Function to calculate u_dot
    double calculate_u_dot(double& v, double& w, double& q, double& r, double& fx, double& mass);
    // Function to calculate v_dot
    double calculate_v_dot(double& u, double& w, double&p,double& fy, double& mass);
    // Function to calculate w_dot
    double calculate_w_dot(double& u, double& v, double& p,  double& q,double& fz, double& mass);
    // Function to calculate p_dot
    double calculate_p_dot(double& p, double& q, double& r,  double& ell,double& n, double& Gamma_1,double& Gamma_2,double& Gamma_3,double& Gamma_4);
    // Function to calculate q_dot
    double calculate_q_dot(double& p, double& r,  double& m ,double& Jy, double& Gamma_5,double& Gamma_6);
    // Function to calculate r_dot
    double calculate_r_dot(double& p, double& q, double& r,  double& ell,double& n, double& Gamma_1,double& Gamma_4,double& Gamma_7,double& Gamma_8);




    
};

#endif
