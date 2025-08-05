/**
 * @file aircraft.cpp
 * @brief Implementation of the Aircraft class used for UAV simulation.
 *
 * This class models a fixed-wing UAV by loading its aerodynamic and inertial
 * parameters from a configuration file, computing forces and moments, performing
 * state propagation via RK4 integration, and updating/rendering its 3D geometry
 * in the Easy3D viewer. It also provides helper functions for calculating
 * aerodynamic coefficients, control inputs for trim, and HUD rendering.
 *
 * @note This implementation depends on Easy3D for visualization and uses
 *       standard C++ STL and Eigen for math operations.
 */
#include "aircraft/aircraft.h"
#include <easy3d/renderer/text_renderer.h>

/// ================================
// ========== FUNCTIONS ===========
// ================================

/**
 * @brief Load aircraft parameters from a text file and assign them to members.
 * @param filePath Path to parameter file.
 * @param vehicle_count Reference to vehicle counter, used to assign ID.
 * @note Also computes derived parameters like aspect ratio and combined coefficients.
 */
void Aircraft::load_a_plane(const std::string& filePath, int& vehicle_count) 
{
            
    int id = vehicle_count;
             
           
    
    std::ifstream file(filePath);
    if (file.is_open())
    {
        // Read and assign values to member variables
            
        //Physical
        file >> mass;
        file >> Jx;
        file >> Jy;
        file >> Jz;
        file >> Jxz;
        file >> wing_area;
        file >> wing_chord;
        file >> wing_span;
        
        file >> e; // oswalds efficiency factor
        file >> g; // gravity
        file >> rho; // density of air
        
        file >> k_motor; //motor constant
        file >> prop_area; //s_[rop]
        file >> prop_thrust_coef; // constant determined by experiment k_t_p
        file >> prop_efficiency; // e
        file >> prop_omega; //angular speed k_omega

        file >> C_L_0;
        file >> C_L_alpha;
        file >> C_L_q;
        file >> C_L_delta_e;

        file >> C_D_0;
        file >> C_D_alpha;
        file >> C_D_p;
        file >> C_D_q;
        file >> C_D_delta_e;

        file >> C_m_0;
        file >> C_m_alpha;
        file >> C_m_q;
        file >> C_m_delta_e;

        file >> C_Y_0;
        file >> C_Y_beta;
        file >> C_Y_p;
        file >> C_Y_r;
        file >> C_Y_delta_a;
        file >> C_Y_delta_r;

        file >> C_ell_0;
        file >> C_ell_beta;
        file >> C_ell_p;
        file >> C_ell_r;
        file >> C_ell_delta_a;
        file >> C_ell_delta_r;

        file >> C_n_0;
        file >> C_n_beta;
        file >> C_n_p;
        file >> C_n_r;
        file >> C_n_delta_a;
        file >> C_n_delta_r;

        file >> C_prop;
        file >> trans_rate;
        file >> epsilon;
        file >> alpha0;

        
        file >> pn_0;
        file >> pe_0;
        file >> pd_0;

        file >> u_0; //body axis velocity
        file >> v_0; //body axis velocity
        file >> w_0; //body axis velocity

        file >> phi_0; 
        file >> theta_0;
        file >> psi_0;

        file >> p_0;
        file >> q_0;
        file >> r_0;
        file >> delta_t;
        file >> delta_a;
        file >> delta_e;
        file >> delta_r;
        file >> delta_t_max;
        file >> delta_t_min;
        file >> delta_a_max;
        file >> delta_a_min;
        file >> delta_e_max;
        file >> delta_e_min;
        file >> delta_r_max;
        file >> delta_r_min;
        

        file.close();
        wing_aspect_ratio = (wing_span*wing_span)/wing_area;
        Gamma=(Jx*Jz)-(Jxz*Jxz);
        Gamma_1=(Jxz*(Jx-Jy+Jz))/Gamma;
        Gamma_2=(Jz*(Jz-Jy)+(Jxz*Jxz))/Gamma;
        Gamma_3=Jz/Gamma;
        Gamma_4=Jxz/Gamma;
        Gamma_5=(Jz-Jx)/Jy;
        Gamma_6=Jxz/Jy;
        Gamma_7=(((Jx-Jy)*Jx)+(Jxz*Jxz))/Gamma;
        Gamma_8=Jx/Gamma;
        beta0 = 1*(M_PI/180);

        // ===== Compute combined roll/yaw coefficients (Eqns from book) =====
        C_p_0        = Gamma_3 * C_ell_0        + Gamma_4 * C_n_0;
        C_p_beta     = Gamma_3 * C_ell_beta     + Gamma_4 * C_n_beta;
        C_p_p        = Gamma_3 * C_ell_p        + Gamma_4 * C_n_p;
        C_p_r        = Gamma_3 * C_ell_r        + Gamma_4 * C_n_r;
        C_p_delta_a  = Gamma_3 * C_ell_delta_a  + Gamma_4 * C_n_delta_a;
        C_p_delta_r  = Gamma_3 * C_ell_delta_r  + Gamma_4 * C_n_delta_r;

        C_r_0        = Gamma_4 * C_ell_0        + Gamma_8 * C_n_0;
        C_r_beta     = Gamma_4 * C_ell_beta     + Gamma_8 * C_n_beta;
        C_r_p        = Gamma_4 * C_ell_p        + Gamma_8 * C_n_p;
        C_r_r        = Gamma_4 * C_ell_r        + Gamma_8 * C_n_r;
        C_r_delta_a  = Gamma_4 * C_ell_delta_a  + Gamma_8 * C_n_delta_a;
        C_r_delta_r  = Gamma_4 * C_ell_delta_r  + Gamma_8 * C_n_delta_r;

        std::cout << "Aircraft loaded from file successfully" << "\n";
        std::cout << "Vehicle has been given the ID number: "<< vehicle_count <<"\n";
        vehicle_count++;
    }
    else {
        std::cerr << "Unable to open file: " << filePath << "\n";
    }
}




/**
 * @brief Constructor. Loads parameters and initializes state variables.
 * @param fname Path to the aircraft parameter file.
 * @param vehicle_count Reference to global vehicle count.
 */
Aircraft::Aircraft(const std::string& fname, int& vehicle_count)
 : points(dummy_points),
      aircraft(nullptr),
      steps(10),
      clock(0.0),
      // Pre-initialize all runtime members to safe defaults
      pn(0), pe(0), pd(0),
      u(0), v(0), w(0),
      phi(0), theta(0), psi(0),
      p(0), q(0), r(0),
      V_m(0), alpha(0), beta(0),
      fx(0), fy(0), fz(0),
      ell(0), m(0), n(0)
{
        // Fully zero the state vector before use
        X.assign(18, 0.0);   

    //1. First it loads the aircraft parameters from the file using the following function
        load_a_plane(fname,vehicle_count);
    //2. Then it modifies the state values by setting them to the defaults imported from the aircraft parameter file.    
        clock = 0;
        pn = pn_0;
        pe = pe_0;
        pd = pd_0;
        u = u_0;
        v = v_0;
        w = w_0; 
        phi = phi_0;
        theta = theta_0;
        psi = psi_0;
        p = p_0;
        q = q_0;
        r = r_0;

        // Compute correct initial velocity magnitude
        V_m = std::sqrt(u*u + v*v + w*w);        

        alpha = alpha0;
        beta = beta0;
        /*delta_t = delta_t;
        delta_a = delta_a;
        delta_e = delta_e;
        delta_r = delta_r;*/

        // Fill the state vector x 
        X[0] = pn_0;
        X[1] = pe_0;
        X[2] = pd_0;
        X[3] = u_0;
        X[4] = v_0;
        X[5] = w_0; 
        X[6] = p_0;
        X[7] = q_0;
        X[8]= r_0;
        X[9] = phi_0;
        X[10] = theta_0;
        X[11] = psi_0;
        X[12] = fx;
        X[13] = fy;
        X[14] = fz;
        X[15] = ell;
        X[16] = m;
        X[17] = n;

        

}

/**
 * @brief Destructor for Aircraft class. Currently does minimal cleanup.
 */
Aircraft::~Aircraft() 
{
    
    //endwin(); // End ncurses stuff  -< Deprecated
}

/**
 * @brief Calculate total aerodynamic, gravitational, and propulsion forces.
 */
void Aircraft::calculate_forces()
{
    // Calculate velocity, alpha and beta (in body-frame)
    calculate_body_frame_velocity_and_angles();
   
    calculate_lift_drag_coefficients();
    
    // Sources of Forces
    // Component: Gravity
    force_g << -mass * g*std::sin(X[10]),mass * g*std::cos(X[10])*sin(X[9]),mass * g*std::cos(X[9])*cos(X[9]);
    
    
    // Component: Aerodynamic
    force_aero1 = 0.5 * rho * (V_m*V_m) * wing_area;

  
    force_aero2 <<(CxAlpha+CxqAlpha*((wing_chord)/(2*V_m))*X[7])+CxdeltaeAlpha*delta_e,
        C_Y_0+(C_Y_beta*beta)+(C_Y_p*((wing_span)/(2*V_m))*X[6])+(C_Y_r*((wing_span)/(2*V_m))*X[8])+(C_Y_delta_a*delta_a)+(C_Y_delta_r*delta_r),
        CzAlpha+(CzqAlpha*((wing_chord)/(2*V_m))*X[7])+CzdeltaeAlpha*delta_e;
    
    force_aero << force_aero1 * force_aero2;

    // Component: Propulsive
    force_prop1 = 0.5*rho*prop_area*C_prop;
    
    force_prop2 << ((k_motor*delta_t)*(k_motor*delta_t))-(V_m*V_m),0,0; 
    
    force_prop << force_prop1 * force_prop2; 

    // Total Force
    Force << force_g + force_aero + force_prop;  

    // Writing outputs for components of the Force
    fx = Force[0];
    fy = Force[1];
    fz = Force[2];

    X[12] = fx;
    X[13] = fy;
    X[14] = fz;

    
}
/**
 * @brief Compute body-frame airspeed, angle of attack, and sideslip.
 */
void Aircraft::calculate_body_frame_velocity_and_angles() 
{
    velocity_b = {X[3], X[4], X[5]};
    V_m = sqrt((velocity_b[0]*velocity_b[0]) + (velocity_b[1]*velocity_b[1]) + (velocity_b[2]*velocity_b[2]));
    alpha = std::atan2(velocity_b[2], velocity_b[0]);
    beta = std::asin(velocity_b[1] / V_m);
}

/**
 * @brief Compute lift and drag coefficients as functions of alpha and control inputs.
 */
void Aircraft::calculate_lift_drag_coefficients()
{
    // Lift/Drag coefficient calculations [ Cl and Cd ]
      // Cd(alpha)                                          //* <------SUSPECT
     Cd_of_alpha = C_D_p + ((C_L_0 + C_L_alpha*alpha)*(C_L_0 + C_L_alpha*alpha)/(M_PI*e*wing_aspect_ratio));
      
    // sigma(alpha)
     sigma_num = 1 + std::exp(-trans_rate*(alpha-alpha0)) + std::exp(trans_rate*(alpha+alpha0));
     sigma_den = (1 + std::exp(-trans_rate*(alpha-alpha0))) * (1 + std::exp(trans_rate*(alpha+alpha0)));
     sigma_of_alpha = sigma_num/sigma_den;
    
     // Cl of flat plate
      Cl_flat_plate = 2*(std::signbit(alpha) ? -1.0 : 1.0) * (std::sin(alpha) * std::sin(alpha)) * (std::cos(alpha));
     // Linear Cl
      Cl_linear = C_L_0 + C_L_alpha*alpha;
     // Combined Cl
      Cl_of_alpha = ((1-sigma_of_alpha) * (Cl_linear)) + (sigma_of_alpha * Cl_flat_plate);

    // Coefficients for X and Z directions
     CxAlpha = (-Cd_of_alpha*std::cos(alpha))+(Cl_of_alpha*std::sin(alpha));
     CxqAlpha = (-C_D_q*std::cos(alpha))+(C_L_q*std::sin(alpha));
     CxdeltaeAlpha = (-C_D_delta_e*std::cos(alpha))+(C_L_delta_e*std::sin(alpha));
     CzAlpha = (-Cd_of_alpha*std::sin(alpha))-(Cl_of_alpha*std::cos(alpha));
     CzqAlpha = (-C_D_q*std::sin(alpha))-(C_L_q*std::cos(alpha));
     CzdeltaeAlpha = (-C_D_delta_e*std::sin(alpha))-(C_L_delta_e*std::cos(alpha));

}

/**
 * @brief Calculate aerodynamic and propulsive moments (roll, pitch, yaw).
 */
 void Aircraft::calculate_moments()
 {
    // Moment/Torque calculations

    // Moment/Torque resulting from aerodynamic forces
    Aero_t1 = force_aero1;
    
   
    
    Aero_t2 << wing_span*(C_ell_0 + (C_ell_beta*beta) + (C_ell_p*((wing_span)/(2*V_m))*X[6]) + (C_ell_r*((wing_span)/(2*V_m))*X[8]) + (C_ell_delta_a*delta_a)+(C_ell_delta_r*delta_r)),
    wing_chord*(C_m_0 + (C_m_alpha*alpha) + (C_m_q*((wing_chord)/(2*V_m))*X[7]) + (C_m_delta_e*delta_e)),
    wing_span*(C_n_0 + (C_n_beta*beta) + (C_n_p*((wing_span)/(2*V_m))*X[6]) + (C_n_r*((wing_span)/(2*V_m))*X[8]) + (C_n_delta_a*delta_a)+(C_n_delta_r*delta_r));  
    

    Aero_torque = Aero_t1 * Aero_t2;
    
    // Moment/Torque due to propulsion system

    Prop_torque << -prop_thrust_coef * ((prop_omega*delta_t)*(prop_omega*delta_t)),0,0;
    
    
    // Total Moment/Torque
    Torque = Aero_torque + Prop_torque;
    
    
    // l m n - 3 components of the moment/torque
    ell = Torque[0]; // this is just "l", written this way for readability
    m = Torque[1];
    n = Torque[2];

    X[15] = ell;
    X[16] = m;
    X[17] = n;

 }



/**
 * @brief Runge–Kutta 4th order integration to update state vector.
 * @param X State vector containing position, velocity, attitude, forces & moments.
 * @param dt Time step (seconds).
 */
// X = {pn,pe,pd,u,v,w,p,q,r,phi,theta,psi,fx,fy,fz,ell,m,n}; 

void Aircraft::RK4(std::vector<double>& X,  double dt)
{
    // Initial states stored in temporary variables
    double pn_k1 = X[0];
    double pe_k1 = X[1];
    double pd_k1 = X[2];
    double u_k1 = X[3];
    double v_k1 = X[4];
    double w_k1 = X[5];
    double p_k1 = X[6];
    double q_k1 = X[7];
    double r_k1 = X[8];
    double phi_k1 = X[9];
    double theta_k1 = X[10];
    double psi_k1 = X[11];
    double fx_k1 = X[12];
    double fy_k1 = X[13];
    double fz_k1 = X[14];
    double ell_k1 = X[15];
    double m_k1 = X[16];
    double n_k1 = X[17];

    // Arrays to store k-values for each variable
    double k1[12], k2[12], k3[12], k4[12];

    // Step 1: Compute k1 for each variable [ k1 = h * f(x,y,z.....)]
    k1[0] = dt * calculate_pn_dot(u_k1,v_k1,w_k1,phi_k1,theta_k1,psi_k1);
    k1[1] = dt * calculate_pe_dot(u_k1,v_k1,w_k1,phi_k1,theta_k1,psi_k1);
    k1[2] = dt * calculate_pd_dot(u_k1,v_k1,w_k1,phi_k1,theta_k1);
    k1[3] = dt * calculate_u_dot(v_k1,w_k1,q_k1,r_k1,fx_k1,mass);
    k1[4] = dt * calculate_v_dot(u_k1,w_k1,p_k1,fy_k1,mass);
    k1[5] = dt * calculate_w_dot(u_k1,v_k1,p_k1,q_k1,fz_k1,mass);
    k1[6] = dt * calculate_p_dot(p_k1,q_k1,r_k1,ell_k1,n_k1,Gamma_1,Gamma_2,Gamma_3,Gamma_4);
    k1[7] = dt * calculate_q_dot(p_k1,r_k1,m_k1,Jy,Gamma_5,Gamma_6);
    k1[8] = dt * calculate_r_dot(p_k1,q_k1,r_k1,ell_k1,n_k1,Gamma_1,Gamma_4,Gamma_7,Gamma_8);
    k1[9] = dt * calculate_phi_dot(p_k1,q_k1,r_k1,phi_k1,theta_k1);
    k1[10] = dt * calculate_theta_dot(q_k1,r_k1,phi_k1);
    k1[11] = dt * calculate_psi_dot(q_k1,r_k1,phi_k1,theta_k1);

    // Step 2: Compute k2 for each variable using updated intermediate values
    double pn_k2 = pn_k1 + 0.5 * k1[0];
    double pe_k2 = pe_k1 + 0.5 * k1[1];
    double pd_k2 = pd_k1 + 0.5 * k1[2];
    double u_k2 = u_k1 + 0.5 * k1[3];
    double v_k2 = v_k1 + 0.5 * k1[4];
    double w_k2 = w_k1 + 0.5 * k1[5];
    double p_k2 = p_k1 + 0.5 * k1[6];
    double q_k2 = q_k1 + 0.5 * k1[7];
    double r_k2 = r_k1 + 0.5 * k1[8];
    double phi_k2 = phi_k1 + 0.5 * k1[9];
    double theta_k2 = theta_k1 + 0.5 * k1[10];
    double psi_k2 = psi_k1 + 0.5 * k1[11];
    double fx_k2 = fx_k1 + 0.5 * k1[12];
    double fy_k2 = fy_k1 + 0.5 * k1[13];
    double fz_k2 = fz_k1 + 0.5 * k1[14];
    double ell_k2 = ell_k1 + 0.5 * k1[15];
    double m_k2 = m_k1 + 0.5 * k1[16];
    double n_k2 = n_k1 + 0.5 * k1[17];

    // Compute k2 for each variable [ k2 = h * f(x+0.5*k1, y+0.5*k1,z+0.5*k1.....)]
    k2[0] = dt * calculate_pn_dot(u_k2, v_k2, w_k2, phi_k2, theta_k2, psi_k2);
    k2[1] = dt * calculate_pe_dot(u_k2, v_k2, w_k2, phi_k2, theta_k2, psi_k2);
    k2[2] = dt * calculate_pd_dot(u_k2, v_k2, w_k2, phi_k2, theta_k2);
    k2[3] = dt * calculate_u_dot(v_k2, w_k2, q_k2, r_k2, fx_k2, mass);
    k2[4] = dt * calculate_v_dot(u_k2, w_k2, p_k2, fy_k2, mass);
    k2[5] = dt * calculate_w_dot(u_k2, v_k2, p_k2, q_k2, fz_k2, mass);
    k2[6] = dt * calculate_p_dot(p_k2, q_k2, r_k2, ell_k2, n_k2, Gamma_1, Gamma_2, Gamma_3, Gamma_4);
    k2[7] = dt * calculate_q_dot(p_k2, r_k2, m_k2, Jy, Gamma_5, Gamma_6);
    k2[8] = dt * calculate_r_dot(p_k2, q_k2, r_k2, ell_k2, n_k2, Gamma_1, Gamma_4, Gamma_7, Gamma_8);
    k2[9] = dt * calculate_phi_dot(p_k2, q_k2, r_k2, phi_k2, theta_k2);
    k2[10] = dt * calculate_theta_dot(q_k2, r_k2, phi_k2);
    k2[11] = dt * calculate_psi_dot(q_k2, r_k2, phi_k2, theta_k2);

    // Step 2: Compute k3 for each variable using updated intermediate values from k2
    double pn_k3 = pn_k1 + 0.5 * k2[0];
    double pe_k3 = pe_k1 + 0.5 * k2[1];
    double pd_k3 = pd_k1 + 0.5 * k2[2];
    double u_k3 = u_k1 + 0.5 * k2[3];
    double v_k3 = v_k1 + 0.5 * k2[4];
    double w_k3 = w_k1 + 0.5 * k2[5];
    double p_k3 = p_k1 + 0.5 * k2[6];
    double q_k3 = q_k1 + 0.5 * k2[7];
    double r_k3 = r_k1 + 0.5 * k2[8];
    double phi_k3 = phi_k1 + 0.5 * k2[9];
    double theta_k3 = theta_k1 + 0.5 * k2[10];
    double psi_k3 = psi_k1 + 0.5 * k2[11];
    double fx_k3 = fx_k1 + 0.5 * k2[12];
    double fy_k3 = fy_k1 + 0.5 * k2[13];
    double fz_k3 = fz_k1 + 0.5 * k2[14];
    double ell_k3 = ell_k1 + 0.5 * k2[15];
    double m_k3 = m_k1 + 0.5 * k2[16];
    double n_k3 = n_k1 + 0.5 * k2[17];

    // Calculate k3 for each variable using the intermediate state values  [ k3 = h * f(x+0.5*k2, y+0.5*k2,z+0.5*k2.....)]
    k3[0] = dt * calculate_pn_dot(u_k3, v_k3, w_k3, phi_k3, theta_k3, psi_k3);
    k3[1] = dt * calculate_pe_dot(u_k3, v_k3, w_k3, phi_k3, theta_k3, psi_k3);
    k3[2] = dt * calculate_pd_dot(u_k3, v_k3, w_k3, phi_k3, theta_k3);
    k3[3] = dt * calculate_u_dot(v_k3, w_k3, q_k3, r_k3, fx_k3, mass);
    k3[4] = dt * calculate_v_dot(u_k3, w_k3, p_k3, fy_k3, mass);
    k3[5] = dt * calculate_w_dot(u_k3, v_k3, p_k3, q_k3, fz_k3, mass);
    k3[6] = dt * calculate_p_dot(p_k3, q_k3, r_k3, ell_k3, n_k3, Gamma_1, Gamma_2, Gamma_3, Gamma_4);
    k3[7] = dt * calculate_q_dot(p_k3, r_k3, m_k3, Jy, Gamma_5, Gamma_6);
    k3[8] = dt * calculate_r_dot(p_k3, q_k3, r_k3, ell_k3, n_k3, Gamma_1, Gamma_4, Gamma_7, Gamma_8);
    k3[9] = dt * calculate_phi_dot(p_k3, q_k3, r_k3, phi_k3, theta_k3);
    k3[10] = dt * calculate_theta_dot(q_k3, r_k3, phi_k3);
    k3[11] = dt * calculate_psi_dot(q_k3, r_k3, phi_k3, theta_k3);

    // Step 4: Compute k4 for each variable using updated intermediate values from k3 
    double pn_k4 = pn_k1 + k3[0];
    double pe_k4 = pe_k1 + k3[1];
    double pd_k4 = pd_k1 + k3[2];
    double u_k4 = u_k1 + k3[3];
    double v_k4 = v_k1 + k3[4];
    double w_k4 = w_k1 + k3[5];
    double p_k4 = p_k1 + k3[6];
    double q_k4 = q_k1 + k3[7];
    double r_k4 = r_k1 + k3[8];
    double phi_k4 = phi_k1 + k3[9];
    double theta_k4 = theta_k1 + k3[10];
    double psi_k4 = psi_k1 + k3[11];
    double fx_k4 = fx_k1 + k3[12];
    double fy_k4 = fy_k1 + k3[13];
    double fz_k4 = fz_k1 + k3[14];
    double ell_k4 = ell_k1 + k3[15];
    double m_k4 = m_k1 + k3[16];
    double n_k4 = n_k1 + k3[17];

    // Calculating k4 values for each variable   [ k4 = h * f(x+k3, y+k3,z+k3.....)]
    k4[0] = dt * calculate_pn_dot(u_k4, v_k4, w_k4, phi_k4, theta_k4, psi_k4);
    k4[1] = dt * calculate_pe_dot(u_k4, v_k4, w_k4, phi_k4, theta_k4, psi_k4);
    k4[2] = dt * calculate_pd_dot(u_k4, v_k4, w_k4, phi_k4, theta_k4);
    k4[3] = dt * calculate_u_dot(v_k4, w_k4, q_k4, r_k4, fx_k4, mass);
    k4[4] = dt * calculate_v_dot(u_k4, w_k4, p_k4, fy_k4, mass);
    k4[5] = dt * calculate_w_dot(u_k4, v_k4, p_k4, q_k4, fz_k4, mass);
    k4[6] = dt * calculate_p_dot(p_k4, q_k4, r_k4, ell_k4, n_k4, Gamma_1, Gamma_2, Gamma_3, Gamma_4);
    k4[7] = dt * calculate_q_dot(p_k4, r_k4, m_k4, Jy, Gamma_5, Gamma_6);
    k4[8] = dt * calculate_r_dot(p_k4, q_k4, r_k4, ell_k4, n_k4, Gamma_1, Gamma_4, Gamma_7, Gamma_8);
    k4[9] = dt * calculate_phi_dot(p_k4, q_k4, r_k4, phi_k4, theta_k4);
    k4[10] = dt * calculate_theta_dot(q_k4, r_k4, phi_k4);
    k4[11] = dt * calculate_psi_dot(q_k4, r_k4, phi_k4, theta_k4);

    // Update each state variable with RK4 final formula
    // X_new​ = X_old + (1/6)*(k1+2*k2+2*k3+k4)

    X[0] += (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]) / 6; // pn
    X[1] += (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]) / 6; // pe
    X[2] += (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2]) / 6; // pd
    X[3] += (k1[3] + 2 * k2[3] + 2 * k3[3] + k4[3]) / 6; // u
    X[4] += (k1[4] + 2 * k2[4] + 2 * k3[4] + k4[4]) / 6; // v
    X[5] += (k1[5] + 2 * k2[5] + 2 * k3[5] + k4[5]) / 6; // w
    X[6] += (k1[6] + 2 * k2[6] + 2 * k3[6] + k4[6]) / 6; // p
    X[7] += (k1[7] + 2 * k2[7] + 2 * k3[7] + k4[7]) / 6; // q
    X[8] += (k1[8] + 2 * k2[8] + 2 * k3[8] + k4[8]) / 6; // r
    X[9] += (k1[9] + 2 * k2[9] + 2 * k3[9] + k4[9]) / 6; // phi
    X[10] += (k1[10] + 2 * k2[10] + 2 * k3[10] + k4[10]) / 6; // theta
    X[11] += (k1[11] + 2 * k2[11] + 2 * k3[11] + k4[11]) / 6; // psi
    

    // After updating, these values represent the state at t + dt (fingers crossed)
}

/**
 * @brief Compute derivative of north position (pn_dot).
 */
double Aircraft::calculate_pn_dot(double& u, double& v, double& w, double& phi, double& theta, double& psi)
{
    pn_dot = 
    u*std::cos(psi)*std::cos(theta) 
    + v*(std::cos(psi)*std::sin(phi)*std::sin(theta) - std::cos(phi)*std::sin(psi)) 
    + w*(std::sin(phi)*std::sin(psi) + std::cos(phi)*std::cos(psi)*std::sin(theta)) ;
    
    return pn_dot;
}
/**
 * @brief Compute derivative of east position (pe_dot).
 */
double Aircraft::calculate_pe_dot(double& u, double& v, double& w, double& phi, double& theta, double& psi)
{

    pe_dot = 
    u*std::cos(theta)*std::sin(psi) 
    + v*(std::cos(phi)*std::cos(psi) + std::sin(phi)*std::sin(psi)*std::sin(theta)) 
    + w*(std::cos(phi)*std::sin(psi)*std::sin(theta) - std::cos(psi)*std::sin(phi));

    return pe_dot;

}
/**
 * @brief Compute derivative of down position (pd_dot).
 */
double Aircraft::calculate_pd_dot(double& u, double& v, double& w, double& phi, double& theta)
{
    pd_dot = 
    - u*std::sin(theta) 
    + v*std::cos(theta)*std::sin(phi) 
    + w*std::cos(phi)*std::cos(theta) ;

    return pd_dot;
}


/**
 * @brief Compute derivative of roll angle (phi_dot).
 */
double Aircraft::calculate_phi_dot(double& p, double& q, double& r, double& phi, double& theta)
{
    phi_dot = p + r*std::cos(phi)*std::tan(theta) + q*std::sin(phi)*std::tan(theta);
    
    return phi_dot;
}
/**
 * @brief Compute derivative of pitch angle (theta_dot).
 */
double Aircraft::calculate_theta_dot(double& q, double& r,double& phi)
{
   theta_dot = q*std::cos(phi) - r*std::sin(phi);
   
   return theta_dot;
}
/**
 * @brief Compute derivative of yaw angle (psi_dot).
 */
double Aircraft::calculate_psi_dot(double& q, double& r, double& phi, double& theta)
{
    psi_dot = (r*std::cos(phi))/std::cos(theta) + (q*std::sin(phi))/std::cos(theta);
   
    return psi_dot;
}

/**
 * @brief Compute derivative of body-axis X velocity.
 */
double Aircraft::calculate_u_dot(double& v, double& w, double& q, double& r, double& fx, double& mass)
{   
     u_dot = (r*v - q*w)+(fx/mass);

     return u_dot;
} 
/**
 * @brief Compute derivative of body-axis Y velocity.
 */
double Aircraft::calculate_v_dot(double& u, double& w, double&p,double& fy, double& mass)
{   
     v_dot = (p*w - r*u)+(fy/mass);
     
     return v_dot;
} 
/**
 * @brief Compute derivative of body-axis Z velocity.
 */
double Aircraft::calculate_w_dot(double& u, double& v, double& p,  double& q,double& fz, double& mass)
{  
     w_dot = (q*u - p*v)+(fz/mass);

     return w_dot;
} 

/**
 * @brief Compute derivative of roll rate p.
 */
double Aircraft::calculate_p_dot(double& p, double& q, double& r,  double& ell,double& n, double& Gamma_1,double& Gamma_2,double& Gamma_3,double& Gamma_4)
{
    p_dot = Gamma_1*p*q - Gamma_2*q*r + Gamma_3*ell + Gamma_4*n;
 
    return p_dot;
}

/**
 * @brief Compute derivative of pitch rate q.
 */
double Aircraft::calculate_q_dot(double& p, double& r,  double& m ,double& Jy, double& Gamma_5,double& Gamma_6)
{
    q_dot = Gamma_5*p*r - Gamma_6*((p*p)-(r*r)) + (m/Jy); 

    return q_dot;    
}
/**
 * @brief Compute derivative of yaw rate r.
 */
double Aircraft::calculate_r_dot(double& p, double& q, double& r,  double& ell,double& n, double& Gamma_1,double& Gamma_4,double& Gamma_7,double& Gamma_8)
{ 
    r_dot = Gamma_7*p*q - Gamma_1*q*r + Gamma_4*ell + Gamma_8*n;

    return r_dot;
}


/**
 * @brief Apply rotation to aircraft mesh vertices using Euler angles.
 */
void Aircraft::rotate(easy3d::vec3* vertices)
{
    // Create the rotation matrix using Euler angles
    easy3d::Mat3<float> rotationMatrix = easy3d::Mat3<float>::rotation(X[9], X[10] , X[11], 321);
        
    // Apply the rotation to the vertices and hope that it actually works
        for (int i = 0; i < mesh->n_vertices(); ++i)
        {
            vertices[i] = rotationMatrix * vertices[i];
        }

}

/**
 * @brief Apply rotation to local axes vertices.
 */
void Aircraft::rotate_axes(easy3d::vec3* axesVertices)
{
    // Create the rotation matrix using Euler angles (same rotation as aircraft)
    easy3d::Mat3<float> rotationMatrix = easy3d::Mat3<float>::rotation(X[9], X[10] , X[11], 321);
    
    // Assuming we have 6 vertices for the 3 axes (X, Y, Z), rotate them
    for (int i = 0; i < 6; ++i) {
        axesVertices[i] = rotationMatrix * axesVertices[i];
    }
}


/**
 * @brief Apply translation to aircraft mesh vertices.
 */
void Aircraft::translate(easy3d::vec3* vertices)
{
    // pd (X[2]) here is set to negative as pd = - altitude
    easy3d::vec3 translationVector(static_cast<float>(X[0]), static_cast<float>(X[1]), static_cast<float>(-X[2]));


// Position Update loop

            for (int j=0; j<mesh->n_vertices(); ++j)
            {
                vertices[j] -= translationVector;  // Apply translation
            }    

    
}

/**
 * @brief Apply translation to axes vertices.
 */
void Aircraft::translate_axes(easy3d::vec3* axesVertices)
{
    // Use the aircraft's position (pn, pe, pd) as the translation vector
    easy3d::vec3 translationVector(static_cast<float>(X[0]), static_cast<float>(X[1]), static_cast<float>(-X[2]));

    // Apply the translation to each vertex (assuming 6 vertices for the axes)
    for (int i = 0; i < 6; ++i) {
        axesVertices[i] -= translationVector;
    }
}

/**
 * @brief Load and render aircraft geometry using Easy3D.
 * @param viewer Reference to Easy3D viewer.
 */
void Aircraft::renderAircraft(easy3d::Viewer& viewer)
{
    

    if (!mesh) {
        std::cerr << "Failed to load 3D model. Please check the file path and format." << std::endl;
        exit(-1);
    }

    std::cout << "Mesh loaded successfully." << std::endl;
    std::cout << "\tVertices: " << mesh->n_vertices() << std::endl;
    std::cout << "\tEdges: " << mesh->n_edges() << std::endl;
    std::cout << "\tFaces: " << mesh->n_faces() << std::endl;

 
    // Add the mesh as a drawable object in the viewer
    aircraft = new easy3d::TrianglesDrawable("faces");
    // Update drawable with the mesh's vertices
    
    for (auto v : mesh->vertices()) 
    {
        easy3d::vec3 original_pos = mesh->position(v);
        vertices_aircraft.push_back(mesh->position(v));
        original_vertices.push_back(original_pos); // storing a clean copy
    }
    aircraft->update_vertex_buffer(vertices_aircraft);

    // Update drawable with the mesh's faces
    
    for (auto f : mesh->faces()) 
    {
        for (auto v : mesh->vertices(f)) {
            indices.push_back(v.idx());
        }
    }
    aircraft->update_element_buffer(indices);

    aircraft->set_uniform_coloring(easy3d::vec4(1.0f, 1.0f, 1.0f, 1.0f));
    

    // Add the drawable to the viewer
    viewer.add_drawable(aircraft);

    viewer.update();


}


/**
 * @brief Create and render a local coordinate frame.
 * @param viewer Reference to Easy3D viewer.
 */
void Aircraft::createAxesDrawable(easy3d::Viewer& viewer)
{   
    // Create a LinesDrawable to visualize the 3D axes.
    axesDrawable = new easy3d::LinesDrawable("axes");

    // Define the vertices for the three axes.
    axes_vertices = 
    {
    // X-axis 
    easy3d::vec3(X[9], X[10] , X[11]), // Origin
    easy3d::vec3(X[9] - 50.0f,  X[10] , X[11]), // X-axis endpoint (moving in negative x-direction)
    
    // Y-axis 
    easy3d::vec3(X[9], X[10] , X[11]), // Origin
    easy3d::vec3(X[9], X[9] + 50.0f, X[11]), // Y-axis endpoint (moving in positive y-direction)
    
    // Z-axis
    easy3d::vec3(X[9], X[10] , X[11]), // Origin
    easy3d::vec3(X[9], X[10] , X[11] - 50.0f)  // Z-axis endpoint (moving downward in negative z-direction)
    };

    axes_vertices_original = axes_vertices;  // To avoid compound transformatin

    // Upload the axes vertices to the GPU.
    axesDrawable->update_vertex_buffer(axes_vertices);
    
    // Set color
    axesDrawable->set_uniform_coloring(easy3d::vec4(1.0f, 0.0f, 0.0f, 1.0f)); //Red
   
    // Set the width of the axes lines (here 3 pixels).
    axesDrawable->set_line_width(1.0f);

    // Add the axes drawable to the viewer.
    viewer.add_drawable(axesDrawable);

    // Update the viewer.
    viewer.update();
}



/**
 * @brief Update aircraft state each simulation step, applying dynamics and geometry transforms.
 * @param vertices Vertex buffer for aircraft.
 * @param axesVertices Vertex buffer for axes.
 * @param dt Time step in seconds.
 * @return Pointer to updated aircraft vertex buffer.
 */
easy3d::vec3* Aircraft::update_aircraft(easy3d::vec3* vertices, easy3d::vec3* axesVertices,double& dt)
{
    // Calculate forces and moments
    
    calculate_forces();
    calculate_moments();

    // Update dynamics
    RK4(X,dt);

    // Start with fresh copy of aircraft and axes vertices
    // to avoid compound transformations
    for (size_t i = 0; i < original_vertices.size(); ++i)
    {
        vertices[i] = original_vertices[i];
    }
    for (size_t i = 0; i < axes_vertices_original.size(); ++i)
    {
        axesVertices[i] = axes_vertices_original[i];
    }

    // Perform rotation and translation on the geometry
    rotate(vertices);
    translate(vertices);
    rotate_axes(axesVertices);
    translate_axes(axesVertices);


    //printState();


    return vertices;
    
}
/**
 * @brief Animation callback called by viewer per frame.
 * @param viewer Pointer to Easy3D viewer.
 * @param dt Time step in seconds.
 * @return True if updated successfully.
 */
bool Aircraft::animate(easy3d::Viewer* viewer,double dt)
{
    (void)viewer;  
    // FOR THE UAV
    // Map the vertex buffer into the client's address space
    void* aircraftPointer = easy3d::VertexArrayObject::map_buffer(GL_ARRAY_BUFFER, aircraft->vertex_buffer(), GL_WRITE_ONLY);

    easy3d::vec3* vertices = reinterpret_cast<easy3d::vec3*>(aircraftPointer);
    if (!vertices) 
    {
        return false;
    }

    
    // Unmap the vertex buffer
    easy3d::VertexArrayObject::unmap_buffer(GL_ARRAY_BUFFER, aircraft->vertex_buffer());


    // FOR THE UAV's body axes

    // Map the vertex buffer for the axes
    void* axesPointer = easy3d::VertexArrayObject::map_buffer(GL_ARRAY_BUFFER, axesDrawable->vertex_buffer(), GL_WRITE_ONLY);
    
    easy3d::vec3* axesVertices = reinterpret_cast<easy3d::vec3*>(axesPointer);
    if (!axesVertices) {
        return false;
    }

           
    // Unmap the vertex buffer
    easy3d::VertexArrayObject::unmap_buffer(GL_ARRAY_BUFFER, axesDrawable->vertex_buffer());

    // Update UAV per cycle of the loop [Also updates axes]
    update_aircraft(vertices, axesVertices, dt);



    // Update the viewer
    viewer->update();

    return true;

}


/**
 * @brief Debug print of current state variables to console.
 */
void Aircraft::printState()
{
    static int frameCount = 0;
    frameCount++;
    std::cout << "State Variables: << Frame: "<<frameCount<<"\n";
    std::cout << "-----------------\n";
    std::cout << "pn    (North position): " << X[0] << "\n";
    std::cout << "pe    (East position):  " << X[1] << "\n";
    std::cout << "pd    (Down position):  " << X[2] << "\n";
    std::cout << "u     (Body x-velocity): " << X[3] << "\n";
    std::cout << "v     (Body y-velocity): " << X[4] << "\n";
    std::cout << "w     (Body z-velocity): " << X[5] << "\n";
    std::cout << "p     (Roll rate):       " << X[6] << "\n";
    std::cout << "q     (Pitch rate):      " << X[7] << "\n";
    std::cout << "r     (Yaw rate):        " << X[8] << "\n";
    std::cout << "phi   (Roll angle):      " << X[9]*180/M_PI << "\n";
    std::cout << "theta (Pitch angle):     " << X[10]*180/M_PI << "\n";
    std::cout << "psi   (Yaw angle):       " << X[11] *180/M_PI<< "\n";
    std::cout << "Fx    (X-Force):         " << X[12]<<"\n";
    std::cout << "Fy    (Y-Force):         " << X[13]<<"\n";
    std::cout << "Fz    (Z-Force):         " << X[14]<<"\n";
}

/**
 * @brief Render heads-up display (HUD) with aircraft states.
 * @param tr Reference to Easy3D TextRenderer.
 * @param viewer Pointer to Easy3D viewer.
 */
void Aircraft::render_HUD(easy3d::TextRenderer& tr, easy3d::Viewer* viewer) const
{
    int width = viewer->width();
    int height = viewer->height();

    // Format key variables into a multi-line string
    std::ostringstream hud_stream;
    hud_stream << std::fixed << std::setprecision(2);
    hud_stream << std::left;
    hud_stream << std::setw(28) << "Clock:"                     << clock << " s\n";
    hud_stream << std::setw(28) << "Pos (N, E, D) [m]:"         << X[0] << ", " << X[1] << ", " << X[2] << "\n";
    hud_stream << std::setw(28) << "Vel (u, v, w) [m/s]:"       << X[3] << ", " << X[4] << ", " << X[5] << "\n";
    hud_stream << std::setw(28) << "Angular Rates (p, q, r) [deg/s]:"
           << X[6] << ", " << X[7] << ", " << X[8] << "\n";
    hud_stream << std::setw(28) << "Euler Angles (phi, theta, psi) [deg]:"
           << X[9] * 180 / M_PI << ", "
           << X[10] * 180 / M_PI << ", "
           << X[11] * 180 / M_PI << "\n";
    hud_stream << std::setw(28) << "Air Data (V, alpha, beta):"
           << V_m << ", " << alpha << ", " << beta << "\n";
    hud_stream << std::setw(28) << "Forces (fx, fy, fz) [N]:"
           << X[12] << ", " << X[13] << ", " << X[14] << "\n";
    hud_stream << std::setw(28) << "Moments (ell, m, n) [Nm]:"
            << X[15] << ", " << X[16] << ", " << X[17];

    std::string hud_text = hud_stream.str();

    // Set font and color
    float font_size = 16.0f;
    int font_id = 0;
    easy3d::vec3 color(1.0f, 1.0f, 1.0f);  // White

    // Draw HUD on screen (bottom-right)
    tr.draw(hud_text,
            width - 410,          // x position
            height -150,         // y position
            font_size,
            easy3d::TextRenderer::ALIGN_LEFT,
            font_id,
            color,
            0.0f,                 // Line spacing
            true);                // Origin at upper-left
}

/**
 * @brief Compute control inputs (δ_e, δ_a, δ_r, δ_t) that achieve trim.
 * @param alpha Angle of attack.
 * @param beta Side slip angle.
 * @param phi Roll angle.
 * @param p Roll rate.
 * @param q Pitch rate.
 * @param r Yaw rate.
 * @param theta Pitch angle.
 * @param Va Airspeed.
 * @param R Turn radius.
 * @return Struct containing trim control inputs.
 */
Aircraft::ControlInputs Aircraft::computeTrimControls(
    double alpha, double beta, double phi,
    double p, double q, double r,
    double theta, double Va, double R)
{
    Aircraft::ControlInputs controls;

    // Constants for convenience
    const double qbar = 0.5 * rho * Va * Va; // Dynamic pressure
    const double c = wing_chord;
    const double b = wing_span;
    const double S = wing_area;
    const double g_ = g;

    // ----- Elevator (δ_e) from eq (F.1) -----
    // Note: may need to replace or adjust terms according to aircraft model

    // Moment terms from inertia matrix
    double moment_term = (Jxz * (p*p - r*r) + (Jx - Jz) * p * r) / (qbar * c * S);

    // Calculate delta_e
    double numerator = moment_term - C_m_0 - C_m_alpha * alpha - C_m_q * (c * q) / (2.0 * Va);
    double denominator = C_m_delta_e;

    controls.delta_e = numerator / denominator;

    // Clamp to limits
    controls.delta_e = std::clamp(controls.delta_e, delta_e_min, delta_e_max);

   // ----- Throttle (δ_t) from eq (F.2) with  prop model -----

    // Make sure coefficients are up to date
    calculate_lift_drag_coefficients();

    double C_X = CxAlpha
            + CxqAlpha * (c * q) / (2.0 * Va)
            + CxdeltaeAlpha * controls.delta_e;
    if (std::abs(C_X) < 1e-6) C_X = 1e-6;



    // Required net force in X-direction (positive forward)
    double Fx_required = mass * (-r * v + q * w + g * std::sin(theta));  // from book

    // Aerodynamic contribution
    double Fx_aero = qbar * S * C_X;

    // Solve for δ_t using your prop model(I am using the book's but will change it later TODO)
    double prop_term = Fx_required - Fx_aero;
    double delta_t_sq = (Va * Va + (2.0 * prop_term) / (rho * prop_area * C_prop)) / (k_motor * k_motor);

    // Ensure non-negative
    if (delta_t_sq < 0.0) delta_t_sq = 0.0;

    controls.delta_t = std::sqrt(delta_t_sq);

    // Clamp to limits
    controls.delta_t = std::clamp(controls.delta_t, delta_t_min, delta_t_max);



    // ----- Aileron (δ_a) and Rudder (δ_r) from eq (F.3) -----

    Eigen::Matrix2d control_matrix;
    control_matrix << C_p_delta_a, C_p_delta_r,
                    C_r_delta_a, C_r_delta_r;

    double half_rho_Va2Sb = 0.5 * rho * Va * Va * S * b;

    Eigen::Vector2d rhs;

    // Roll moment balance
    rhs(0) = -( C_p_0
            + C_p_beta * beta
            + C_p_p * (b * p) / (2.0 * Va)
            + C_p_r * (b * r) / (2.0 * Va) ) / half_rho_Va2Sb;

    // Yaw moment balance
    rhs(1) = -( C_r_0
            + C_r_beta * beta
            + C_r_p * (b * p) / (2.0 * Va)
            + C_r_r * (b * r) / (2.0 * Va) ) / half_rho_Va2Sb;

    Eigen::Vector2d delta_controls = control_matrix.inverse() * rhs;

    controls.delta_a = std::clamp(delta_controls(0), delta_a_min, delta_a_max);
    controls.delta_r = std::clamp(delta_controls(1), delta_r_min, delta_r_max);


    return controls;
}

Eigen::VectorXd Aircraft::getLateralState() const 
{
    double u = X[3];
    double v = X[4];
    double w = X[5];
    double Va = std::sqrt(u*u + v*v + w*w);
    double beta = std::asin(v / Va);

    Eigen::VectorXd lat(5);
    lat << beta,
           X[9],  // p
           X[11], // r
           X[6],  // φ
           X[8];  // ψ
    return lat;
}

Eigen::VectorXd Aircraft::getLongitudinalState() const 
{
    double h = -X[2]; // altitude from down position
    Eigen::VectorXd lon(5);
    lon << X[3],  // u;
           X[5],  // w
           X[10], // q
           X[7],  // θ
           h;
    return lon;
}

Eigen::VectorXd Aircraft::computeStateDot(const Eigen::VectorXd& X_in,
                                          const Eigen::VectorXd& U_in)
{
    // Save current state and controls
    std::vector<double> oldX = this->X;
    double old_de = this->delta_e;
    double old_da = this->delta_a;
    double old_dr = this->delta_r;
    double old_dt = this->delta_t;

    // Apply perturbed state
    for (size_t i = 0; i < X_in.size(); ++i)
        this->X[i] = X_in[i];

    // Apply perturbed controls
    this->delta_e = U_in[0];
    this->delta_a = U_in[1];
    this->delta_r = U_in[2];
    this->delta_t = U_in[3];

    // Update forces and moments for this perturbed state
    calculate_forces();
    calculate_moments();

    // Unpack state variables (18-element layout)
    double pn    = X[0];
    double pe    = X[1];
    double pd    = X[2];
    double u     = X[3];
    double v     = X[4];
    double w     = X[5];
    double p     = X[6];
    double q     = X[7];
    double r     = X[8];
    double phi   = X[9];
    double theta = X[10];
    double psi   = X[11];
    double fx    = X[12];
    double fy    = X[13];
    double fz    = X[14];
    double ell   = X[15];
    double m     = X[16];
    double n     = X[17];

    // Allocate derivative vector for 12 dynamics states
    Eigen::VectorXd Xdot(12);

    Xdot[0]  = calculate_pn_dot(u, v, w, phi, theta, psi);
    Xdot[1]  = calculate_pe_dot(u, v, w, phi, theta, psi);
    Xdot[2]  = calculate_pd_dot(u, v, w, phi, theta);
    Xdot[3]  = calculate_u_dot(v, w, q, r, fx, mass);
    Xdot[4]  = calculate_v_dot(u, w, p, fy, mass);
    Xdot[5]  = calculate_w_dot(u, v, p, q, fz, mass);
    Xdot[6]  = calculate_p_dot(p, q, r, ell, n, Gamma_1, Gamma_2, Gamma_3, Gamma_4);
    Xdot[7]  = calculate_q_dot(p, r, m, Jy, Gamma_5, Gamma_6);
    Xdot[8]  = calculate_r_dot(p, q, r, ell, n, Gamma_1, Gamma_4, Gamma_7, Gamma_8);
    Xdot[9]  = calculate_phi_dot(p, q, r, phi, theta);
    Xdot[10] = calculate_theta_dot(q, r, phi);
    Xdot[11] = calculate_psi_dot(q, r, phi, theta);

    // Restore original state and controls
    this->X = oldX;
    this->delta_e = old_de;
    this->delta_a = old_da;
    this->delta_r = old_dr;
    this->delta_t = old_dt;

    return Xdot;
}









