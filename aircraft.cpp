// Implementation file for Aircraft class
#include "simulator.h"
// Aircraft Member Functions:


void Aircraft::load_a_plane(const std::string& filePath, int& vehicle_count) 
{
            
    int id = vehicle_count;
             
           
    
    std::ifstream file(filePath);
    if (file.is_open()) {
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
        std::cout << "Aircraft loaded from file successfully" << "\n";
        std::cout << "Vehicle has been given the ID number: "<< vehicle_count <<"\n";
        vehicle_count++;
    }
    else {
        std::cerr << "Unable to open file: " << filePath << "\n";
    }
}




// Class constructor
Aircraft::Aircraft(const std::string& fname, int& vehicle_count)
:
size(1000.0f),
numLines(22),
offset(500.0f),//10000.0f/2
points(dummy_points),
aircraft(nullptr),
gridDrawable(nullptr),
steps(10)
{
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
        V_m = u_0; // SUSPECT
        alpha = alpha0;
        beta = beta0;
        delta_t = delta_t;
        delta_a = delta_a;
        delta_e = delta_e;
        delta_r = delta_r;

        //Added after RK4
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
// Class Destructor
Aircraft::~Aircraft() 
{
    
    endwin(); // End ncurses stuff
}
// Function to calculate the forces and moments acting on the aircraft 
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
// Function to calculate the body-frame velocity, angle of attack and side slip
void Aircraft::calculate_body_frame_velocity_and_angles() 
{
    velocity_b = {X[3], X[4], X[5]};
    V_m = sqrt((velocity_b[0]*velocity_b[0]) + (velocity_b[1]*velocity_b[1]) + (velocity_b[2]*velocity_b[2]));
    alpha = std::atan2(velocity_b[2], velocity_b[0]);
    beta = std::asin(velocity_b[1] / V_m);
}

// Function to calculate the lift and drag related coeefficients
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

// Function to calculate the moments acting on the UAV
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



// Function to apply RK4 on state vector
// Takes in a vector of state variables, dt (step size)
// Computes approximate for state variables at the end of the time step 
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

// Function to calculate pn_dot
double Aircraft::calculate_pn_dot(double& u, double& v, double& w, double& phi, double& theta, double& psi)
{
    pn_dot = 
    u*std::cos(psi)*std::cos(theta) 
    + v*(std::cos(psi)*std::sin(phi)*std::sin(theta) - std::cos(phi)*std::sin(psi)) 
    + w*(std::sin(phi)*std::sin(psi) + std::cos(phi)*std::cos(psi)*std::sin(theta)) ;
    
    return pn_dot;
}
//Function to calculate pe_dot
double Aircraft::calculate_pe_dot(double& u, double& v, double& w, double& phi, double& theta, double& psi)
{

    pe_dot = 
    u*std::cos(theta)*std::sin(psi) 
    + v*(std::cos(phi)*std::cos(psi) + std::sin(phi)*std::sin(psi)*std::sin(theta)) 
    + w*(std::cos(phi)*std::sin(psi)*std::sin(theta) - std::cos(psi)*std::sin(phi));

    return pe_dot;

}
// Function to calculate pd_dot
double Aircraft::calculate_pd_dot(double& u, double& v, double& w, double& phi, double& theta)
{
    pd_dot = 
    - u*std::sin(theta) 
    + v*std::cos(theta)*std::sin(phi) 
    + w*std::cos(phi)*std::cos(theta) ;

    return pd_dot;
}


// Function to calculate phi_dot
double Aircraft::calculate_phi_dot(double& p, double& q, double& r, double& phi, double& theta)
{
    phi_dot = p + r*std::cos(phi)*std::tan(theta) + q*std::sin(phi)*std::tan(theta);
    
    return phi_dot;
}
// Function to calculate theta_dot
double Aircraft::calculate_theta_dot(double& q, double& r,double& phi)
{
   theta_dot = q*std::cos(phi) - r*std::sin(phi);
   
   return theta_dot;
}
// Function to calculate psi_dot
double Aircraft::calculate_psi_dot(double& q, double& r, double& phi, double& theta)
{
    psi_dot = (r*std::cos(phi))/std::cos(theta) + (q*std::sin(phi))/std::cos(theta);
   
    return psi_dot;
}

// Function to calculate u_dot
double Aircraft::calculate_u_dot(double& v, double& w, double& q, double& r, double& fx, double& mass)
{   
     u_dot = (r*v - q*w)+(fx/mass);

     return u_dot;
} 
// Function to calculate v_dot
double Aircraft::calculate_v_dot(double& u, double& w, double&p,double& fy, double& mass)
{   
     v_dot = (p*w - r*u)+(fy/mass);
     
     return v_dot;
} 
// Function to calculate w_dot
double Aircraft::calculate_w_dot(double& u, double& v, double& p,  double& q,double& fz, double& mass)
{  
     w_dot = (q*u - p*v)+(fz/mass);

     return w_dot;
} 

// Function to calculate p_dot
double Aircraft::calculate_p_dot(double& p, double& q, double& r,  double& ell,double& n, double& Gamma_1,double& Gamma_2,double& Gamma_3,double& Gamma_4)
{
    p_dot = Gamma_1*p*q - Gamma_2*q*r + Gamma_3*ell + Gamma_4*n;
 
    return p_dot;
}

// Function to calculate q_dot
double Aircraft::calculate_q_dot(double& p, double& r,  double& m ,double& Jy, double& Gamma_5,double& Gamma_6)
{
    q_dot = Gamma_5*p*r - Gamma_6*((p*p)-(r*r)) + (m/Jy); 

    return q_dot;    
}
// Function to calculate r_dot
double Aircraft::calculate_r_dot(double& p, double& q, double& r,  double& ell,double& n, double& Gamma_1,double& Gamma_4,double& Gamma_7,double& Gamma_8)
{ 
    r_dot = Gamma_7*p*q - Gamma_1*q*r + Gamma_4*ell + Gamma_8*n;

    return r_dot;
}





// Function to generate 2D plots for the states
void Aircraft::graphing() 
{
    /* TEMPORARILY SUPPRESSED: FUTURE FEATURE
    g_clock.push_back(X.clock);
	g_pn.push_back(X.pn);
    g_pe.push_back(X.pe);
    g_pd.push_back(X.pd);
    g_phi.push_back((180/M_PI)*X.phi);
    g_theta.push_back((180/M_PI)*X.theta);
    g_psi.push_back((180/M_PI)*X.psi);
    g_p.push_back((180/M_PI)*X.p);
    g_q.push_back((180/M_PI)*X.q);
    g_r.push_back((180/M_PI)*X.r);
    g_V_m.push_back(X.V_m);
	g_alpha.push_back((180/M_PI)*X.alpha);
    g_beta.push_back((180/M_PI)*X.beta); 

    // Pn
    plt::subplot(4, 3, 1);
    plt::xlabel("time /s");
    plt::ylabel("pn");
	plt::plot(clock, &g_pn);
    // Pe
    plt::subplot(4, 3, 2);
    plt::xlabel("time /s");
    plt::ylabel("pe");
    plt::plot(clock, &g_pe);
    // Pd
    plt::subplot(4, 3, 3);
    plt::xlabel("time /s");
    plt::ylabel("pd");
    plt::plot(clock, g_pd);
    // Roll angle
    plt::subplot(4, 3, 4);
    plt::xlabel("time /s");
    plt::ylabel("ϕ");
	plt::plot(clock, g_phi);
    // Pitch angle
    plt::subplot(4, 3, 5);
    plt::xlabel("time /s");
    plt::ylabel("θ");
    plt::plot(clock, g_theta);
    // Yay angle
    plt::subplot(4, 3, 6);
    plt::xlabel("time /s");
    plt::ylabel("𝛙");
    plt::plot(clock, g_psi);
    // Roll Rate
    plt::subplot(4, 3, 7);
    plt::xlabel("time /s");
    plt::ylabel("p");
	plt::plot(clock, g_p);
    // Pitch Rate
    plt::subplot(4, 3, 8);
    plt::xlabel("time /s");
    plt::ylabel("q");
    plt::plot(clock, g_q);
    // Yaw Rate
    plt::subplot(4, 3, 9);
    plt::xlabel("time /s");
    plt::ylabel("r");
    plt::plot(clock, g_r);
    // Velocity magnitude
    plt::subplot(4, 3, 10);
    plt::xlabel("time /s");
    plt::ylabel("V_m");
	plt::plot(clock, g_V_m);
    // Angle of attack
    plt::subplot(4, 3, 11);
    plt::xlabel("time /s");
    plt::ylabel("α");
    plt::plot(clock, g_alpha);
    // Side slip angle
    plt::subplot(4, 3, 12);
    plt::xlabel("time /s");
    plt::ylabel("β");
    plt::plot(clock, g_beta);


    
    plt::tight_layout();
    plt::pause(0.0001);
    

	// Show plots
	*/
}
// Function to perform rotation on the aircraft geometry
void Aircraft::rotate(easy3d::vec3* vertices)
{
    // Create the rotation matrix using Euler angles
    easy3d::Mat3<float> rotationMatrix = easy3d::Mat3<float>::rotation(X[9], X[10] , X[11], 321);
        
    // Apply the rotation to the vertices and hope that it actually works
        for (int i = 0; i < mesh->n_vertices(); ++i) {
            vertices[i] = rotationMatrix * vertices[i];
        }

}

// Function to perform rotation on the axis vertices
void Aircraft::rotate_axes(easy3d::vec3* axesVertices)
{
    // Create the rotation matrix using Euler angles (same rotation as aircraft)
    easy3d::Mat3<float> rotationMatrix = easy3d::Mat3<float>::rotation(X[9], X[10] , X[11], 321);
    
    // Assuming we have 6 vertices for the 3 axes (X, Y, Z), rotate them
    for (int i = 0; i < 6; ++i) {
        axesVertices[i] = rotationMatrix * axesVertices[i];
    }
}


// Function to perform translation of the aircraft geometry
void Aircraft::translate(easy3d::vec3* vertices)
{

    easy3d::vec3 translationVector(static_cast<float>(X[0]), static_cast<float>(X[1]), static_cast<float>(X[2]));


// Position Update loop

            for (int j=0; j<mesh->n_vertices(); ++j)
            {
                vertices[j] -= translationVector;  // Apply translation
            }    

    
}

// Function to perform translation of the axes
void Aircraft::translate_axes(easy3d::vec3* axesVertices)
{
    // Use the aircraft's position (pn, pe, pd) as the translation vector
    easy3d::vec3 translationVector(static_cast<float>(X[0]), static_cast<float>(X[1]), static_cast<float>(X[2]));

    // Apply the translation to each vertex (assuming 6 vertices for the axes)
    for (int i = 0; i < 6; ++i) {
        axesVertices[i] -= translationVector;
    }
}


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
        vertices_aircraft.push_back(mesh->position(v)*aircraft_scale);
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


// Function to render a local coordinate frame for the UAV
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





// Function to create the grid system
void Aircraft::createGridDrawable(easy3d::Viewer& viewer)
{   
    // Create a LinesDrawable to visualize the 3D grid.
    gridDrawable = new easy3d::LinesDrawable("grid");
    
    // Create the grid lines.
    
    for (int i = 0; i < numLines; i++) 
    {
        float t = -0.5f * size + (size / (numLines - 1)) * i;
// X-Y Plane
        // Create a vertical line along the x-axis.
        grid_vertices.push_back(easy3d::vec3(t, -0.5f * size, 0.0f-offset));
        grid_vertices.push_back(easy3d::vec3(t, 0.5f * size, 0.0f-offset));

        // Create a horizontal line along the y-axis.
        grid_vertices.push_back(easy3d::vec3(-0.5f * size, t, 0.0f-offset));
        grid_vertices.push_back(easy3d::vec3(0.5f * size, t, 0.0f-offset));
// Y-Z Plane
        // Create y line along the z-axis.
        grid_vertices.push_back(easy3d::vec3(0.0f-offset, -0.5f * size, t));
        grid_vertices.push_back(easy3d::vec3(0.0f-offset,  0.5f * size, t));

        // intersecting lines (z lines/verticals)
        grid_vertices.push_back(easy3d::vec3(0.0f-offset, t, -0.5f * size));
        grid_vertices.push_back(easy3d::vec3(0.0f-offset, t,  0.5f * size));
// X-Z Plane 
        // The horizontal lines
        grid_vertices.push_back(easy3d::vec3(-0.5f * size, 0.0f-offset,  t));
        grid_vertices.push_back(easy3d::vec3(0.5f * size, 0.0f-offset,   t));
        
        // The vertical lines
        grid_vertices.push_back(easy3d::vec3(t, 0.0f-offset, -0.5f * size));
        grid_vertices.push_back(easy3d::vec3(t, 0.0f-offset,  0.5f * size));

    }

    

    
    // Upload the grid vertices to the GPU.
    gridDrawable->update_vertex_buffer(grid_vertices);

    // Set the color of the grid lines (here we use gray).
    gridDrawable->set_uniform_coloring(easy3d::vec4(0.5f, 0.5f, 0.5f, 1.0f));

    // Set the width of the grid lines (here we use 1 pixel).
    gridDrawable->set_line_width(1.0f);

    // Add the grid drawable to the viewer.
    viewer.add_drawable(gridDrawable);

    // Color settings for viewer background
        //viewer.set_background_color(easy3d::vec4(0.1f, 0.1f, 0.1f, 1.0f)); // RGBA: dark gray, fully opaque
        //viewer.set_background_color(easy3d::vec4(0.1f, 0.1f, 0.44f, 1.0f)); // Midnight Blue
        //viewer.set_background_color(easy3d::vec4(0.6f, 0.8f, 0.6f, 1.0f)); // Soft Pastel Green
        viewer.set_background_color(easy3d::vec4(0.0f, 0.0f, 0.0f, 1.0f)); // Deep Space Black
        //viewer.set_background_color(easy3d::vec4(1.0f, 0.5f, 0.0f, 1.0f)); // Sunset Orange
        //viewer.set_background_color(easy3d::vec4(0.0f, 0.5f, 0.5f, 1.0f)); // Ocean Teal
        //viewer.set_background_color(easy3d::vec4(0.5f, 0.0f, 0.13f, 1.0f)); // Rich Burgundy
        //viewer.set_background_color(easy3d::vec4(0.53f, 0.81f, 0.98f, 1.0f)); // Bright Sky Blue
        //viewer.set_background_color(easy3d::vec4(0.678f, 0.847f, 0.902f, 1.0f)); // SKY attempts
    //set_uniform_coloring(easy3d::vec4(0.678f, 0.847f, 0.902f, 1.0f));


    std::cout << "Grid drawable added to viewer" <<"\n";

    // Update the viewer
    viewer.update();


}
// Function to update all UAV related parameters per cycle
easy3d::vec3* Aircraft::update_aircraft(easy3d::vec3* vertices, easy3d::vec3* axesVertices,double& dt)
{
    // Calculate forces and moments
    
    calculate_forces();
    calculate_moments();  
    

    
    // Update dynamics
    RK4(X,dt);

    // Perform rotation and translation on the geometry
    rotate(vertices);
    translate(vertices);
    rotate_axes(axesVertices);
    translate_axes(axesVertices);

    
    /*std::cout<<"After RK4"<<std::endl;
    std::cout << X[12] << "\t" <<X[13] << "\t" << X[14] << std::endl;*/


    // Keyboard input
    collectInput();
    
    

    return vertices;
    
}
// Function to create the animation of the dynamic UAV
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


void Aircraft::initKeyboard()
{
    // Initialize ncurses for keyboard input (boiler plate)
    initscr();            // Start curses mode
    cbreak();             // Disable line buffering
    noecho();             // Don't echo user input
    keypad(stdscr, TRUE); // Enable function keys like arrow keys

}

void Aircraft::collectInput() {

    // Set non-blocking input
    nodelay(stdscr, TRUE);
    double control_step = 0.02617993878/2; //-> Move to class members

    char input = getch(); 

    switch (input) {
        case '1': // Positive roll
            if (delta_a+control_step >= delta_a_max)
            {
                delta_a = delta_a_max;  // set it to max
            }
            else
            {
                delta_a += control_step;  // Increment the value
            }
            break;
        case '3': // Negative roll
            if(delta_a-control_step <= delta_a_min)
            {
                delta_a = delta_a_min; // SEt to minimum
            }
            else
            {
                delta_a -= control_step; // decrement
            }
            break;
        case '5': // Positive pitch
            if (delta_e+control_step >= delta_e_max)
            {
                delta_e = delta_e_max;  // set it to max
            }
            else
            {
                delta_e += control_step;  // Increment the value
            }
            break;
        case '2': // Negative pitch
            if(delta_e-control_step <= delta_e_min)
            {
                delta_e = delta_e_min; // SEt to minimum
            }
            else
            {
                delta_e -= control_step; // decrement
            }
            break;
        case '4': // Positive Yaw
            if (delta_r+control_step >= delta_r_max)
            {
                delta_r = delta_r_max;  // set it to max
            }
            else
            {
                delta_r += control_step;  // Increment the value
            }
            break;
        case '6': // Negative Yaw
            if(delta_r-control_step <= delta_r_min)
            {
                delta_r = delta_r_min; // SEt to minimum
            }
            else
            {
                delta_r -= control_step; // decrement
            }
            break;
        case '+': // Positive Throttle
            if (delta_t+0.05 >= delta_t_max)
            {
                delta_t = delta_t_max;  // set it to max
            }
            else
            {
                delta_t += 0.05;  // Increment the value
            }
            break;
        case '-': // Negative Throttle
            if(delta_t-0.05 <= delta_t_min)
            {
                delta_t = delta_t_min; // SEt to minimum
            }
            else
            {
                delta_t -= 0.05; // decrement
            }
            break;
        case '7':
            X[0] += 100;  // Increment the value
            break;
        case '8':
            X[0] -= 100;; // Decrement the value
            break;
        default:
            // Ignore any other keys
            break;
    }
    // Debug output to monitor input changes
    // Display updated values on the screen
        mvprintw(0, 0, "delta_a: %.2f, delta_e: %.2f, delta_r: %.2f, delta_t: %.2f", delta_a, delta_e, delta_r, delta_t);
        //mvprintw(0, 0, "L: %.2f, M: %.2f, N: %.2f", phi*(180/M_PI), theta*(180/M_PI), psi*(180/M_PI));
        refresh(); // Refresh to display updates
    
}

