//Header file for the simulator project

class Aircraft 
{
public:

struct State
{
    // This will be used to track state variables
    double clock;
    double pn;
    double pe;
    double pd;
    double u;
    double v;
    double w; 
    double phi;
    double theta;
    double psi;
    double p;
    double q;
    double r;
    double V_m;
    double alpha;
    double beta; 
    double fx;
    double fy;
    double fz;
    double ell;
    double m;
    double n;
    // This will be used to track control inputs
    double delta_t; //throttle
    double delta_a; //aileron
    double delta_e; //elevator
    double delta_r; //rudder
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

    
    
};

    
    int dt;
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

    double Gamma=(Jx*Jz)-(Jxz*Jxz);
    double Gamma_1=(Jxz*(Jx-Jy+Jz))/Gamma;
    double Gamma_2=(Jz*(Jz-Jy)+(Jxz*Jxz))/Gamma;
    double Gamma_3=Jz/Gamma;
    double Gamma_4=Jxz/Gamma;
    double Gamma_5=(Jz-Jx)/Jy;
    double Gamma_6=Jxz/Jy;
    double Gamma_7=(((Jx-Jy)*Jx)+(Jxz*Jxz))/Gamma;
    double Gamma_8=Jx/Gamma;

    //Stability & Control derivatives

    double C_L_0         ;
    double C_L_alpha     ;
    double C_L_q         ;
    double C_L_delta_e   ;

    double C_D_0         ;
    double C_D_alpha     ;
    double C_D_p         ;
    double C_D_q         ;
    double C_D_delta_e   ;

    double C_m_0         ;
    double C_m_alpha     ;
    double C_m_q         ;
    double C_m_delta_e   ;

    double C_Y_0         ;
    double C_Y_beta      ;
    double C_Y_p         ;
    double C_Y_r         ;
    double C_Y_delta_a   ;
    double C_Y_delta_r   ;

    double C_ell_0       ;
    double C_ell_beta    ;
    double C_ell_p       ;
    double C_ell_r       ;
    double C_ell_delta_a ;
    double C_ell_delta_r ;

    double C_n_0         ;
    double C_n_beta      ;
    double C_n_p         ;
    double C_n_r         ;
    double C_n_delta_a   ;
    double C_n_delta_r   ;

    double C_prop        ;
    double trans_rate    ;
    double epsilon       ;
    double alpha0        ;
    double beta0         ;

    // Initial state
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

    // Constructor
    Aircraft(const std::string& fname, int& vehicle_count);
    
    // functin to get the state
    State get_state()
    {

        return state;
    }

    // function to load a plane from text file
    void load_a_plane(const std::string& filePath, int& vehicle_count);
    // functions to act on the state changes
    void forces_moments(State& X, const Aircraft& Y);
    void dynamics(State& X, const Aircraft& Y, double& dt);
    void graphing(const State& X, std::vector<double>& clock, std::vector<double>& pn, std::vector<double>& pe,  std::vector<double>& pd,  std::vector<double>& phi,  std::vector<double>& theta,  std::vector<double>& psi,  std::vector<double>& p,  std::vector<double>& q,  std::vector<double>& r,  std::vector<double>& V_m, std::vector<double>& alpha, std::vector<double>& beta);
    // functions for 3D rendering based on state changes
    void rotate(const State& X, easy3d::vec3* vertices, const int& vertices_size, float& old_roll, float& old_pitch, float& old_yaw );
    void translate(const State& X, easy3d::vec3* vertices, const int& vertices_size,float& pn, float& pe, float& pd);

    private:

    State state;
    
    
    
};
