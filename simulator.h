//Header file for the simulator project

#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <memory> // For std::unique_ptr
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <easy3d/viewer/viewer.h>
#include <easy3d/renderer/drawable_triangles.h>
#include <easy3d/renderer/drawable_lines.h>
#include <easy3d/renderer/vertex_array_object.h>
#include <fstream>
#include <cmath>
#include <ncurses.h>


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
    int aircraft_scale = 500;

    std::vector<std::vector<int>> faces;
    std::vector<easy3d::vec3> vertices_aircraft;
    std::vector<unsigned int> indices;
    
    std::vector<easy3d::vec3> dummy_points;//DUMMY var for points
    std::vector<easy3d::vec3> &points;

    easy3d::TrianglesDrawable* aircraft;
    const float size;
    const int numLines;
    const float offset;
    easy3d::LinesDrawable* gridDrawable;
    std::vector<easy3d::vec3> grid_vertices;

    // For Plotting graphs [FUTURE]
    std::vector<double> g_clock, g_pn,g_pe,g_pd,g_phi,g_theta,g_psi, g_p,g_q,g_r,g_V_m,g_alpha,g_beta;
    


    // Constructor
    Aircraft(const std::string& fname, int& vehicle_count);
    //Destructor
    ~Aircraft();
    
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
    void graphing(const State& X);
    // functions for 3D rendering based on state changes
    void rotate(const State& X, easy3d::vec3* vertices, const int& vertices_size);
    void translate(const State& X, easy3d::vec3* vertices, const int& vertices_size,float& pn, float& pe, float& pd);
    void initializePreviousState();
    void initializeVertices();
    void createAircraftDrawable(easy3d::Viewer& viewer);
    void createGridDrawable(easy3d::Viewer& viewer);
    bool animate(easy3d::Viewer* viewer, Aircraft::State& state, double dt);
    void initializeVerticesIndices();
    void initKeyboard();
    void collectInput(State& X);
    //easy3d::Mat3<float> transpose(const easy3d::Mat3<float>& matrix); // test for easy3d mat3 transpose
   
   



    private:

    State state;
    
    
    
};

#endif