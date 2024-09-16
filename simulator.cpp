#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include "matplotlibcpp.h"
#include <easy3d/viewer/viewer.h>
#include <easy3d/renderer/camera.h>
#include <easy3d/renderer/drawable_triangles.h>
#include <easy3d/core/types.h>
#include <easy3d/util/resource.h>
#include <easy3d/util/initializer.h>
#include <easy3d/renderer/drawable_lines.h>
#include <easy3d/renderer/vertex_array_object.h>
#define _USE_MATH_DEFINES

namespace plt = matplotlibcpp;
//Forward declaration
class Aircraft;

struct State{
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

    // functions to act on the state changes
    void forces_moments(State& X, const Aircraft& Y);
    void dynamics(State& X, const Aircraft& Y, double& dt);
    void graphing(const State& X, std::vector<double>& clock, std::vector<double>& pn, std::vector<double>& pe,  std::vector<double>& pd,  std::vector<double>& phi,  std::vector<double>& theta,  std::vector<double>& psi,  std::vector<double>& p,  std::vector<double>& q,  std::vector<double>& r,  std::vector<double>& V_m, std::vector<double>& alpha, std::vector<double>& beta);
    // functions for 3D rendering based on state changes
    void rotate(const State& X, easy3d::vec3* vertices, const int& vertices_size, float& old_roll, float& old_pitch, float& old_yaw );
    void translate(const State& X, easy3d::vec3* vertices, const int& vertices_size,float& pn, float& pe, float& pd);
    
};

class Aircraft {
public:
    
    
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
    double beta0;

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
    State get_state(){

        return state;
    }

    private:

    State state;
    // function to load a plane from text file
    void load_a_plane(const std::string& filePath, int& vehicle_count);
    
    
};

int main() {

    int steps = 10;  
    double dt = 0.0003;
    int vehicle_count = 1;
    std::string fname = "../data.txt";
    Aircraft obj(fname,vehicle_count);
    State state = obj.get_state(); 
    // For Plotting graphs
    std::vector<double> clock(steps), pn(steps),pe(steps),pd(steps),phi(steps),theta(steps),psi(steps), p(steps),q(steps),r(steps),V_m(steps),alpha(steps),beta(steps);
    //For "3D" visuals lol 
    // Eigen::RowVector3d x(0.0,1.4,1.1,1.1, 1.1,1.1,0.0,0.0,0.0,0.0, 0.0);
    // Eigen::RowVector3d y(0.0,0.0,0.0,0.6,-0.6,0.0,0.0,0.0,0.0,0.3,-0.3);
    // Eigen::RowVector3d z(0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.3,0.0,0.0, 0.0);
    // std::vector<double> x = {0.0,1.4,1.1,1.1, 1.1,1.1,0.0,0.0,0.0,0.0, 0.0};
    // std::vector<double> y = {0.0,0.0,0.0,0.6,-0.6,0.0,0.0,0.0,0.0,0.3,-0.3};
    // std::vector<double> z = {0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.3,0.0,0.0, 0.0};
    // Aircraft 3D
    // Define the coordinates of the vertices of the aircraft.
    std::vector<double> vertices_x = {-4, -2, -2, -2, -2, 12, 0, 3, 3, 0, 9.5, 12, 12, 9.5, 9.5, 12};
    std::vector<double> vertices_y = {0, 1, -1, -1, 1, 0, 5, 5, -5, -5, 2.5, 2.5, -2.5, -2.6, 0, 0};
    std::vector<double> vertices_z = {0, -1, -1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5};
    int vertices_size = vertices_x.size();
    float old_roll = static_cast<float>(obj.phi_0); // For fixing rotation issues. Just a test
    float old_pitch = static_cast<float>(obj.theta_0); // For fixing rotation issues. Just a test
    float old_yaw = static_cast<float>(obj.psi_0); // For fixing rotation issues. Just a test
    
    float old_pn = static_cast<float>(obj.pn_0);
    float old_pe = static_cast<float>(obj.pe_0);
    float old_pd = static_cast<float>(obj.pd_0); 

    std::cout<<"SIZE = "<<vertices_size<<"\n";
    // For scaling the thing
    int aircraft_scale = 300;
    for (int i = 0; i < vertices_x.size()-1; i++){
        vertices_x[i] = aircraft_scale*vertices_x[i];
        vertices_y[i] = aircraft_scale*vertices_y[i];
        vertices_z[i] = aircraft_scale*vertices_z[i];


    }



    // Define the faces of the aircraft.
    std::vector<std::vector<int>> faces = {
        {0, 1, 2},    // cockpit top
        {0, 2, 3},    // cockpit left
        {0, 3, 4},    // cockpit bottom
        {0, 4, 1},    // cockpit right
        {1, 5, 2},    // fuselage top
        {2, 5, 3},    // fuselage left
        {3, 5, 4},    // fuselage bottom
        {4, 5, 1},    // fuselage right
        {6, 7, 8},    // wing triangle half
        {6, 9, 8},    // wing triangle half
        {10, 11, 12}, // horizontal stabilizer tri. half
        {10, 13, 12}, // horizontal stabilizer tri. half
        {14, 5, 15}   // vertical stabilizer
    };

    //-------------------------------------------------------------

   

    // Populate the aircraftDrawable with the vertices and faces.
    std::vector<easy3d::vec3> vertices_aircraft;
    std::vector<unsigned int> indices;

    // Add the vertices to the 'vertices' vector.
    for (size_t i = 0; i < vertices_x.size(); ++i) {
        vertices_aircraft.push_back(easy3d::vec3(vertices_x[i], vertices_y[i], vertices_z[i]));
    }

    std::vector<easy3d::vec3> &points = vertices_aircraft;
    // Add the faces to the 'indices' vector.
    for (const auto& face : faces) {
        indices.push_back(face[0]);
        indices.push_back(face[1]);
        indices.push_back(face[2]);
    }
    
    // initialize Easy3D.
    easy3d::initialize();
    std::cout << "INITIALISING 3D" <<"\n";
    // Create the default Easy3D viewer.
    // Note: a viewer must be created before creating any drawables.
    easy3d::Viewer viewer("TEST");
    std::cout << "Viewer Object created" <<"\n";
    auto aircraft = new easy3d::TrianglesDrawable("faces");
    std::cout << "Triangles Drawable Created" <<"\n";
    // Upload the vertex positions of the surface to the GPU.
    aircraft->update_vertex_buffer(points);
    // Upload the vertex indices of the surface to the GPU.
    aircraft->update_element_buffer(indices);
    //color test
    aircraft->set_uniform_coloring(easy3d::vec4(1.0f, 0.0f, 0.0f, 1.0f));
    // Add the drawable to the viewer
    viewer.add_drawable(aircraft);
    std::cout << "Aircraft drawable added to viewer" <<"\n";

    // Grid 3D
    const float size = 120000.0f;
    const int numLines = 22;
    const float offset = size/2;

    // Create a LinesDrawable to visualize the 3D grid.
    auto gridDrawable = new easy3d::LinesDrawable("grid");
    std::cout << "Grid lines drawing created" <<"\n";

    // Create the grid lines.
    std::vector<easy3d::vec3> grid_vertices;
    for (int i = 0; i < numLines; i++) {
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
    std::cout << "Grid drawable added to viewer" <<"\n";
    //===========================================================

     // Make sure everything is within the visible region of the viewer.
    viewer.fit_screen();
    viewer.set_animation(true);
    // Define an animation function to specify how vertex positions are updated.
    // Adapted from example (Tutorial 311)
    
    viewer.animation_func_ = [&](easy3d::Viewer* v) -> bool {
        (void)v;
      
        // map the vertex buffer into the client's address space
        void* pointer = easy3d::VertexArrayObject::map_buffer(GL_ARRAY_BUFFER, aircraft->vertex_buffer(), GL_WRITE_ONLY);
        
        
        easy3d::vec3* vertices = reinterpret_cast<easy3d::vec3*>(pointer);
        if (!vertices)
            
            return false;

                // std::vector<float> pn_float;
                // std::vector<float> pe_float;
                // std::vector<float> pd_float;
                // float pn_float;
                // float pe_float;
                // float pd_float;
                
                state.forces_moments(state,obj);
                
                std::cout<<"state.V: "<<state.V_m<< "\n";
                std::cout<<"obj.V: "<<obj.v_0<<"\n";
                state.dynamics(state,obj,dt);
                
                //state.graphing(state, clock, pn, pe, pd, phi, theta, psi, p, q, r, V_m, alpha, beta);

                // Rotating test

                state.rotate(state,vertices,vertices_size, old_roll, old_pitch, old_yaw);
                state.translate(state,vertices,vertices_size,old_pn, old_pe, old_pd);
                
                       

        // unmap the vertex buffer
        easy3d::VertexArrayObject::unmap_buffer(GL_ARRAY_BUFFER, aircraft->vertex_buffer());

        viewer.update();
        
        return true;
                 
    //plt::show();
       
    };

    
    

    
      
    return viewer.run();
    
}


// Class functions
void Aircraft::load_a_plane(const std::string& filePath, int& vehicle_count) {
            
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

            

           

            file >> C_L_0         ;
            file >> C_L_alpha     ;
            file >> C_L_q         ;
            file >> C_L_delta_e   ;

            file >> C_D_0         ;
            file >> C_D_alpha     ;
            file >> C_D_p         ;
            file >> C_D_q         ;
            file >> C_D_delta_e   ;

            file >> C_m_0         ;
            file >> C_m_alpha     ;
            file >> C_m_q         ;
            file >> C_m_delta_e   ;

            file >> C_Y_0         ;
            file >> C_Y_beta      ;
            file >> C_Y_p         ;
            file >> C_Y_r         ;
            file >> C_Y_delta_a   ;
            file >> C_Y_delta_r   ;

            file >> C_ell_0       ;
            file >> C_ell_beta    ;
            file >> C_ell_p       ;
            file >> C_ell_r       ;
            file >> C_ell_delta_a ;
            file >> C_ell_delta_r ;

            file >> C_n_0         ;
            file >> C_n_beta      ;
            file >> C_n_p         ;
            file >> C_n_r         ;
            file >> C_n_delta_a   ;
            file >> C_n_delta_r   ;

            file >> C_prop        ;
            file >> trans_rate    ;
            file >> epsilon       ;
            file >> alpha0        ;

            
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





Aircraft::Aircraft(const std::string& fname, int& vehicle_count){

        load_a_plane(fname,vehicle_count);
        
        state.clock = 0;
        state.pn = pn_0;
        state.pe = pe_0;
        state.pd = pd_0;
        state.u = u_0;
        state.v = v_0;
        state.w = w_0; 
        state.phi = phi_0;
        state.theta = theta_0;
        state.psi = psi_0;
        state.p = p_0;
        state.q = q_0;
        state.r = r_0;
        state.V_m = u_0; // SUSPECT
        state.alpha = alpha0;
        state.beta = beta0;
        state.delta_t = delta_t;
        state.delta_a = delta_a;
        state.delta_e = delta_e;
        state.delta_r = delta_r; 

        

    }

   // Struct functions
void State::forces_moments(State& X, const Aircraft& Y){

    double u  = X.u;
    double v  = X.v;
    double w  = X.w;
    double phi = X.phi;
    double theta = X.theta;
    double psi = X.psi;
    double p = X.p;
    double q = X.q;
    double r = X.r;
    double V_m;//double V_m = X.V_m;
    double alpha0 = X.alpha;
    double beta = X.beta;
    std::vector<double> velocity_b = X.velocity_b;

    double delta_t = X.delta_t; 
    double delta_a = X.delta_a; 
    double delta_e = X.delta_e; 
    double delta_r = X.delta_r; 

    // Velocity in bodyframe
    velocity_b = {u,v,w};
    // Velocity magnitude (square root of the sum of the squares of u v w)
    V_m = sqrt((velocity_b[0]*velocity_b[0])+(velocity_b[1]*velocity_b[1])+( velocity_b[2]*velocity_b[2]));
    // Angle of attack (alpha)
    alpha = std::atan2(velocity_b[2],velocity_b[0]);
    // Side slip angle (beta)
    beta = std::asin(velocity_b[1]/V_m);

    // Lift/Drag coefficient calculations [ Cl and Cd ]
      // Cd(alpha)                                          //* <------SUSPECT
    double Cd_of_alpha = Y.C_D_p + ((Y.C_L_0 + Y.C_L_alpha*alpha)*(Y.C_L_0 + Y.C_L_alpha*alpha)/(M_PI*Y.e*Y.wing_aspect_ratio));
      
      // sigma(alpha)
    double sigma_num = 1 + std::exp(-Y.trans_rate*(alpha-alpha0)) + std::exp(Y.trans_rate*(alpha+alpha0));
    double sigma_den = (1 + std::exp(-Y.trans_rate*(alpha-alpha0))) * (1 + std::exp(Y.trans_rate*(alpha+alpha0)));
    double sigma_of_alpha = sigma_num/sigma_den;
    
     // Cl of flat plate
     double Cl_flat_plate = 2*(std::signbit(alpha) ? -1.0 : 1.0) * (std::sin(alpha) * std::sin(alpha)) * (std::cos(alpha));
     // Linear Cl
     double Cl_linear = Y.C_L_0 + Y.C_L_alpha*alpha;
     // Combined Cl
     double Cl_of_alpha = ((1-sigma_of_alpha) * (Cl_linear)) + (sigma_of_alpha * Cl_flat_plate);

    // Coefficients for X and Z directions
    double CxAlpha = (-Cd_of_alpha*std::cos(alpha))+(Cl_of_alpha*std::sin(alpha));
    double CxqAlpha = (-Y.C_D_q*std::cos(alpha))+(Y.C_L_q*std::sin(alpha));
    double CxdeltaeAlpha = (-Y.C_D_delta_e*std::cos(alpha))+(Y.C_L_delta_e*std::sin(alpha));
    double CzAlpha = (-Cd_of_alpha*std::sin(alpha))-(Cl_of_alpha*std::cos(alpha));
    double CzqAlpha = (-Y.C_D_q*std::sin(alpha))-(Y.C_L_q*std::cos(alpha));
    double CzdeltaeAlpha = (-Y.C_D_delta_e*std::sin(alpha))-(Y.C_L_delta_e*std::cos(alpha));

    // Sources of Forces
    Eigen::Vector3d force_g;
    force_g << -Y.mass * Y.g*std::sin(theta),Y.mass * Y.g*std::cos(theta)*sin(phi),Y.mass * Y.g*std::cos(theta)*cos(phi);
    double force_aero1 = 0.5 * Y.rho * (V_m*V_m) * Y.wing_area;

    Eigen::Vector3d force_aero2;
    force_aero2 <<(CxAlpha+CxqAlpha*((Y.wing_chord)/(2*V_m))*q)+CxdeltaeAlpha*delta_e,
        Y.C_Y_0+(Y.C_Y_beta*beta)+(Y.C_Y_p*((Y.wing_span)/(2*V_m))*p)+(Y.C_Y_r*((Y.wing_span)/(2*V_m))*r)+(Y.C_Y_delta_a*delta_a)+(Y.C_Y_delta_r*delta_r),
        CzAlpha+(CzqAlpha*((Y.wing_chord)/(2*V_m))*q)+CzdeltaeAlpha*delta_e;
    Eigen::Vector3d force_aero;
    force_aero << force_aero1 * force_aero2;

    double force_prop1 = 0.5*Y.rho*Y.prop_area*Y.C_prop;
    Eigen::Vector3d force_prop2;
    force_prop2 << ((Y.k_motor*delta_t)*(Y.k_motor*delta_t))-(V_m*V_m),0,0; 
    Eigen::Vector3d force_prop;
    force_prop << force_prop1 * force_prop2; 

    Eigen::Vector3d Force;
    Force << force_g + force_aero + force_prop;  

   

    // Moment/Torque calculations

    double Aero_t1 = force_aero1;
    
    //Suspect (Y.C_n_p**((Y.wing_span)/(2*V_m))*p
    Eigen::Vector3d Aero_t2;
    Aero_t2 << Y.wing_span*(Y.C_ell_0 + (Y.C_ell_beta*beta) + (Y.C_ell_p*((Y.wing_span)/(2*V_m))*p) + (Y.C_ell_r*((Y.wing_span)/(2*V_m))*r) + (Y.C_ell_delta_a*delta_a)+(Y.C_ell_delta_r*delta_r)),
    Y.wing_chord*(Y.C_m_0 + (Y.C_m_alpha*alpha) + (Y.C_m_q*((Y.wing_chord)/(2*V_m))*q) + (Y.C_m_delta_e*delta_e)),
    Y.wing_span*(Y.C_n_0 + (Y.C_n_beta*beta) + (Y.C_n_p*((Y.wing_span)/(2*V_m))*p) + (Y.C_n_r*((Y.wing_span)/(2*V_m))*r) + (Y.C_n_delta_a*delta_a)+(Y.C_n_delta_r*delta_r));  
    //std::cout<<"Aero_t2: "<< Aero_t2<<"\n"; //DEBUG
    Eigen::Vector3d Aero_torque;

    Aero_torque = Aero_t1 * Aero_t2;
    //std::cout<<"Aero_t1 * Aero_t2: "<<Aero_torque<<"\n"; //DEBUG
    Eigen::Vector3d Prop_torque;

    Prop_torque << -Y.prop_thrust_coef * ((Y.prop_omega*delta_t)*(Y.prop_omega*delta_t)),0,0;
    //std::cout<<"Prop_torque: "<<Prop_torque<<"\n"; //DEBUG
    Eigen::Vector3d Torque;

    Torque = Aero_torque + Prop_torque;
    //std::cout<<"Torque total: "<<Torque<<"\n"; //DEBUG
    // Writing outputs for Force and Torque
    X.fx = Force[0];
    X.fy = Force[1];
    X.fz = Force[2];

    X.ell = Torque[0];
    X.m = Torque[1];
    X.n = Torque[2];

    X.alpha = alpha;
    X.beta = beta;
    X.V_m = V_m;
    
    
}

void State::dynamics(State& X, const Aircraft& Y, double& dt){

    double u = X.u;
    double v = X.v;
    double w = X.w; 
    double phi = X.phi;
    double theta = X.theta;
    double psi = X.psi;
    double p = X.p;
    double q = X.q;
    double r = X.r;
    double fx = X.fx;
    double fy = X.fy;
    double fz = X.fz;
    double ell = X.ell;
    double m = X.m;
    double n = X.n;



double pn_dot = w*(std::sin(phi)*std::sin(psi) + std::cos(phi)*std::cos(psi)*std::sin(theta)) - v*(std::cos(phi)*std::sin(psi) - std::cos(psi)*std::sin(phi)*std::sin(theta)) + u*std::cos(psi)*std::cos(theta);
double pe_dot = v*(std::cos(phi)*std::cos(psi) + std::sin(phi)*std::sin(psi)*std::sin(theta)) - w*(std::cos(psi)*std::sin(phi) - std::cos(phi)*std::sin(psi)*std::sin(theta)) + u*std::cos(theta)*std::sin(psi);
double pd_dot = w*std::cos(phi)*std::cos(theta) - u*std::sin(theta) + v*std::cos(theta)*std::sin(phi);
double u_dot = (r*v - q*w)+(fx/Y.mass);
double v_dot = (p*w - r*u)+(fy/Y.mass);
double w_dot = (q*u - p*v)+(fz/Y.mass);
double phi_dot = p + r*std::cos(phi)*std::tan(theta) + q*std::sin(phi)*std::tan(theta);
double theta_dot = q*std::cos(phi) - r*std::sin(phi);
double psi_dot = (r*std::cos(phi))/std::cos(theta) + (q*std::sin(phi))/std::cos(theta);
double p_dot = Y.Gamma_1*p*q - Y.Gamma_2*q*r + Y.Gamma_3*ell + Y.Gamma_4*n;
double q_dot = Y.Gamma_5*p*r - Y.Gamma_6*((p*p)-(r*r)) + (m/Y.Jy); 
double r_dot = Y.Gamma_7*p*q - Y.Gamma_1*q*r + Y.Gamma_4*ell + Y.Gamma_8*n;

// Making this the next initial values
X.pn =  X.pn + pn_dot * dt;
X.pe = X.pe + pe_dot * dt;
X.pd = X.pd + pd_dot * dt;
X.u = X.u + u_dot * dt;
X.v = X.v + v_dot * dt;
X.w = X.w + w_dot * dt;
X.phi = X.phi + phi_dot * dt;
X.theta = X.theta + theta_dot * dt;
X.psi = X. psi + psi_dot * dt;
X.p = X.p + p_dot * dt;
X.q = X.q + q_dot * dt;
X.r = X.r + r_dot * dt;
X.clock++;




}

void State::graphing(const State& X, std::vector<double>& clock, std::vector<double>& pn, std::vector<double>& pe,  std::vector<double>& pd,  std::vector<double>& phi,  std::vector<double>& theta,  std::vector<double>& psi,  std::vector<double>& p,  std::vector<double>& q,  std::vector<double>& r,  std::vector<double>& V_m, std::vector<double>& alpha, std::vector<double>& beta) 
{
    // // Prepare data
    // const double clock = X.clock;
	// const double pn= X.pn;
    // const double pe = X.pe;
    // const double pd = X.pd;
    // const double phi = X.phi;
    // const double theta = X.theta;
    // const double psi= X.psi;
    // const double p = X.p;
    // const double q = X.q;
    // const double r = X.r;
    // const double V_m = X.V_m;
	// const double alpha = X.alpha;
    // const double beta = X.beta;
    //int n = 10;
	//std::vector<double> clock(n), pn(n),pe(n),pd(n),phi(n),theta(n),psi(n), p(n),q(n),r(n),V_m(n),alpha(n),beta(n);
    // Prepare data
    
    

    
    clock.push_back(X.clock);
	pn.push_back(X.pn);
    pe.push_back(X.pe);
    pd.push_back(X.pd);
    phi.push_back((180/M_PI)*X.phi);
    theta.push_back((180/M_PI)*X.theta);
    psi.push_back((180/M_PI)*X.psi);
    p.push_back((180/M_PI)*X.p);
    q.push_back((180/M_PI)*X.q);
    r.push_back((180/M_PI)*X.r);
    V_m.push_back(X.V_m);
	alpha.push_back((180/M_PI)*X.alpha);
    beta.push_back((180/M_PI)*X.beta); 

    // Pn
    plt::subplot(4, 3, 1);
    plt::xlabel("time /s");
    plt::ylabel("pn");
	plt::plot(clock, pn);
    // Pe
    plt::subplot(4, 3, 2);
    plt::xlabel("time /s");
    plt::ylabel("pe");
    plt::plot(clock, pe);
    // Pd
    plt::subplot(4, 3, 3);
    plt::xlabel("time /s");
    plt::ylabel("pd");
    plt::plot(clock, pd);
    // Roll angle
    plt::subplot(4, 3, 4);
    plt::xlabel("time /s");
    plt::ylabel("ϕ");
	plt::plot(clock, phi);
    // Pitch angle
    plt::subplot(4, 3, 5);
    plt::xlabel("time /s");
    plt::ylabel("θ");
    plt::plot(clock, theta);
    // Yay angle
    plt::subplot(4, 3, 6);
    plt::xlabel("time /s");
    plt::ylabel("𝛙");
    plt::plot(clock, psi);
    // Roll Rate
    plt::subplot(4, 3, 7);
    plt::xlabel("time /s");
    plt::ylabel("p");
	plt::plot(clock, p);
    // Pitch Rate
    plt::subplot(4, 3, 8);
    plt::xlabel("time /s");
    plt::ylabel("q");
    plt::plot(clock, q);
    // Yaw Rate
    plt::subplot(4, 3, 9);
    plt::xlabel("time /s");
    plt::ylabel("r");
    plt::plot(clock, r);
    // Velocity magnitude
    plt::subplot(4, 3, 10);
    plt::xlabel("time /s");
    plt::ylabel("V_m");
	plt::plot(clock, V_m);
    // Angle of attack
    plt::subplot(4, 3, 11);
    plt::xlabel("time /s");
    plt::ylabel("α");
    plt::plot(clock, alpha);
    // Side slip angle
    plt::subplot(4, 3, 12);
    plt::xlabel("time /s");
    plt::ylabel("β");
    plt::plot(clock, beta);


    
    plt::tight_layout();
    plt::pause(0.0001);
    
    
    
    


    
	// Show plots
	
}

void State::rotate(const State& X, easy3d::vec3* vertices, const int& vertices_size, float& old_roll, float& old_pitch, float& old_yaw ){

        float roll = static_cast<float>(X.phi);
        float pitch = static_cast<float>(X.theta);
        float yaw = static_cast <float>(X.psi);
        std::cout<< "Roll: "<<(roll*180/3.14) << "Pitch: "<<(pitch*180/3.14)<<"Yaw: "<<(yaw*180/3.14)<<"\n";
            
        // Create the rotation matrix using Euler angles
        easy3d::Mat3<float> rotationMatrix = easy3d::Mat3<float>::rotation(old_roll-roll, old_pitch-pitch , old_yaw-yaw, 321);
        
        old_roll = roll;
        old_pitch = pitch;
        old_yaw = yaw;


    // Apply the rotation to the vertices and hope that it actually works
        for (int i = 0; i < vertices_size; ++i) {
            vertices[i] = rotationMatrix * vertices[i];
        }


}

void State::translate(const State& X, easy3d::vec3* vertices, const int& vertices_size, float& old_pn, float& old_pe, float& old_pd){


float pn_float = static_cast<float>(X.pn);
float pe_float = static_cast<float>(X.pe);
float pd_float = static_cast<float>(X.pd);
// float delta_pn = pn_float - old_pn;
// float delta_pe = pe_float - old_pe;
// float delta_pd = pd_float - old_pd;


// Position Update loop

            for (int j=0; j<vertices_size; ++j){
                vertices[j].x += pn_float;
                vertices[j].y += pe_float;
                vertices[j].z += pd_float;
                
            }     

// old_pn = pn_float;
// old_pe = pe_float;
// old_pd = pd_float;


}