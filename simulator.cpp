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
#include <memory>
#include "simulator.h"
#define _USE_MATH_DEFINES

namespace plt = matplotlibcpp;



int main() 
{
    // Simulation parameters
    int steps = 10;  
    double dt = 0.00001;
    int vehicle_count = 1;
    std::string fname = "../data.txt";

    //Instantiate a UAV
    Aircraft obj(fname,vehicle_count);
    Aircraft::State state = obj.get_state(); 
    obj.steps = steps;
    obj.dt = dt;
    
    // Initialize UAV geometry
    obj.initializeVertices();
    obj.initializePreviousState();
    obj.initializeVerticesIndices();
    
    
    // initialize Easy3D.
    easy3d::initialize(true);
    
    // Create the default Easy3D viewer. Note: a viewer must be created before creating any drawables.
    easy3d::Viewer viewer("UAV Simulator");

    // Draw the aircraft and 3D graphs
    obj.createAircraftDrawable(viewer);
    obj.createGridDrawable(viewer);

    // Make sure everything is within the visible region of the viewer.
    viewer.fit_screen();
    viewer.set_animation(true); // -> True here will create a log file in /build
    
    // Animation function
    viewer.animation_func_ = [&](easy3d::Viewer* v) -> bool 
    {
        return obj.animate(v, state, dt);
    };

      
    return viewer.run();
    
}


// Class functions
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




// Class constructor
Aircraft::Aircraft(const std::string& fname, int& vehicle_count)
:size(120000.0f),numLines(22),offset(120000.0f/2),points(dummy_points),aircraft(nullptr),gridDrawable(nullptr),steps(10)
{
    //1. First it loads the aircraft parameters from the file using the following function
        load_a_plane(fname,vehicle_count);
    //2. Then it modifies the state values by setting them to the defaults imported from the aircraft parameter file.    
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
Aircraft::~Aircraft() 
{
    if (aircraft) {
        delete aircraft; 
    }
    if (gridDrawable) {
        delete gridDrawable; 
    }
}
// Struct functions
void Aircraft::forces_moments(State& X, const Aircraft& Y)
{

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
    double alpha = std::atan2(velocity_b[2],velocity_b[0]);
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
    Eigen::Vector3d force_g; // Component: Gravity
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

void Aircraft::dynamics(State& X, const Aircraft& Y, double& dt)
{

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

void Aircraft::graphing(const State& X, std::vector<double>& clock, std::vector<double>& pn, std::vector<double>& pe,  std::vector<double>& pd,  std::vector<double>& phi,  std::vector<double>& theta,  std::vector<double>& psi,  std::vector<double>& p,  std::vector<double>& q,  std::vector<double>& r,  std::vector<double>& V_m, std::vector<double>& alpha, std::vector<double>& beta) 
{
    
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

void Aircraft::rotate(const State& X, easy3d::vec3* vertices, const int& vertices_size, float& old_roll, float& old_pitch, float& old_yaw )
{

        float roll = static_cast<float>(X.phi);
        float pitch = static_cast<float>(X.theta);
        float yaw = static_cast <float>(X.psi);
        std::cout<< "Roll: "<<(roll*180/3.14) << "Pitch: "<<(pitch*180/3.14)<<" Yaw: "<<(yaw*180/3.14)<<"\n";
            
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

void Aircraft::translate(const State& X, easy3d::vec3* vertices, const int& vertices_size, float& old_pn, float& old_pe, float& old_pd)
{


float pn_float = static_cast<float>(X.pn);
float pe_float = static_cast<float>(X.pe);
float pd_float = static_cast<float>(X.pd);



// Position Update loop

            for (int j=0; j<vertices_size; ++j){
                vertices[j].x += pn_float;
                vertices[j].y += pe_float;
                vertices[j].z += pd_float;
                
            }     


}

void Aircraft::initializePreviousState() {
    old_roll = static_cast<float>(phi_0);
    old_pitch = static_cast<float>(theta_0);
    old_yaw = static_cast<float>(psi_0);
    
    old_pn = static_cast<float>(pn_0);
    old_pe = static_cast<float>(pe_0);
    old_pd = static_cast<float>(pd_0); 
}

void Aircraft::initializeVertices()
{
    vertices_x = {-4, -2, -2, -2, -2, 12, 0, 3, 3, 0, 9.5, 12, 12, 9.5, 9.5, 12};
    vertices_y = {0, 1, -1, -1, 1, 0, 5, 5, -5, -5, 2.5, 2.5, -2.5, -2.6, 0, 0};
    vertices_z = {0, -1, -1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5};
    vertices_size = vertices_x.size();
    std::cout<<"SIZE = "<<vertices_size<<"\n";

    // For scaling the thing
    aircraft_scale = 500;
    for (int i = 0; i < vertices_x.size()-1; i++)
    {
        vertices_x[i] = aircraft_scale*vertices_x[i];
        vertices_y[i] = aircraft_scale*vertices_y[i];
        vertices_z[i] = aircraft_scale*vertices_z[i];
    }
}

void Aircraft::createAircraftDrawable(easy3d::Viewer& viewer)
{
   std::cout<<"Reached createAircraftDrawable()"<<std::endl;
    
    aircraft = new easy3d::TrianglesDrawable("faces");
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

}

void Aircraft::createGridDrawable(easy3d::Viewer& viewer)
{

    std::cout<<"Reached createGrid"<<std::endl;

    // Create a LinesDrawable to visualize the 3D grid.
    gridDrawable = new easy3d::LinesDrawable("grid");
    std::cout << "Grid lines drawing created" <<"\n";

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

    //DEBUG
    for (const auto& vertex : grid_vertices) 
    {
        std::cout << "Vertex: " << vertex.x << ", " << vertex.y << ", " << vertex.z << std::endl;
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

    // Update the viewer
    viewer.update();


}

bool Aircraft::animate(easy3d::Viewer* viewer, Aircraft::State& state, double dt)
{
    (void)viewer;  

    // Map the vertex buffer into the client's address space
    void* pointer = easy3d::VertexArrayObject::map_buffer(GL_ARRAY_BUFFER, aircraft->vertex_buffer(), GL_WRITE_ONLY);

    easy3d::vec3* vertices = reinterpret_cast<easy3d::vec3*>(pointer);
    if (!vertices) {
        return false;
    }

    // Calculate forces and moments
    forces_moments(state, *this);
    std::cout << "State.Vm  " << state.V_m << "\n";
    std::cout << "state.fx: " << state.fx << "\n";
    std::cout << "state.fy: " << state.fy << "\n";
    std::cout << "state.fz " << state.fz << "\n";

    // Update dynamics
    dynamics(state, *this, dt);

    // Rotate and translate the aircraft
    rotate(state, vertices, vertices_size, old_roll, old_pitch, old_yaw);
    translate(state, vertices, vertices_size, old_pn, old_pe, old_pd);

    // Unmap the vertex buffer
    easy3d::VertexArrayObject::unmap_buffer(GL_ARRAY_BUFFER, aircraft->vertex_buffer());

    // Update the viewer
    viewer->update();

    return true;

}

void Aircraft::initializeVerticesIndices()
{
    std::cout<<"Reached initializeVerticesIndices()"<<std::endl;
     // Define the faces of the aircraft.
    faces = 
    {
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
    std::cout << "vertices_x size: " << vertices_x.size() << std::endl;
    std::cout << "vertices_y size: " << vertices_y.size() << std::endl;
    std::cout << "vertices_z size: " << vertices_z.size() << std::endl;
   

    // Populate the aircraftDrawable with the vertices and faces.
    

    // Add the vertices to the 'vertices' vector.
    for (size_t i = 0; i < vertices_x.size(); ++i) 
    {
        std::cout << "Vertex " << i << ": (" << vertices_x[i] << ", " << vertices_y[i] << ", " << vertices_z[i] << ")" << std::endl;

        
        vertices_aircraft.push_back(easy3d::vec3(vertices_x[i], vertices_y[i], vertices_z[i]));
    }
    std::cout<<"Reached BEFORE points line"<<std::endl;
    points = vertices_aircraft;
    std::cout<<"Reached AFTER points line"<<std::endl;
    // Add the faces to the 'indices' vector.

    std::cout << "Number of faces: " << faces.size() << std::endl;
    for (const auto& face : faces) 
    {
        indices.push_back(face[0]);
        indices.push_back(face[1]);
        indices.push_back(face[2]);
    }

    std::cout<<"Finished initializeVerticesIndices()"<<std::endl;
}