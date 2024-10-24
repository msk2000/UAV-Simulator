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
size(400.0f),
numLines(22),
offset(200.0f),//10000.0f/2
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
    force_g << -mass * g*std::sin(theta),mass * g*std::cos(theta)*sin(phi),mass * g*std::cos(theta)*cos(phi);
    
    
    // Component: Aerodynamic
    force_aero1 = 0.5 * rho * (V_m*V_m) * wing_area;

  
    force_aero2 <<(CxAlpha+CxqAlpha*((wing_chord)/(2*V_m))*q)+CxdeltaeAlpha*delta_e,
        C_Y_0+(C_Y_beta*beta)+(C_Y_p*((wing_span)/(2*V_m))*p)+(C_Y_r*((wing_span)/(2*V_m))*r)+(C_Y_delta_a*delta_a)+(C_Y_delta_r*delta_r),
        CzAlpha+(CzqAlpha*((wing_chord)/(2*V_m))*q)+CzdeltaeAlpha*delta_e;
    
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

    
}
// Function to calculate the body-frame velocity, angle of attack and side slip
void Aircraft::calculate_body_frame_velocity_and_angles() 
{
    velocity_b = {u, v, w};
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
    
   
    
    Aero_t2 << wing_span*(C_ell_0 + (C_ell_beta*beta) + (C_ell_p*((wing_span)/(2*V_m))*p) + (C_ell_r*((wing_span)/(2*V_m))*r) + (C_ell_delta_a*delta_a)+(C_ell_delta_r*delta_r)),
    wing_chord*(C_m_0 + (C_m_alpha*alpha) + (C_m_q*((wing_chord)/(2*V_m))*q) + (C_m_delta_e*delta_e)),
    wing_span*(C_n_0 + (C_n_beta*beta) + (C_n_p*((wing_span)/(2*V_m))*p) + (C_n_r*((wing_span)/(2*V_m))*r) + (C_n_delta_a*delta_a)+(C_n_delta_r*delta_r));  
    

    Aero_torque = Aero_t1 * Aero_t2;
    
    // Moment/Torque due to propulsion system

    Prop_torque << -prop_thrust_coef * ((prop_omega*delta_t)*(prop_omega*delta_t)),0,0;
    
    
    // Total Moment/Torque
    Torque = Aero_torque + Prop_torque;
    
    
    // l m n - 3 components of the moment/torque
    ell = Torque[0]; // this is just "l", written this way for readability
    m = Torque[1];
    n = Torque[2];


 }





//Function to calculate the State changes (position, orientation,etc)
void Aircraft::calculate_dynamics(double& dt)
{

 calculate_position_rate(dt);
 calculate_velocity_rate(dt);
 calculate_orientation_rate(dt);
 calculate_angular_rate(dt);
 update_state(dt);

}

// Function to calculate delta_position
void Aircraft::calculate_position_rate(double& dt)
{
    pn_dot = 
    u*std::cos(psi)*std::cos(theta) 
    + v*(std::cos(psi)*std::sin(phi)*std::sin(theta) - std::cos(phi)*std::sin(psi)) 
    + w*(std::sin(phi)*std::sin(psi) + std::cos(phi)*std::cos(psi)*std::sin(theta)) ;
    
    pe_dot = 
    u*std::cos(theta)*std::sin(psi) 
    + v*(std::cos(phi)*std::cos(psi) + std::sin(phi)*std::sin(psi)*std::sin(theta)) 
    + w*(std::cos(phi)*std::sin(psi)*std::sin(theta) - std::cos(psi)*std::sin(phi));

    pd_dot = 
    - u*std::sin(theta) 
    + v*std::cos(theta)*std::sin(phi) 
    + w*std::cos(phi)*std::cos(theta) ;
}

// Function to calculate the angular changes of the orientation
void Aircraft::calculate_orientation_rate(double& dt)
{
    phi_dot = p + r*std::cos(phi)*std::tan(theta) + q*std::sin(phi)*std::tan(theta);
    theta_dot = q*std::cos(phi) - r*std::sin(phi);
    psi_dot = (r*std::cos(phi))/std::cos(theta) + (q*std::sin(phi))/std::cos(theta);
}

// Function to calculate the rate of change of velocity components
void Aircraft::calculate_velocity_rate(double& dt)
{   
     u_dot = (r*v - q*w)+(fx/mass);
     v_dot = (p*w - r*u)+(fy/mass);
     w_dot = (q*u - p*v)+(fz/mass);
} 

// Function to calculate the angular rates
void Aircraft::calculate_angular_rate(double& dt)
{
     p_dot = Gamma_1*p*q - Gamma_2*q*r + Gamma_3*ell + Gamma_4*n;
    q_dot = Gamma_5*p*r - Gamma_6*((p*p)-(r*r)) + (m/Jy); 
    r_dot = Gamma_7*p*q - Gamma_1*q*r + Gamma_4*ell + Gamma_8*n;
}

// Function to update the state parameters of the UAV
void Aircraft::update_state(double& dt)
{
    // Making this the next initial values
    pn =  pn + pn_dot * dt;
    pe = pe + pe_dot * dt;
    pd = pd + pd_dot * dt;
    u = u + u_dot * dt;
    v = v + v_dot * dt;
    w = w + w_dot * dt;
    phi = phi + phi_dot * dt;
    theta = theta + theta_dot * dt;
    psi =  psi + psi_dot * dt;
    p = p + p_dot * dt;
    q = q + q_dot * dt;
    r = r + r_dot * dt;
    clock++;

}

// Function to apply RK4 to state update
void Aircraft::RK4(double& dt)
{
    // Temporary variables to hold intermediate slopes
    double k1_pn_dot;
    double k2_pn_dot;
    double k3_pn_dot;
    double k4_pn_dot;
    
    // Step 1: Evaluate k1 (slope at the current time)
    k1_pn_dot = pn_dot;

    // Step 2: Evaluate k2 (slope at the midpoint, using k1)
    double pn_temp = pn + 0.5 * dt * k1_pn_dot;
    k2_pn_dot = // Calculate pn_dot at t + 0.5*dt using updated pn_temp

    // Step 3: Evaluate k3 (another midpoint slope, using k2)
    pn_temp = pn + 0.5 * dt * k2_pn_dot;
    k3_pn_dot = // Calculate pn_dot at t + 0.5*dt using updated pn_temp

    // Step 4: Evaluate k4 (slope at the end of the interval, using k3)
    pn_temp = pn + dt * k3_pn_dot;
    k4_pn_dot = // Calculate pn_dot at t + dt using updated pn_temp

    // Combine the slopes to compute the next state
    pn = pn + (dt / 6.0) * (k1_pn_dot + 2*k2_pn_dot + 2*k3_pn_dot + k4_pn_dot);

    // Repeat for pe, pd, u, v, w, phi, theta, psi, p, q, r in a similar manner
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
    easy3d::Mat3<float> rotationMatrix = easy3d::Mat3<float>::rotation(phi, theta , psi, 321);
        
    // Apply the rotation to the vertices and hope that it actually works
        for (int i = 0; i < mesh->n_vertices(); ++i) {
            vertices[i] = rotationMatrix * vertices[i];
        }

}

// Function to perform rotation on the axis vertices
void Aircraft::rotate_axes(easy3d::vec3* axesVertices)
{
    // Create the rotation matrix using Euler angles (same rotation as aircraft)
    easy3d::Mat3<float> rotationMatrix = easy3d::Mat3<float>::rotation(phi, theta, psi, 321);
    
    // Assuming we have 6 vertices for the 3 axes (X, Y, Z), rotate them
    for (int i = 0; i < 6; ++i) {
        axesVertices[i] = rotationMatrix * axesVertices[i];
    }
}


// Function to perform translation of the aircraft geometry
void Aircraft::translate(easy3d::vec3* vertices)
{

    easy3d::vec3 translationVector(static_cast<float>(pn), static_cast<float>(pe), static_cast<float>(pd));


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
    easy3d::vec3 translationVector(static_cast<float>(pn), static_cast<float>(pe), static_cast<float>(pd));

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
    easy3d::vec3(pn, pe, pd), // Origin
    easy3d::vec3(pn - 50.0f, pe, pd), // X-axis endpoint (moving in negative x-direction)
    
    // Y-axis 
    easy3d::vec3(pn, pe, pd), // Origin
    easy3d::vec3(pn, pe + 50.0f, pd), // Y-axis endpoint (moving in positive y-direction)
    
    // Z-axis
    easy3d::vec3(pn, pe, pd), // Origin
    easy3d::vec3(pn, pe, pd - 50.0f)  // Z-axis endpoint (moving downward in negative z-direction)
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
    calculate_dynamics(dt);

    // Perform rotation and translation on the geometry
    rotate(vertices);
    translate(vertices);
    rotate_axes(axesVertices);
    translate_axes(axesVertices);

    //std::cout << pn << "\t" <<pe << "\t" << pd << std::endl;


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
            pn += 100;  // Increment the value
            break;
        case '8':
            pn -= 100;; // Decrement the value
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

/*easy3d::Mat3<float> Aircraft::transpose(const easy3d::Mat3<float>& matrix) 
{    
    easy3d::Mat3<float> transposed;    
    for (int i = 0; i < 3; ++i) 
    {        
        for (int j = 0; j < 3; ++j) 
        {            
            transposed(i, j) = matrix(j, i);  // Swap indices for transposition 
        }    
    }    
    return transposed;
}*/