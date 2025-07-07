// This is the main program from which you setup/run the simulation
#include <iostream>
#include <aircraft.h>
#include <world.h>
#include <plotter.h>
#include <easy3d/viewer/viewer.h>
#include <easy3d/renderer/camera.h>
#include <easy3d/core/types.h>
#include <easy3d/util/resource.h>
#include <easy3d/util/initializer.h>
#include <easy3d/renderer/text_renderer.h>
#include <easy3d/renderer/shadow.h> 

class TestViewer : public easy3d::Viewer 
{
public:
    TestViewer(const std::string& title)
        : easy3d::Viewer(title), text_renderer_(new easy3d::TextRenderer(dpi_scaling())) 
    {
        if (!text_renderer_->add_font("/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf")) 
        {
            std::cerr << "Failed to load font!" << std::endl;
        }

    }

    ~TestViewer() override 
    {
        delete text_renderer_;
    }

    void setAircraft(Aircraft* aircraft) { aircraft_ = aircraft; }

protected:
    void draw() const override 
    {
        if(!aircraft_) { return;}

        // Draw the 3D scene first
        easy3d::Viewer::draw();

               // Draw the aircraft HUD
        if (aircraft_ && text_renderer_)
            aircraft_->render_HUD(*text_renderer_, const_cast<TestViewer*>(this));
    }

    bool key_press_event(int key, int modifiers) override 
    {
        if (!aircraft_)
        {
            return false;
        }

        double control_step = 1;

        switch (key) {
            case KEY_1:
                aircraft_->delta_a = std::min(aircraft_->delta_a + control_step, aircraft_->delta_a_max);
                break;
            case KEY_3:
                aircraft_->delta_a = std::max(aircraft_->delta_a - control_step, aircraft_->delta_a_min);
                break;
            case KEY_5:
                aircraft_->delta_e = std::min(aircraft_->delta_e + control_step, aircraft_->delta_e_max);
                break;
            case KEY_2:
                aircraft_->delta_e = std::max(aircraft_->delta_e - control_step, aircraft_->delta_e_min);
                break;
            case KEY_4:
                aircraft_->delta_r = std::min(aircraft_->delta_r + control_step, aircraft_->delta_r_max);
                break;
            case KEY_6:
                aircraft_->delta_r = std::max(aircraft_->delta_r - control_step, aircraft_->delta_r_min);
                break;
            case KEY_EQUAL:  // '+' key
                aircraft_->delta_t = std::min(aircraft_->delta_t + 0.05, aircraft_->delta_t_max);
                break;
            case KEY_MINUS:  // '-' key
                aircraft_->delta_t = std::max(aircraft_->delta_t - 0.05, aircraft_->delta_t_min);
                break;
            case KEY_7:
                aircraft_->X[0] += 1;
                break;
            case KEY_8:
                aircraft_->X[0] -= 1;
                break;
            default:
                return Viewer::key_press_event(key, modifiers);  // Pass to base class
        }

        return true;  // Key event handled
    }

private:
    easy3d::TextRenderer* text_renderer_;
    Aircraft* aircraft_ = nullptr;

    
};



int main()
{
    // Simulation parameters
    int steps = 10;
    double dt = 0.0001;
    int vehicle_count = 1;
    std::string fname = "../data.txt";

    Aircraft drone(fname, vehicle_count);
    World world;
    drone.steps = steps;
    drone.dt = dt;
    //drone.initKeyboard();

    easy3d::initialize(true);

    // Use TestViewer instead of Viewer
    TestViewer viewer("UAV Simulator");
    viewer.setAircraft(&drone);  // pass the UAV to the viewer
    viewer.set_background_color(easy3d::vec4(0.0f, 0.0f, 0.0f, 1.0f));

    // Load and render the model
    drone.file_name = "/home/fahim/Coding/Git/UAV-Simulator/y_for_z_up_uav2.stl";
    drone.mesh = easy3d::SurfaceMeshIO::load(drone.file_name);

    world.createTerrainWithTexture(viewer);
    drone.renderAircraft(viewer);
    drone.createGridDrawable(viewer);
    drone.createAxesDrawable(viewer);

    //viewer.fit_screen();
    viewer.set_animation(true);

    viewer.animation_func_ = [&](easy3d::Viewer* v) -> bool
    {
        drone.clock++;
        return drone.animate(v, dt);
    };

    return viewer.run();
}


