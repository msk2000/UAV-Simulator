// SimViewer class implementation
#include <simviewer.h>

//Constructor
SimViewer::SimViewer(const std::string& title)
        : easy3d::Viewer(title), text_renderer_(new easy3d::TextRenderer(dpi_scaling())) 
    {
        if (!text_renderer_->add_font("/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf")) 
        {
            std::cerr << "Failed to load font!" << std::endl;
        }

    }
//Destructor
SimViewer::~SimViewer() 
    {
        delete text_renderer_;
    }

    void SimViewer::setAircraft(Aircraft* aircraft) { aircraft_ = aircraft; }

    void SimViewer::draw() const 
    {
        if(!aircraft_) { return;}

        // Draw the 3D scene first
        easy3d::Viewer::draw();

               // Draw the aircraft HUD
        if (aircraft_ && text_renderer_)
            aircraft_->render_HUD(*text_renderer_, const_cast<SimViewer*>(this));
    }

    bool SimViewer::key_press_event(int key, int modifiers) 
    {
        if (!aircraft_)
        {
            return false;
        }

        double control_step = 1;

        switch (key)
        {
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


