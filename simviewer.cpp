/**
 * @file simviewer.cpp
 * @brief Implementation of the SimViewer class, a GUI-enabled Easy3D viewer for UAV simulation.
 */
#include <simviewer.h>
#include <3rd_party/imgui/imgui.h>
#include <implot.h>

/**
 * @brief Constructor for SimViewer.
 * Initializes the Easy3D viewer with ImGui support and sets up a text renderer.
 * @param title Title of the window.
 */
SimViewer::SimViewer(const std::string& title)
        :ViewerImGui(title, 4, 3, 2, false /* fullscreen */, true, 24, 8, 1920, 1080), text_renderer_(new easy3d::TextRenderer(dpi_scaling()))
    {   // Load font for HUD rendering
        if (!text_renderer_->add_font("/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf")) 
        {
            std::cerr << "Failed to load font!" << std::endl;
        }


    }
/**
 * @brief Destructor for SimViewer.
 * Cleans up text renderer memory.
 */
SimViewer::~SimViewer() 
    {
        delete text_renderer_;
    }

    /**
 * @brief Assigns an aircraft object to the viewer.
 * @param aircraft Pointer to the Aircraft instance used in simulation and rendering.
 */
    void SimViewer::setAircraft(Aircraft* aircraft) { aircraft_ = aircraft; }
/**
 * @brief Main rendering function.
 * Called every frame to render 3D geometry and aircraft HUD.
 */
    void SimViewer::draw() const 
    {
        if(!aircraft_) { return;}

        // Draw the 3D scene first
        easy3d::Viewer::draw();

               // Draw the aircraft HUD
        if (aircraft_ && text_renderer_)
            aircraft_->render_HUD(*text_renderer_, const_cast<SimViewer*>(this));
    }
/**
 * @brief Handles keyboard input for controlling aircraft.
 * @param key GLFW key code
 * @param modifiers Key modifiers (Shift, Ctrl, etc.)
 * @return true if the key was handled, false if passed to base class.
 */
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
/**
 * @brief Called after the main draw function.
 * Displays ImGui control panel for aircraft control surfaces.
 * Displays Real-Time plots of the UAV state variables
 */
    void SimViewer::post_draw()
    {
        ImGuiIO& io = ImGui::GetIO();
        float offset_x = 10.0f;  // horizontal offset from left edge
        float offset_y = 10.0f;  // vertical offset from bottom edge

        // Position bottom-left, offset by offset_x and offset_y
        ImGui::SetNextWindowPos(ImVec2(offset_x, io.DisplaySize.y - offset_y), ImGuiCond_Always, ImVec2(0.0f, 1.0f));
        ImGui::SetNextWindowCollapsed(true, ImGuiCond_Once); // window stays collapsed by default

        // Push dark theme colors
        // Control Panel Background and Text
        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.05f, 0.05f, 0.05f, 1.0f));  // near black
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.85f, 0.85f, 0.85f, 1.0f));      // light gray

        // Frame background (slider track)
        ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.15f, 0.15f, 0.15f, 1.0f));   // dark gray

        // Frame background when hovered
        ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, ImVec4(0.25f, 0.25f, 0.25f, 1.0f));

        // Active slider handle
        ImGui::PushStyleColor(ImGuiCol_SliderGrab, ImVec4(0.3f, 0.7f, 1.0f, 1.0f));   // bright cyan
        ImGui::PushStyleColor(ImGuiCol_SliderGrabActive, ImVec4(1.0f, 0.6f, 0.2f, 1.0f));  // orange



        if (ImGui::Begin("Controls", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
        {

            if (aircraft_)
            {
                float throttle = static_cast<float>(aircraft_->delta_t);
                float elevator = static_cast<float>(aircraft_->delta_e);
                float aileron  = static_cast<float>(aircraft_->delta_a);
                float rudder   = static_cast<float>(aircraft_->delta_r);

                float t_min = static_cast<float>(aircraft_->delta_t_min);
                float t_max = static_cast<float>(aircraft_->delta_t_max);
                float e_min = static_cast<float>(aircraft_->delta_e_min);
                float e_max = static_cast<float>(aircraft_->delta_e_max);
                float a_min = static_cast<float>(aircraft_->delta_a_min);
                float a_max = static_cast<float>(aircraft_->delta_a_max);
                float r_min = static_cast<float>(aircraft_->delta_r_min);
                float r_max = static_cast<float>(aircraft_->delta_r_max);

                if (ImGui::SliderFloat("Throttle", &throttle, t_min, t_max))
                    aircraft_->delta_t = throttle;
                if (ImGui::SliderFloat("Elevator", &elevator, e_min, e_max))
                    aircraft_->delta_e = elevator;
                if (ImGui::SliderFloat("Aileron", &aileron, a_min, a_max))
                    aircraft_->delta_a = aileron;
                if (ImGui::SliderFloat("Rudder", &rudder, r_min, r_max))
                    aircraft_->delta_r = rudder;
            }

        }
        // Pop the style colors

        ImGui::PopStyleColor(6);

        ImGui::End();

        // === Realtime plot for aircraft state ===
        // Get main viewport size

        float menu_bar_height = 35.0f;

        // Anchor right edge of window to right edge of screen, offset from top by menu height
        ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x, menu_bar_height), ImGuiCond_Always, ImVec2(1.0f, 0.0f));
        ImGui::SetNextWindowCollapsed(true, ImGuiCond_Once); // window stays collapsed by default

        // Push dark theme colors
        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.0f, 0.0f, 0.0f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.8f, 0.8f, 0.8f, 1.0f));
        ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(1.0f, 0.0f, 0.0f, 1.0f));
        ImPlot::PushStyleColor(ImPlotCol_Fill, ImVec4(0.0f, 0.0f, 0.0f, 1.0f));
        ImPlot::PushStyleColor(ImPlotCol_AxisText, ImVec4(0.8f, 0.8f, 0.8f, 1.0f));
        ImPlot::PushStyleColor(ImPlotCol_FrameBg,    ImVec4(0.0f, 0.0f, 0.0f, 1.0f));


        if (ImGui::Begin("Realtime UAV State Plot", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
        {
            if (aircraft_)
            {
                float dt = 0.01f;  /// TODO get directly from sim
                plot_time_ += dt;

                // Push new state values
                time_data_.push_back(plot_time_);
                for (int i = 0; i < 18; ++i)
                    state_data_[i].push_back(aircraft_->X[i]);

                // Maintain sliding time window
                const float history_span = 10.0f;
                while (!time_data_.empty() && time_data_.front() < plot_time_ - history_span)
                {
                    time_data_.erase(time_data_.begin());
                    for (auto& vec : state_data_)
                        vec.erase(vec.begin());
                }

                // Labels for each state variable
                const char* labels[18] =
                {
                    "pn","pe","pd","u","v","w","p","q","r",
                    "phi","theta","psi","fx","fy","fz","ell","m","n"
                };

                // Begin the table ONCE
                if (ImGui::BeginTable("PlotTable", 3, ImGuiTableFlags_BordersInnerV))
                {
                    for (int i = 0; i < 18; ++i)
                    {
                        ImGui::TableNextColumn();  // Move to next cell

                        std::string label = labels[i];
                        std::string plot_title = label + "##" + labels[i];  // Unique plot ID

                        if (ImPlot::BeginPlot(plot_title.c_str(), ImVec2(-1, 150), ImPlotFlags_NoLegend))
                        {
                            ImPlot::SetupAxes("Time (s)", label.c_str(), ImPlotAxisFlags_NoTickLabels, ImPlotAxisFlags_AutoFit);
                            ImPlot::SetupAxisLimits(ImAxis_X1, plot_time_ - history_span, plot_time_, ImGuiCond_Always);
                            ImPlot::PlotLine(label.c_str(), time_data_.data(), state_data_[i].data(), static_cast<int>(time_data_.size()));
                            ImPlot::EndPlot();
                        }
                    }
                    ImGui::EndTable();
                }
            }
        }
         // Pop the style colors
        ImGui::PopStyleColor(2);
        ImPlot::PopStyleColor(3);

        ImGui::End();


    ViewerImGui::post_draw(); // Keep Easy3D overlays (logo, FPS, etc.)


    show_frame_rate_ = true;

     if (texter_) 
     {
        const float font_size = 25.0f;

        // Position the UAV title at the top-left
        if (true) 
        {
            float offset_x = 20.0f * dpi_scaling();
            float offset_y = (20.0f + menu_height_) * dpi_scaling();
            texter_->draw("UAV Simulator", offset_x, offset_y, font_size, 0, easy3d::vec3(1.0f, 1.0f, 1.0f));
        }

        // Draw FPS (bottom-left or somewhere visible)
        if (show_frame_rate_) 
        {
            const std::string fps = framerate_; // make sure it's valid
            float offset_x = 20.0f * dpi_scaling();
            float offset_y = (100.0f + menu_height_) * dpi_scaling();  // try higher y if it’s hidden
            texter_->draw(fps, offset_x, offset_y, font_size, 1, easy3d::vec3(1.0f, 1.0f, 1.0f));
        }
    }
   
    }
/**
 * @brief Called before each frame is drawn.
 * Ensures ImGui and Easy3D are synchronized.
 */
void SimViewer::pre_draw()
{
    ViewerImGui::pre_draw();  // Optional: keeps Easy3D behavior
}




