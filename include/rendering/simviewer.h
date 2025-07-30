/********************************************************************
 * @file simviewer.h
 * @brief Header for SimViewer, an extended Easy3D viewer with ImGui and aircraft rendering support.
 *
 * This class provides an interface for integrating an Aircraft object
 * and rendering it with GUI controls using ImGui.
 ********************************************************************/
#ifndef SIMVIEWER_H
#define SIMVIEWER_H 

#include <easy3d/viewer/viewer.h>
#include "viewer_imgui.h"
#include <implot.h>
#include <easy3d/renderer/text_renderer.h>
#include <easy3d/renderer/shadow.h>
#include <aircraft/aircraft.h>
/**
 * @class SimViewer
 * @brief Specialized Easy3D viewer class for UAV simulation with GUI integration.
 *
 * Inherits from `ViewerImGui` to enable ImGui-based user interface.
 * Allows for rendering an aircraft model and custom GUI panels.
 */
class SimViewer : public easy3d::ViewerImGui
{
public:
     /**
     * @brief Constructor.
     * @param title Title of the simulation window.
     */
    SimViewer(const std::string& title);
    /**
     * @brief Destructor.
     */
    ~SimViewer() override;
     /**
     * @brief Sets the aircraft to be displayed in the viewer.
     * @param aircraft Pointer to the Aircraft object to visualize.
     */
    void setAircraft(Aircraft* aircraft);

protected:
    /**
     * @brief Custom draw function called every frame.
     * Handles rendering of the aircraft and any overlays.
     */
    void draw() const override;
    /**
     * @brief Key press event handler.
     * @param key The GLFW key code.
     * @param modifiers Modifier keys (e.g., Shift, Ctrl).
     * @return true if the event is handled, false otherwise.
     */
    bool key_press_event(int key, int modifiers) override;

    //From ImGui
    /**
     * @brief Called before rendering each frame (for ImGui setup).
     */
    void pre_draw() override;
    /**
     * @brief Called after rendering each frame (for ImGui overlays).
     */
    void post_draw() override;
       
private:
    easy3d::TextRenderer* text_renderer_;   ///< Renders on-screen text such as FPS or labels.
    Aircraft* aircraft_ = nullptr;  ///< Pointer to the Aircraft object for simulation.


    // === GUI state flags ===
    bool show_controls_ = true; ///< If true, display control panel in ImGui.

    // For real-time plotting
    std::vector<float> time_data_;
    std::vector<std::vector<float>> state_data_ = std::vector<std::vector<float>>(18);
    float plot_time_ = 0.0f;
    
};





#endif 
