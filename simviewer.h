#ifndef SIMVIEWER_H
#define SIMVIEWER_H 

#include <easy3d/viewer/viewer.h>
#include "viewer_imgui.h"
#include <easy3d/renderer/text_renderer.h>
#include <easy3d/renderer/shadow.h>
#include <aircraft.h>

class SimViewer : public easy3d::ViewerImGui
{
public:
    SimViewer(const std::string& title);
    ~SimViewer() override;

    void setAircraft(Aircraft* aircraft);

protected:
    void draw() const override; 
    bool key_press_event(int key, int modifiers) override;

    //From ImGui
    void pre_draw() override;
    void post_draw() override;
       
private:
    easy3d::TextRenderer* text_renderer_;
    Aircraft* aircraft_ = nullptr;

    // Optional GUI variables
    bool show_controls_ = true;
    
};





#endif 
