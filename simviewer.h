#ifndef SIMVIEWER_H
#define SIMVIEWER_H 

#include <easy3d/viewer/viewer.h>
#include <easy3d/renderer/text_renderer.h>
#include <easy3d/renderer/shadow.h>
#include <aircraft.h>

class SimViewer : public easy3d::Viewer
{
public:
    SimViewer(const std::string& title);
    ~SimViewer() override;

    void setAircraft(Aircraft* aircraft);

protected:
    void draw() const override; 
    bool key_press_event(int key, int modifiers) override;
       
private:
    easy3d::TextRenderer* text_renderer_;
    Aircraft* aircraft_ = nullptr;

    
};





#endif 
