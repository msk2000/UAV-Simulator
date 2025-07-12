// Header file for GNC 
#ifndef GNC_H
#define GNC_H

#include "waypoint_list.h"
#include "aircraft.h"
#include <easy3d/core/vec.h>
#include "common.h"

// Guidance, Navigation, and Control system
class GNC 
{
public:
    void setWaypoints(const WaypointList& waypoints);
    void update(const Aircraft& aircraft, float dt);

private:
    WaypointList waypoints_;
};

#endif // GNC_H
