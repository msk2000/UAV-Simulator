//header file for waypoint list 
#ifndef WAYPOINT_LIST_H
#define WAYPOINT_LIST_H

#include <vector>

#include <easy3d/core/vec.h>
#include <easy3d/viewer/viewer.h>

// Holds discrete waypoints to follow
class WaypointList
{
public:
    WaypointList(const std::vector<easy3d::vec3>& waypoints);

    const easy3d::vec3& currentWaypoint() const;
    void advanceToNext();
    bool missionComplete() const;
    void draw(easy3d::Viewer& viewer) const;

private:
    std::vector<easy3d::vec3> waypoints_;
    size_t current_index_;
};

#endif // WAYPOINT_LIST_H
