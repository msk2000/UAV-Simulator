//Implementation file for waypoint list 
#include "waypoint_list.h"
#include <easy3d/renderer/drawable_points.h>

WaypointList::WaypointList(const std::vector<easy3d::vec3>& waypoints)
    : waypoints_(waypoints), current_index_(0) {}

const easy3d::vec3& WaypointList::currentWaypoint() const
{
    return waypoints_[current_index_];
}

void WaypointList::advanceToNext() 
{
    if (current_index_ + 1 < waypoints_.size())
        ++current_index_;
}

bool WaypointList::missionComplete() const 
{
    return current_index_ >= waypoints_.size() - 1;
}

void WaypointList::draw(easy3d::Viewer& viewer) const
{
    auto* drawable = new easy3d::PointsDrawable("waypoints");

    drawable->update_vertex_buffer(waypoints_);
    drawable->set_uniform_coloring(easy3d::vec4(1.0f, 1.0f, 0.0f, 1.0f));
    drawable->set_point_size(5.0f);

    viewer.add_drawable(drawable);
    viewer.update();
}
