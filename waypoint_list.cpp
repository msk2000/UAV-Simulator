/**
 * @file waypoint_list.cpp
 * @brief Implements the WaypointList class for managing and visualizing 3D waypoints in a UAV simulation.
 */
#include "waypoint_list.h"
#include <easy3d/renderer/drawable_points.h>


WaypointList::WaypointList() : waypoints_(), current_index_(0) {}
/**
 * @brief Constructs a WaypointList with the provided set of waypoints.
 * @param waypoints A vector of 3D points representing the mission trajectory.
 */
WaypointList::WaypointList(const std::vector<easy3d::vec3>& waypoints)
    : waypoints_(waypoints), current_index_(0) {}
/**
 * @brief Retrieves the current active waypoint in the mission.
 * @return A reference to the current waypoint (as a 3D vector).
 */
const easy3d::vec3& WaypointList::currentWaypoint() const
{
    return waypoints_[current_index_];
}
/**
 * @brief Advances the mission to the next waypoint, if available.
 * If the current waypoint is the last one, no change occurs.
 */
void WaypointList::advanceToNext() 
{
    if (current_index_ + 1 < waypoints_.size())
        ++current_index_;
}
/**
 * @brief Checks whether the mission has reached or passed the final waypoint.
 * @return True if the current waypoint is the last in the list; otherwise, false.
 */
bool WaypointList::missionComplete() const 
{
    return current_index_ >= waypoints_.size() - 1;
}
/**
 * @brief Visualizes all waypoints in the viewer as yellow points.
 *
 * This function adds a `PointsDrawable` to the viewer to render
 * all mission waypoints. Each point appears with a fixed size and color.
 *
 * @param viewer Reference to the Easy3D viewer for rendering.
 */
void WaypointList::draw(easy3d::Viewer& viewer) const
{
    auto* drawable = new easy3d::PointsDrawable("waypoints");
    // Upload all waypoints to the GPU
    drawable->update_vertex_buffer(waypoints_);
    // Set  color for all points
    drawable->set_uniform_coloring(easy3d::vec4(1.0f, 1.0f, 0.0f, 1.0f));
    drawable->set_point_size(5.0f); // Set point size (in pixels)

    viewer.add_drawable(drawable);
    viewer.update();
}
