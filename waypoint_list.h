/**
 * @file waypoint_list.h
 * @brief Defines the WaypointList class that manages a sequence of 3D waypoints for navigation or mission planning.
 */
#ifndef WAYPOINT_LIST_H
#define WAYPOINT_LIST_H

#include <vector>

#include <easy3d/core/vec.h>
#include <easy3d/viewer/viewer.h>

/**
 * @class WaypointList
 * @brief Stores and manages a list of 3D waypoints for trajectory following or mission execution.
 *
 * This class is responsible for holding a sequence of spatial waypoints (`easy3d::vec3`) and tracking progress
 * through them. It allows you to:
 * - Retrieve the current waypoint
 * - Move to the next waypoint
 * - Check if all waypoints are completed
 * - Visualize them in an Easy3D viewer
 */
class WaypointList
{
public:
    /**
     * @brief Constructor to initialize the waypoint list.
     * @param waypoints A vector of 3D coordinates representing waypoints in space.
     */
    WaypointList(const std::vector<easy3d::vec3>& waypoints);
    /**
     * @brief Returns the currently active waypoint.
     * @return A constant reference to the current waypoint.
     */

    const easy3d::vec3& currentWaypoint() const;
    /**
     * @brief Advances the pointer to the next waypoint.
     * If already at the last waypoint, it will stay there.
     */
    void advanceToNext();
    /**
     * @brief Checks whether all waypoints have been processed.
     * @return True if the mission is complete (i.e., no more waypoints left).
     */
    bool missionComplete() const;
    /**
     * @brief Renders the waypoints in the provided Easy3D viewer.
     * @param viewer Reference to the Easy3D viewer for rendering the points.
     */
    void draw(easy3d::Viewer& viewer) const;

private:
    std::vector<easy3d::vec3> waypoints_;   ///< List of waypoints in 3D space.
    size_t current_index_;  ///< Index of the current active waypoint.
};

#endif // WAYPOINT_LIST_H
