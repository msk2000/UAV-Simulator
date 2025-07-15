/**
 * @file path.h
 * @brief Defines the Path class for managing and visualizing sequences of path segments in 3D space.
 */
#ifndef PATH_H
#define PATH_H

#include <vector>
#include "path_segment.h"
#include <easy3d/core/vec.h>
#include <easy3d/viewer/viewer.h>

/**
 * @class Path
 * @brief Represents a sequence of connected path segments (e.g., straight lines, arcs).
 *
 * This class allows the construction of a path using various types of segments
 * and supports generating discrete waypoints and visualizing the full path.
 */
class Path 
{
public:
    /**
     * @brief Adds a straight line segment to the path.
     *
     * @param start The 3D start point of the segment.
     * @param end The 3D end point of the segment.
     */
    void addStraightSegment(const easy3d::vec3& start, const easy3d::vec3& end);

    /**
     * @brief Generates a list of evenly spaced waypoints along the entire path.
     *
     * @param spacing Distance (in world units) between consecutive waypoints.
     * @return A vector of 3D waypoints covering all segments in the path.
     */
    std::vector<easy3d::vec3> generateWaypoints(float spacing) const;

    /**
     * @brief Renders the full path in the viewer.
     *
     * This method draws all path segments currently stored in the path.
     * Each segment handles its own rendering logic (e.g., line, arc).
     *
     * @param viewer Reference to the Easy3D viewer used for rendering.
     */
    void draw(easy3d::Viewer& viewer) const;

private:
    /// The sequence of segments (e.g., lines, arcs) that make up the full path.
    std::vector<PathSegment*> segments;
};

#endif // PATH_H
