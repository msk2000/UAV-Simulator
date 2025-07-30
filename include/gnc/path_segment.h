/**
 * @file path_segment.h
 * @brief Abstract base class for different types of path segments.
 *
 * This class defines the interface for any path segment type, such as
 * straight lines, arcs, or curves. Each segment must implement
 * a method to generate waypoints along its geometry.
 */
#ifndef PATH_SEGMENT_H
#define PATH_SEGMENT_H

#include <vector>

#include <easy3d/core/vec.h>
#include "common.h"

/**
 * @class PathSegment
 * @brief Abstract base class representing a segment of a path.
 *
 * Defines the interface for generating discrete waypoints along the segment.
 */
class PathSegment
{
public:
    /**
     * @brief Virtual destructor for safe polymorphic destruction.
     */
    virtual ~PathSegment() = default;
    /**
     * @brief Generate waypoints along the path segment.
     *
     * This pure virtual function must be implemented by all derived
     * classes to provide a set of points spaced approximately by
     * the given distance along the segment.
     *
     * @param spacing Desired spacing between waypoints.
     * @return Vector of 3D points representing the waypoints along the segment.
     */
    virtual std::vector<easy3d::vec3> generateWaypoints(float spacing) const = 0;
};

#endif // PATH_SEGMENT_H
