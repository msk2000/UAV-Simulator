/**
 * @file line_segment.h
 * @brief Defines the LineSegment class for a straight path segment.
 */
#ifndef LINE_SEGMENT_H
#define LINE_SEGMENT_H

#include "path_segment.h"
#include <easy3d/core/vec.h>
#include "common.h"

/**
 * @class LineSegment
 * @brief Represents a simple straight line segment between two 3D points.
 *
 * Inherits from PathSegment and implements waypoint generation along the line.
 */
class LineSegment : public PathSegment 
{
public:
    /**
     * @brief Constructs a LineSegment with specified start and end points.
     * @param start The starting point of the line segment.
     * @param end The ending point of the line segment.
     */
    LineSegment(const easy3d::vec3& start, const easy3d::vec3& end);
    /**
     * @brief Generates a set of waypoints spaced along the line segment.
     * @param spacing The distance between consecutive waypoints.
     * @return A vector of 3D points representing the waypoints on the segment.
     */

    std::vector<easy3d::vec3> generateWaypoints(float spacing) const override;

private:
    easy3d::vec3 start_; /**< Start point of the line segment */
    easy3d::vec3 end_;   /**< End point of the line segment */
};

#endif // LINE_SEGMENT_H
