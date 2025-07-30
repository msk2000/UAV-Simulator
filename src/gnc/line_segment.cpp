/**
 * @file line_segment.cpp
 * @brief Implementation of the LineSegment class for a straight path segment.
 */
#include "line_segment.h"

/**
 * @brief Constructs a LineSegment with specified start and end points.
 * @param start The starting point of the line segment.
 * @param end The ending point of the line segment.
 */
LineSegment::LineSegment(const easy3d::vec3& start, const easy3d::vec3& end)
    : start_(start), end_(end) {}

/**
 * @brief Generates waypoints spaced along the line segment.
 *
 * The function calculates equally spaced points starting from
 * the start point up to the end point based on the given spacing.
 *
 * @param spacing Distance between consecutive waypoints.
 * @return std::vector<easy3d::vec3> A vector containing the generated waypoints.
 */
std::vector<easy3d::vec3> LineSegment::generateWaypoints(float spacing) const
{
    std::vector<easy3d::vec3> waypoints;

    float distance = (end_ - start_).norm();
    int steps = static_cast<int>(distance / spacing);

    for (int i = 0; i <= steps; ++i)
    {
        float t = static_cast<float>(i) / steps;
        waypoints.push_back(start_ + t * (end_ - start_));
    }

    return waypoints;
}
