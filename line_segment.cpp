//line segment implementation file 
#include "line_segment.h"


LineSegment::LineSegment(const easy3d::vec3& start, const easy3d::vec3& end)
    : start_(start), end_(end) {}

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
