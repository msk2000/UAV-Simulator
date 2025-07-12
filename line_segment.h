// line segment header file 
#ifndef LINE_SEGMENT_H
#define LINE_SEGMENT_H

#include "path_segment.h"
#include <easy3d/core/vec.h>
#include "common.h"

// Simple straight line segment between two points
class LineSegment : public PathSegment 
{
public:
    LineSegment(const easy3d::vec3& start, const easy3d::vec3& end);

    std::vector<easy3d::vec3> generateWaypoints(float spacing) const override;

private:
    easy3d::vec3 start_, end_;
};

#endif // LINE_SEGMENT_H
