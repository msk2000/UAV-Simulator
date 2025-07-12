// Path Segment header file ====================
#ifndef PATH_SEGMENT_H
#define PATH_SEGMENT_H

#include <vector>

#include <easy3d/core/vec.h>
#include "common.h"

// Base class for any path segment
class PathSegment
{
public:
    virtual ~PathSegment() = default;
    virtual std::vector<easy3d::vec3> generateWaypoints(float spacing) const = 0;
};

#endif // PATH_SEGMENT_H
