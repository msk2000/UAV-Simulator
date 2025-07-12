// Header file for path ====================
#ifndef PATH_H
#define PATH_H

#include <vector>
#include "path_segment.h"
#include <easy3d/core/vec.h>
#include <easy3d/viewer/viewer.h>

// Path = a sequence of segments (lines, arcs, etc.)
class Path 
{
public:
    void addStraightSegment(const easy3d::vec3& start, const easy3d::vec3& end);

    // Generate discrete waypoints along the entire path
    std::vector<easy3d::vec3> generateWaypoints(float spacing) const;

    //visualising path
    void draw(easy3d::Viewer& viewer) const;

private:
    std::vector<PathSegment*> segments;
};

#endif // PATH_H
