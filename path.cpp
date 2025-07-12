// Path implementation file 
#include "path.h"
#include "line_segment.h"
#include <easy3d/renderer/drawable_lines.h>

void Path::addStraightSegment(const easy3d::vec3& start, const easy3d::vec3& end)
{
    segments.push_back(new LineSegment(start, end));
}

std::vector<easy3d::vec3> Path::generateWaypoints(float spacing) const
{
    std::vector<easy3d::vec3> waypoints;

    for (const auto& segment : segments) 
    {
        auto segment_waypoints = segment->generateWaypoints(spacing);
        waypoints.insert(waypoints.end(), segment_waypoints.begin(), segment_waypoints.end());
    }

    return waypoints;
}

void Path::draw(easy3d::Viewer& viewer) const
{
    auto* drawable = new easy3d::LinesDrawable("path");

    std::vector<easy3d::vec3> vertices;
    for (const auto& segment : segments)
    {
        auto points = segment->generateWaypoints(10.0f);  // For simplicity
        for (size_t i = 0; i + 1 < points.size(); ++i)
        {
            vertices.push_back(points[i]);
            vertices.push_back(points[i + 1]);
        }
    }

    drawable->update_vertex_buffer(vertices);
    drawable->set_uniform_coloring(easy3d::vec4(1.0f, 0.0f, 0.0f, 1.0f));
    drawable->set_line_width(4.0f);

    viewer.add_drawable(drawable);
    viewer.update();
}
