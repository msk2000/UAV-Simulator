/**
 * @file path.cpp
 * @brief Implementation of the Path class for managing and visualizing path segments in 3D space.
 */

#include "path.h"
#include "line_segment.h"
#include <easy3d/renderer/drawable_lines.h>

/**
 * @brief Adds a straight line segment to the path.
 *
 * This function creates a new LineSegment instance and appends it
 * to the internal list of path segments.
 *
 * @param start Start position of the line segment in 3D space.
 * @param end End position of the line segment in 3D space.
 */
void Path::addStraightSegment(const easy3d::vec3& start, const easy3d::vec3& end)
{
    segments.push_back(std::make_unique<LineSegment>(start, end));
}
/**
 * @brief Generates waypoints along all segments in the path.
 *
 * This method queries each segment in the path to compute evenly spaced waypoints,
 * then aggregates them into a single list.
 *
 * @param spacing Desired distance between consecutive waypoints.
 * @return A vector of 3D points representing the waypoints.
 */
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
/**
 * @brief Renders the entire path as a sequence of line segments in the viewer.
 *
 * This method converts the path into a set of line segments using sampled waypoints
 * and sends the data to an Easy3D `LinesDrawable` for visualization.
 *
 * @param viewer Reference to the Easy3D viewer instance.
 */

void Path::draw(easy3d::Viewer& viewer) const
{
    auto* drawable = new easy3d::LinesDrawable("path");

    std::vector<easy3d::vec3> vertices;
    for (const auto& segment : segments)
    {
        auto points = segment->generateWaypoints(10.0f);  // default sample resolution
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
