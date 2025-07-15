#ifndef WORLD_H
#define WORLD_H

#include <Eigen/Dense>
#include <easy3d/viewer/viewer.h>
#include <easy3d/renderer/drawable_triangles.h>
#include <easy3d/renderer/drawable_lines.h>
#include <easy3d/renderer/vertex_array_object.h>
#include <easy3d/core/surface_mesh.h>
#include <easy3d/fileio/surface_mesh_io.h>
#include <easy3d/renderer/texture.h>
#include <easy3d/renderer/texture_manager.h>
#include <easy3d/renderer/renderer.h>

class World 
{
public:
    World();
    void createGround(easy3d::Viewer& viewer); //Flat ground
    void createTerrain(easy3d::Viewer& viewer);  // 3d terrain
    void createTerrainWithTexture(easy3d::Viewer& viewer); // textured terrain
    void createGridDrawable(easy3d::Viewer& viewer); // Creates the 3d grid system

private:
    // === Default parameters for the 3D grid visualization ===

    // The total length of each axis in the grid (in world units).
    // The grid spans from -size/2 to +size/2 along each axis plane.
    // Increase this to make the grid larger, decrease for a smaller grid.
    static constexpr float DEFAULT_GRID_SIZE = 6000.0f;

    // The offset that shifts the grid's location along one or more axes.
    // In the provided grid creation logic, this is used to "lift" the grid away from the world origin
    // so that it doesn't obscure other objects like the aircraft.
    static constexpr float DEFAULT_OFFSET = 3000.0f;

    // The number of lines to draw along each axis.
    // Controls the resolution (density) of the grid.
    // A higher number results in finer divisions in the grid, a lower number creates a coarser grid.
    static constexpr int DEFAULT_NUM_LINES = 70;

    // === Instance variables for the grid system ===

    // The actual grid size to be used for this world instance.
    // Initialized from DEFAULT_GRID_SIZE but could be made configurable later.
    const float size;

    // The number of lines used in this world instance's grid.
    const int numLines;

    // The offset used for this grid instance to shift it away from the origin.
    const float offset;

    // Pointer to the Easy3D LinesDrawable, which represents the grid in the viewer.
    // This object holds the OpenGL buffers for rendering the grid lines.
    easy3d::LinesDrawable* gridDrawable;

    // List of vertices that define the grid lines.
    // Each consecutive pair of vertices forms a single line segment.
    // Populated in createGridDrawable() and passed to gridDrawable for rendering.
    std::vector<easy3d::vec3> grid_vertices;
};

#endif // WORLD_H
