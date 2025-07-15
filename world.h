/********************************************************************
 * @file world.h
 * @brief Defines the World class responsible for ground, terrain, and grid visualization.
 *
 * This class encapsulates all rendering and geometric data needed to
 * visualize a ground plane, terrain surface, and 3D reference grid for
 * simulation environments in the UAV Simulator.
 ********************************************************************/
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

/**
 * @class World
 * @brief Manages terrain, ground, and 3D grid elements within the simulation scene.
 *
 * This class generates various visualization components such as:
 * - A flat ground surface
 * - A procedurally generated 3D terrain
 * - Optional textured terrain
 * - A 3D grid system for spatial orientation
 */
class World 
{
public:
     /**
     * @brief Constructs a World object with default parameters.
     *
     * Initializes size, offset, and resolution of the 3D grid.
     */
    World();
    /**
     * @brief Creates a flat ground surface and adds it to the viewer.
     *
     * @param viewer The Easy3D viewer to which the ground will be added.
     */
    void createGround(easy3d::Viewer& viewer); //Flat ground
    /**
     * @brief Creates a basic 3D terrain surface without textures.
     *
     * @param viewer The Easy3D viewer to which the terrain will be added.
     */
    void createTerrain(easy3d::Viewer& viewer);  // 3d terrain
    /**
     * @brief Creates a 3D terrain surface and applies a texture image.
     *
     * @param viewer The Easy3D viewer to which the textured terrain will be added.
     */
    void createTerrainWithTexture(easy3d::Viewer& viewer); // textured terrain
    /**
     * @brief Generates and displays a 3D coordinate grid.
     *
     * @param viewer The Easy3D viewer to which the grid will be added.
     */
    void createGridDrawable(easy3d::Viewer& viewer); // Creates the 3d grid system

private:
    // === Default parameters for the 3D grid visualization ===

    // The total length of each axis in the grid (in world units).
    // The grid spans from -size/2 to +size/2 along each axis plane.
    // Increase this to make the grid larger, decrease for a smaller grid.
    /**
     * @brief Total size of the grid (length of each axis), in world units.
     *
     * The grid spans from `-size/2` to `+size/2` in each direction.
     */
    static constexpr float DEFAULT_GRID_SIZE = 6000.0f;

    // The offset that shifts the grid's location along one or more axes.
    // In the provided grid creation logic, this is used to "lift" the grid away from the world origin
    // so that it doesn't obscure other objects like the aircraft.
    /**
     * @brief Vertical offset for the grid to visually separate it from objects like aircraft.
     */
    static constexpr float DEFAULT_OFFSET = 3000.0f;

    // The number of lines to draw along each axis.
    // Controls the resolution (density) of the grid.
    // A higher number results in finer divisions in the grid, a lower number creates a coarser grid.
     /**
     * @brief Number of lines along each axis of the grid.
     *
     * Determines the grid's resolution (higher = denser grid).
     */
    static constexpr int DEFAULT_NUM_LINES = 70;

    // === Instance variables for the grid system ===

    // The actual grid size to be used for this world instance.
    // Initialized from DEFAULT_GRID_SIZE but could be made configurable later.
    /**
     * @brief Actual grid size for this world instance.
     *
     * Initialized from DEFAULT_GRID_SIZE.
     */
    const float size;

    // The number of lines used in this world instance's grid.
    /**
     * @brief Number of grid lines for this world instance.
     *
     * Initialized from DEFAULT_NUM_LINES.
     */
    const int numLines;

    // The offset used for this grid instance to shift it away from the origin.
     /**
     * @brief Grid offset to displace it from the origin.
     *
     * Initialized from DEFAULT_OFFSET.
     */
    const float offset;

    // Pointer to the Easy3D LinesDrawable, which represents the grid in the viewer.
    // This object holds the OpenGL buffers for rendering the grid lines.
    /**
     * @brief Pointer to Easy3D's LinesDrawable used to render the grid.
     *
     * Managed by Easy3D and registered with the Viewer.
     */
    easy3d::LinesDrawable* gridDrawable;

    // List of vertices that define the grid lines.
    // Each consecutive pair of vertices forms a single line segment.
    // Populated in createGridDrawable() and passed to gridDrawable for rendering.
     /**
     * @brief Vertices defining the grid lines.
     *
     * Stored as a flat list of 3D points, where each pair defines a line.
     */
    std::vector<easy3d::vec3> grid_vertices;
};

#endif // WORLD_H
