#ifndef WORLD_H
#define WORLD_H

#include <Eigen/Dense>
#include <easy3d/viewer/viewer.h>
#include <easy3d/renderer/drawable_triangles.h>
#include <easy3d/renderer/drawable_lines.h>
#include <easy3d/renderer/vertex_array_object.h>
#include <easy3d/core/surface_mesh.h>
#include <easy3d/fileio/surface_mesh_io.h>

class World 
{
public:
    void createGround(easy3d::Viewer& viewer);
};

#endif // WORLD_H
