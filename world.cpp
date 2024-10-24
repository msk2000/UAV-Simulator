// Class implementation file for the class world
#include "world.h"

void World::createGround(easy3d::Viewer& viewer) 
{
    // Create a TrianglesDrawable for the ground plane
    auto groundDrawable = new easy3d::TrianglesDrawable("ground");
    float w_size = 400.0f;
    float w_height = -200.0f;

    // Define vertices for the ground (a large flat plane at a fixed height)
    std::vector<easy3d::vec3> groundVertices = 
    {
        easy3d::vec3(-w_size, -w_size, w_height),  // Bottom-left corner
        easy3d::vec3(w_size, -w_size, w_height),   // Bottom-right corner
        easy3d::vec3(w_size, w_size, w_height),    // Top-right corner
        easy3d::vec3(-w_size, w_size, w_height)    // Top-left corner
    };

    // Define the indices to create two triangles forming the rectangular ground plane
    std::vector<unsigned int> indices = 
    {
        0, 1, 2,  // First triangle
        2, 3, 0   // Second triangle
    };

    // Upload the vertices and indices to the GPU
    groundDrawable->update_vertex_buffer(groundVertices);
    groundDrawable->update_element_buffer(indices);

    // Set a uniform color (e.g., green for grass)
    //groundDrawable->set_uniform_coloring(easy3d::vec4(0.0f, 1.0f, 0.0f, 1.0f));  // Green
    //groundDrawable->set_uniform_coloring(easy3d::vec4(0.0f, 0.5f, 0.0f, 1.0f));  // Dark Green
    groundDrawable->set_uniform_coloring(easy3d::vec4(0.0f, 0.3f, 0.0f, 1.0f));  // Darker Green

    // Add the ground drawable to the viewer
    viewer.add_drawable(groundDrawable);
}
