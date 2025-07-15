// Class implementation file for the class world
#include "world.h"

World::World(): size(DEFAULT_GRID_SIZE),
      numLines(DEFAULT_NUM_LINES),
      offset(DEFAULT_OFFSET),
      gridDrawable(nullptr)
      {}
[[deprecated("Use createTerrainWithTexture() instead")]]
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

// Function to load terrain from stl
[[deprecated("Use createTerrainWithTexture() instead")]]
void World::createTerrain(easy3d::Viewer& viewer) 
{
    // Specify the file path to your STL file
    const std::string filePath = "/home/fahim/Coding/Git/UAV-Simulator/terrain.stl";

    // Load the STL mesh using Easy3D's MeshIO
    easy3d::SurfaceMesh* surfaceMesh = new easy3d::SurfaceMesh();
    // Load the STL file into the SurfaceMesh
    if (!easy3d::io::load_stl(filePath, surfaceMesh)) 
    {
        std::cerr << "Failed to load STL file: " << filePath << std::endl;
        delete surfaceMesh;  // Clean up if loading fails
        return;
    }


    // Set uniform color for all vertices
    auto color_prop = surfaceMesh->get_vertex_property<easy3d::vec3>("v:color");
    if (!color_prop) 
    {
        color_prop = surfaceMesh->add_vertex_property<easy3d::vec3>("v:color");
    }
    //easy3d::vec3 uniformColor(0.0f, 0.3f, 0.0f);  // green
    easy3d::vec3 uniformColor(0.2f, 0.1f, 0.0f);  // green
    for (auto v : surfaceMesh->vertices()) 
    {
        color_prop[v] = uniformColor;
    }


    // Add the SurfaceMesh as a drawable object to the viewer
    auto* groundDrawable = viewer.add_model(surfaceMesh, "Terrain");
    if (groundDrawable) 
    {
        std::cout << "Terrain loaded successfully from: " << filePath << std::endl;
        
    
        
     //groundDrawable->set_uniform_coloring(easy3d::vec4(0.5f, 0.4f, 0.3f, 1.0f)); // Example color
    } 
    else 
    {
        std::cerr << "Failed to add the terrain to the viewer." << std::endl;
    }
}

void World::createTerrainWithTexture(easy3d::Viewer& viewer)
{
    const std::string terrain_file = "/home/fahim/Coding/Git/UAV-Simulator/terrain3.stl";
    const std::string texture_file = "/home/fahim/Coding/Git/UAV-Simulator/texture.jpg";

    // Load mesh as shared_ptr
    std::shared_ptr<easy3d::SurfaceMesh> terrainMesh = std::make_shared<easy3d::SurfaceMesh>();
    if (!easy3d::io::load_stl(terrain_file, terrainMesh.get()))
    {
        std::cerr << "Failed to load STL file: " << terrain_file << std::endl;
        return;
    }

    // Compute min/max x,z for UV normalization
    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();

    for (auto v : terrainMesh->vertices())
    {
        const auto& pos = terrainMesh->position(v);
        min_x = std::min(min_x, pos[0]);
        max_x = std::max(max_x, pos[0]);
        min_z = std::min(min_z, pos[1]);
        max_z = std::max(max_z, pos[1]);
    }

    float terrain_width = max_x - min_x;
    float terrain_depth = max_z - min_z;

    // Add UV coordinates normalized and tiled with aspect ratio correction
    auto uv_prop = terrainMesh->add_vertex_property<easy3d::vec2>("v:texcoord");
    const float tile_factor = 2.0f;  // number of texture repeats on smaller side

    // Determine which side is smaller for uniform square scaling
    float aspect_ratio = terrain_width / terrain_depth;

    for (auto v : terrainMesh->vertices())
    {
        const auto& pos = terrainMesh->position(v);
        float u = (pos[0] - min_x) / terrain_width;
        float w = (pos[1] - min_z) / terrain_depth;

        if (terrain_width < terrain_depth)
        {
            // width smaller, scale u by tile_factor, w scaled to keep squares
            uv_prop[v] = easy3d::vec2(u * tile_factor,
                                      w * tile_factor * aspect_ratio);
        }
        else
        {
            // depth smaller or equal, scale w by tile_factor, u scaled accordingly
            uv_prop[v] = easy3d::vec2(u * tile_factor / aspect_ratio,
                                      w * tile_factor);
        }
    }

    // Load texture via TextureManager with REPEAT wrap mode
    easy3d::Texture* texture = easy3d::Texture::create(texture_file, easy3d::Texture::WrapMode::REPEAT, easy3d::Texture::FilterMode::LINEAR);
    if (!texture)
    {
        std::cerr << "Failed to load texture image: " << texture_file << std::endl;
        return;
    }


    // Add model to viewer
    viewer.add_model(terrainMesh, true);
    //viewer.fit_screen(terrainMesh.get());


    // Get drawable and assign texture
    auto drawable = terrainMesh->renderer()->get_triangles_drawable("faces");
    drawable->set_texture(texture);
    drawable->set_texture_coloring(easy3d::State::VERTEX, "v:texcoord", texture);

    // Optional: smooth shading
    drawable->set_smooth_shading(true);
}


// Function to create the grid system
void World::createGridDrawable(easy3d::Viewer& viewer)
{
    // Create a LinesDrawable to visualize the 3D grid.
    gridDrawable = new easy3d::LinesDrawable("grid");

     // Z-axis offset to lift the entire grid above z = 0
    float z_offset = 0.5f * size;

    // Create the grid lines.
    for (int i = 0; i < numLines; i++)
    {
        float t = -0.5f * size + (size / (numLines - 1)) * i;

        // === X-Y Plane (at z = 0 ) ===
        /*grid_vertices.push_back(easy3d::vec3(t, -0.5f * size, 0));
        grid_vertices.push_back(easy3d::vec3(t,  0.5f * size, 0));

        grid_vertices.push_back(easy3d::vec3(-0.5f * size, t, 0));
        grid_vertices.push_back(easy3d::vec3( 0.5f * size, t, 0));*/

        // === Y-Z Plane (shift z by offset) ===
        grid_vertices.push_back(easy3d::vec3(0.0f - offset, -0.5f * size, t + z_offset));
        grid_vertices.push_back(easy3d::vec3(0.0f - offset,  0.5f * size, t + z_offset));

        grid_vertices.push_back(easy3d::vec3(0.0f - offset, t, -0.5f * size + z_offset));
        grid_vertices.push_back(easy3d::vec3(0.0f - offset, t,  0.5f * size + z_offset));

        // === X-Z Plane (shift z by offset) ===
        grid_vertices.push_back(easy3d::vec3(-0.5f * size, 0.0f - offset, t + z_offset));
        grid_vertices.push_back(easy3d::vec3( 0.5f * size, 0.0f - offset, t + z_offset));

        grid_vertices.push_back(easy3d::vec3(t, 0.0f - offset, -0.5f * size + z_offset));
        grid_vertices.push_back(easy3d::vec3(t, 0.0f - offset,  0.5f * size + z_offset));
    }


    // Upload the grid vertices to the GPU.
    gridDrawable->update_vertex_buffer(grid_vertices);

    // Set the color of the grid lines (here we use gray).
    gridDrawable->set_uniform_coloring(easy3d::vec4(0.5f, 0.5f, 0.5f, 0.3f));

    // Set the width of the grid lines (here we use 1 pixel).
    gridDrawable->set_line_width(1.0f);

    // Add the grid drawable to the viewer.
    viewer.add_drawable(gridDrawable);

    // Color settings for viewer background
        //viewer.set_background_color(easy3d::vec4(0.1f, 0.1f, 0.1f, 1.0f)); // RGBA: dark gray, fully opaque
        //viewer.set_background_color(easy3d::vec4(0.1f, 0.1f, 0.44f, 1.0f)); // Midnight Blue
        //viewer.set_background_color(easy3d::vec4(0.6f, 0.8f, 0.6f, 1.0f)); // Soft Pastel Green
        viewer.set_background_color(easy3d::vec4(0.0f, 0.0f, 0.0f, 1.0f)); // Deep Space Black
        //viewer.set_background_color(easy3d::vec4(1.0f, 0.5f, 0.0f, 1.0f)); // Sunset Orange
        //viewer.set_background_color(easy3d::vec4(0.0f, 0.5f, 0.5f, 1.0f)); // Ocean Teal
        //viewer.set_background_color(easy3d::vec4(0.5f, 0.0f, 0.13f, 1.0f)); // Rich Burgundy
        //viewer.set_background_color(easy3d::vec4(0.53f, 0.81f, 0.98f, 1.0f)); // Bright Sky Blue
        //viewer.set_background_color(easy3d::vec4(0.678f, 0.847f, 0.902f, 1.0f)); // SKY attempts
    //set_uniform_coloring(easy3d::vec4(0.678f, 0.847f, 0.902f, 1.0f));


    std::cout << "Grid drawable added to viewer" <<"\n";

    // Update the viewer
    viewer.update();


}



