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

// Function to load terrain from stl
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
    const std::string terrain_file = "/home/fahim/Coding/Git/UAV-Simulator/terrain.stl";
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


    // Get drawable and assign texture
    auto drawable = terrainMesh->renderer()->get_triangles_drawable("faces");
    drawable->set_texture(texture);
    drawable->set_texture_coloring(easy3d::State::VERTEX, "v:texcoord", texture);

    // Optional: smooth shading
    drawable->set_smooth_shading(true);
}




