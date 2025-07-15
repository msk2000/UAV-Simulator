/**
 * @file common.h
 * @brief Common definitions and aliases for Easy3D types.
 */
#ifndef COMMON_H
#define COMMON_H

#include <easy3d/core/vec.h>

/**
 * @namespace easy3d
 * @brief Namespace for Easy3D rendering and geometry types.
 *
 * This alias simplifies access to 3D float vectors, which are commonly used
 * throughout the simulation for positions, directions, and waypoints.
 */
namespace easy3d
{
    using vec3 = Vec<3, float>;  // This creates easy3d::vec3
}

#endif // COMMON_H
