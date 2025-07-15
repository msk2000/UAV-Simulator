/**
 * @file mission_manager.h
 * @brief Defines the MissionManager class responsible for creating high-level UAV missions.
 *
 * This class provides functionality to generate flight paths or circuits that the UAV
 * can follow during a mission. Currently, it supports creating simple predefined shapes,
 * such as square circuits.
 */
#ifndef MISSION_MANAGER_H
#define MISSION_MANAGER_H

#include "path.h"

/**
 * @class MissionManager
 * @brief Manages generation of high-level UAV flight missions.
 *
 * Provides methods to generate common flight patterns and potentially
 * load missions from external sources.
 */
class MissionManager 
{
public:
    /**
     * @brief Default constructor.
     */
    MissionManager();

    /**
     * @brief Generate a simple square flight circuit path.
     *
     * Creates a square path centered at (centerX, centerY) with the specified
     * size and constant altitude. The path can be used for repetitive circuits or testing.
     *
     * @param centerX X-coordinate of the square's center in world units.
     * @param centerY Y-coordinate of the square's center in world units.
     * @param size Length of each side of the square (world units).
     * @param altitude Altitude at which the square circuit is flown.
     * @return Path object representing the generated square circuit.
     */
    Path generateSquareCircuit(float centerX, float centerY, float size, float altitude);

private:
    // TODO: Add methods to load missions from files or generate other flight patterns.
};

#endif // MISSION_MANAGER_H
