// mission_manager.cpp
// Implementation of the MissionManager class
#include "mission_manager.h"

MissionManager::MissionManager() {}

/**
 * @brief Generates a square flight path centered at (centerX, centerY) with a given size and altitude.
 *
 * The path consists of four straight segments forming a closed square loop.
 *
 * @param centerX X coordinate of the square center
 * @param centerY Y coordinate of the square center
 * @param size Length of each side of the square
 * @param altitude Constant altitude for the square flight path
 * @return Path object representing the square circuit
 */
Path MissionManager::generateSquareCircuit(float centerX, float centerY, float size, float altitude) 
{
    Path path;

    // Four corners of a square
    path.addStraightSegment({centerX - size / 2, centerY - size / 2, altitude},
                            {centerX + size / 2, centerY - size / 2, altitude});
    path.addStraightSegment({centerX + size / 2, centerY - size / 2, altitude},
                            {centerX + size / 2, centerY + size / 2, altitude});
    path.addStraightSegment({centerX + size / 2, centerY + size / 2, altitude},
                            {centerX - size / 2, centerY + size / 2, altitude});
    path.addStraightSegment({centerX - size / 2, centerY + size / 2, altitude},
                            {centerX - size / 2, centerY - size / 2, altitude});

    return path;
}
