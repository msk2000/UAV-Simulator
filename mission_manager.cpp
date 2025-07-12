// Mission Manager Implementation file ====================
#include "mission_manager.h"

MissionManager::MissionManager() {}

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
