// ==================== mission_manager.h ====================
#ifndef MISSION_MANAGER_H
#define MISSION_MANAGER_H

#include "path.h"

// Responsible for generating high-level missions (paths/circuits)
class MissionManager 
{
public:
    MissionManager();

    // Create a simple square circuit with specified parameters
    Path generateSquareCircuit(float centerX, float centerY, float size, float altitude);

private:
    // Later: add methods to load missions from files or generate other shapes
};

#endif // MISSION_MANAGER_H
