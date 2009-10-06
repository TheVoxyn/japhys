#ifndef BG_PHYSICS_H
#define BG_PHYSICS_H

#include <Newton.h>
#include <dMatrix.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "q_shared.h"

extern NewtonWorld* bg_physicsWorld;
extern int bg_physicsDefaultGroupId;

extern const float METRES_PER_UNIT;
extern const float UNITS_PER_METRE;

enum CollisionShape
{
    CUBOID = 0,
    SPHERE,
    CAPSULE,
    COMPOSITE
};

qboolean BG_InitPhysics ( const char* mapname );
void BG_ShutdownPhysics();

#ifdef __cplusplus
}
#endif

#endif
