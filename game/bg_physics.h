#ifndef BG_PHYSICS_H
#define BG_PHYSICS_H

#ifndef _NEWTON_USE_LIB
#define _NEWTON_USE_LIB
#endif

#include "Newton.h"
#include "dMath/dMatrix.h"
#include "dMath/dVector.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "q_shared.h"

extern NewtonWorld* bg_physicsWorld;
extern int bg_physicsDefaultGroupId;

enum CollisionShape
{
    CUBOID = 0,
    SPHERE,
    CAPSULE,
    COMPOSITE
};

qboolean BG_InitPhysics ( const char* mapname );
void BG_ShutdownPhysics();

void BG_QuakeToSIUnits ( const vec3_t in, vec3_t out );
void BG_QuakeToSIUnits2 ( const vec3_t in, dVector& out );
float BG_QuakeToSIUnitsFloat ( float value );

void BG_SIToQuakeUnits ( const vec3_t in, vec3_t out );

void BG_LoadBrushModelForEntity ( int entityId, int modelId );
void BG_DestroyBodyForEntity ( int entityId );

#ifdef __cplusplus
}
#endif

#endif
