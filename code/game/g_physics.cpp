#include "bg_physics.h"

#include <map>

#ifdef __cplusplus
extern "C" {
#endif

#include "g_local.h"

static int metalMaterial;
static const vec3_t reduction = { 2.0f, 2.0f, 2.0f };

static const float baseRigidBodyMass = 100.0f;
static const float baseRigidBodyVolume = 64.0f / 27.0f;

struct StaticBody
{
    StaticBody(): ent (NULL), body (NULL) {}
    
    gentity_t* ent;
    NewtonBody* body;
};

typedef std::map<int, StaticBody> StaticBodyList;
static StaticBodyList staticEnts;

/*#ifdef _DEBUG
void G_TestLine (vec3_t start, vec3_t end, int color, int time);
void DebugPolies ( void* userData, int vertexCount, const float* faceArray, int faceID )
{
    int limit = vertexCount - 1;
    vec3_t start, end;
    vec3_t begin = { faceArray[0], faceArray[1], faceArray[2] };
    for ( int i = 0; i < limit; ++i )
    {
        VectorSet (start, faceArray[i * 3 + 0], faceArray[i * 3 + 1], faceArray[i * 3 + 2]);
        VectorSet (end, faceArray[i * 3 + 3], faceArray[i * 3 + 4], faceArray[i * 3 + 5]);
        G_TestLine (start, end, 0x00ff0000, 1000 / g_svfps.integer);
    }
    
    G_TestLine (begin, end, 0x00ff0000, 1000 / g_svfps.integer);
}

void DebugBodies ( const NewtonBody* body )
{
    float mass, Ixx, Iyy, Izz;
    
    NewtonBodyGetMassMatrix (body, &mass, &Ixx, &Iyy, &Izz);
    if ( mass > 0.0f )
    {
        dMatrix matrix;
        NewtonBodyGetMatrix (body, &matrix[0][0]);
        NewtonCollisionForEachPolygonDo (NewtonBodyGetCollision (body), &matrix[0][0], DebugPolies, NULL);
    }
}
#endif*/

qboolean G_InitPhysics ( const char* mapname )
{
    if ( !BG_InitPhysics (mapname) )
    {
        return qfalse;
    }
    
    metalMaterial = NewtonMaterialCreateGroupID (bg_physicsWorld);
    NewtonMaterialSetDefaultElasticity (bg_physicsWorld, bg_physicsDefaultGroupId, metalMaterial, 0.005f);
    NewtonMaterialSetDefaultFriction (bg_physicsWorld, bg_physicsDefaultGroupId, metalMaterial, 0.7f, 0.7f);
    
    return qtrue;
}

static int GetBuoyancyPlane ( const int collisionID, void *context, const dFloat* globalSpaceMatrix, dFloat* globalSpacePlane )
{
    trace_t* tr = (trace_t*)context;
    
    float dist = BG_QuakeToSIUnitsFloat (tr->plane.dist);

    globalSpacePlane[0] = tr->plane.normal[0];
    globalSpacePlane[1] = tr->plane.normal[1];
    globalSpacePlane[2] = tr->plane.normal[2];
    
    globalSpacePlane[3] = dist;

    return 1;
}

extern vmCvar_t phys_fluidDensity;
extern vmCvar_t phys_fluidLinearViscosity;
extern vmCvar_t phys_fluidAngularViscosity;
static void G_PhysicsEntityThink ( const NewtonBody* body, dFloat timestep, int threadIndex )
{
    gentity_t* ent = (gentity_t*)NewtonBodyGetUserData (body);
    
    trace_t tr;
    
    trap_Trace (&tr, ent->r.currentOrigin, ent->r.mins, ent->r.maxs, ent->r.currentOrigin, ent->s.number, CONTENTS_WATER);
    if ( tr.fraction != 1.0f )
    {
        vec3_t gravity = { 0.0f, 0.0f, -9.81f };
    
        NewtonBodyAddBuoyancyForce (body, phys_fluidDensity.value * (ent->mass / baseRigidBodyMass) * (baseRigidBodyVolume / ent->bodyVolume), phys_fluidLinearViscosity.value, phys_fluidAngularViscosity.value, &gravity[0], GetBuoyancyPlane, (void*)&tr);
    }
    
    vec3_t impulse, origin;
    BG_QuakeToSIUnits (ent->forceApplied, impulse);
    BG_QuakeToSIUnits (ent->s.origin, origin);
    VectorScale (impulse, 5.0f / ent->mass, impulse);
    
    NewtonBodyAddImpulse (body, &impulse[0], &origin[0]);
    VectorClear (ent->forceApplied); 
    
    vec3_t force;
    VectorClear (force);
    
    force[2] += ent->mass * -9.81f;
    
    NewtonBodyAddForce (body, &force[0]);
}

static void G_PhysicsEntitySetTransform ( const NewtonBody* body, const dFloat* matrix, int threadIndex )
{
    gentity_t *ent = (gentity_t*)NewtonBodyGetUserData (body);
    vec3_t newPosition;
    vec3_t angles;
    vec3_t axis[3];
    
    newPosition[0] = matrix[12];
    newPosition[1] = matrix[13];
    newPosition[2] = matrix[14];
    
    BG_SIToQuakeUnits (newPosition, newPosition);
    
    axis[0][0] = matrix[0];
    axis[0][1] = matrix[1];
    axis[0][2] = matrix[2];
    axis[1][0] = matrix[4];
    axis[1][1] = matrix[5];
    axis[1][2] = matrix[6];
    axis[2][0] = matrix[8];
    axis[2][1] = matrix[9];
    axis[2][2] = matrix[10];
    
    AxisToAngles (axis, angles);
    
    trap_UnlinkEntity (ent);
    
    VectorCopy (newPosition, ent->s.pos.trBase);
    VectorCopy (newPosition, ent->r.currentOrigin);
    VectorCopy (newPosition, ent->s.origin);
    
    VectorCopy (angles, ent->s.apos.trBase);
    VectorCopy (angles, ent->r.currentAngles);
    VectorCopy (angles, ent->s.angles);
    
    trap_LinkEntity (ent);
    
    //Com_Printf ("PHYSICS: updating %d old position: %s new position %s\n", ent->s.number, vtos (oldPosition), vtos (newPosition));
}

/*static void G_PhysicsEntityAddComposite ( gentity_t* ent, NewtonCollision** collision )
{
    //collision = NewtonCreateSphere (world, size[0] * 0.5f, size[2] * 0.5f, size[1] * 0.5f, NULL);
}*/

static void G_PhysicsEntityAddSphere ( gentity_t* ent, NewtonCollision** collision, qboolean reduce = qtrue )
{
    vec3_t size;
    
    VectorSubtract (ent->r.maxs, ent->r.mins, size);
    if ( reduce )
    {
        VectorSubtract (size, reduction, size);
    }
    
    BG_QuakeToSIUnits (size, size);
    
    *collision = NewtonCreateSphere (bg_physicsWorld, size[0] * 0.5f, size[2] * 0.5f, size[1] * 0.5f, ent->s.number, NULL);
}

static void G_PhysicsEntityAddCapsule ( gentity_t* ent, NewtonCollision** collision, qboolean reduce = qtrue )
{
    vec3_t size;
    
    VectorSubtract (ent->r.maxs, ent->r.mins, size);
    if ( reduce )
    {
        VectorSubtract (size, reduction, size);
    }
    
    BG_QuakeToSIUnits (size, size);
    
    /*dMatrix offsetMatrix (
        dVector (0.0f, 1.0f, 0.0, 0.0f),
        dVector (0.0f, 0.0f, 1.0f, 0.0f),
        dVector (1.0f, 0.0f, 0.0f, 0.0f),
        dVector (0.0f, 0.0f, 0.0f, 1.0f)
    );*/
    
    *collision = NewtonCreateCapsule (bg_physicsWorld, size[0] / 4.0f, size[2], ent->s.number, NULL/*&offsetMatrix[0][0]*/);
}

static void G_PhysicsEntityAddCuboid ( gentity_t* ent, NewtonCollision** collision, qboolean reduce = qtrue )
{
    vec3_t size;
    
    VectorSubtract (ent->r.maxs, ent->r.mins, size);
    if ( reduce )
    {
        VectorSubtract (size, reduction, size);
    }
    
    BG_QuakeToSIUnits (size, size);
    
    *collision = NewtonCreateBox (bg_physicsWorld, size[0], size[1], size[2], ent->s.number, NULL);
}

static void G_PhysicsEntityDie ( const NewtonBody* )
{
    // do nothing
}

static void G_PhysicsEntityAdd ( gentity_t* ent )
{
    NewtonCollision* collision = NULL;
    NewtonBody* body = NULL;

    switch ( ent->genericValue1 )
    {
        case CUBOID:
            G_PhysicsEntityAddCuboid (ent, &collision);
        break;
        
        case SPHERE:
            G_PhysicsEntityAddSphere (ent, &collision);
        break;
        
        /*case COMPOSITE:
        
        break;*/
        
        default:
            // Fail
            G_FreeEntity (ent);
            return;
        break;
    }
    
    body = NewtonCreateBody (bg_physicsWorld, collision);
    ent->bodyVolume = NewtonConvexCollisionCalculateVolume (collision);
    NewtonReleaseCollision (bg_physicsWorld, collision);
    
    NewtonBodySetMaterialGroupID (body, metalMaterial);
    NewtonBodySetUserData (body, (void*)ent);
    NewtonBodySetDestructorCallback (body, G_PhysicsEntityDie);
    NewtonBodySetContinuousCollisionMode (body, 1);
    NewtonBodySetForceAndTorqueCallback (body, G_PhysicsEntityThink);
    NewtonBodySetTransformCallback (body, G_PhysicsEntitySetTransform);
    
    vec3_t inertia, com;
    NewtonConvexCollisionCalculateInertialMatrix (collision, &inertia[0], &com[0]);
    NewtonBodySetCentreOfMass (body, &com[0]);
    
    VectorScale (inertia, ent->mass, inertia);
    
    NewtonBodySetMassMatrix (body, ent->mass, inertia[0], inertia[1], inertia[2]);
    
    dMatrix matrix (GetIdentityMatrix());
    BG_QuakeToSIUnits2 (ent->s.origin, matrix.m_posit);
    
    NewtonBodySetMatrix (body, &matrix[0][0]);
}

void G_AddStaticEntity ( gentity_t* ent, entityType_t entityType )
{
    if ( !ent )
    {
        return;
    }
    
    NewtonCollision* collision = NULL;
    
    switch ( entityType )
    {
        case ET_PLAYER:
            G_PhysicsEntityAddCapsule (ent, &collision, qfalse);
        break;
        
        case ET_MOVER:
            G_PhysicsEntityAddCuboid (ent, &collision);
        break;
        
        default:
            return;
        break;
    }
    
    StaticBody sbody;
    
    sbody.ent = ent;
    sbody.body = NewtonCreateBody (bg_physicsWorld, collision);
    NewtonReleaseCollision (bg_physicsWorld, collision);
    
    NewtonBodySetDestructorCallback (sbody.body, G_PhysicsEntityDie);
    
    dMatrix matrix (GetIdentityMatrix());
    BG_QuakeToSIUnits2 (ent->r.currentOrigin, matrix.m_posit);
    
    NewtonBodySetMatrix (sbody.body, &matrix[0][0]);
    
    staticEnts[ent->s.number] = sbody;
}

void G_RemoveStaticEntity ( gentity_t* ent )
{
    if ( !ent )
    {
        return;
    }
    
    StaticBodyList::iterator it = staticEnts.find (ent->s.number);
    if ( it == staticEnts.end() )
    {
        return;
    }
    
    NewtonDestroyBody (bg_physicsWorld, it->second.body);
    staticEnts.erase (it);
}

void G_UpdateStaticBodies()
{
    dMatrix matrix (GetIdentityMatrix());

    for ( StaticBodyList::iterator it = staticEnts.begin();
            it != staticEnts.end();
            ++it )
    {
        StaticBody& sbody = it->second;
    
        BG_QuakeToSIUnits2 (sbody.ent->r.currentOrigin, matrix.m_posit);
        
        NewtonBodySetMatrix (sbody.body, &matrix[0][0]);
        NewtonBodySetFreezeState (sbody.body, 0);
    }
}

void InitMover( gentity_t *ent );
void SP_func_physics ( gentity_t* ent )
{
    trap_SetBrushModel (ent, ent->model);
    
    VectorCopy (ent->s.origin, ent->pos1);
    VectorCopy (ent->s.origin, ent->pos2);
    
    InitMover (ent);
    
    ent->s.pos.trType = TR_INTERPOLATE;
    VectorCopy (ent->s.origin, ent->s.pos.trBase);
    VectorCopy (ent->s.origin, ent->r.currentOrigin);
    
    ent->s.apos.trType = TR_INTERPOLATE;
    VectorCopy (ent->s.angles, ent->s.apos.trBase);
    VectorCopy (ent->s.angles, ent->r.currentAngles);
	
	G_SpawnFloat ("mass", "500.0", &ent->mass);
	G_SpawnInt ("shape", "0", &ent->genericValue1);
    
    G_PhysicsEntityAdd (ent);
}

void Cmd_SpawnPhysicsEntity ( gentity_t *ent )
{
    gentity_t *obj;
    vec3_t origin, forward;

    AngleVectors (ent->client->ps.viewangles, forward, NULL, NULL);

    VectorMA (ent->client->ps.origin, 100.0f, forward, origin);
    
    obj = G_Spawn();

    obj->classname = "func_physics";
    obj->model = "*1";
    VectorCopy (origin, obj->s.origin);
    
    obj->mass = (float)Q_irand (50, 100);
    VectorCopy (ent->s.angles, obj->s.angles);

    SP_func_physics (obj);
}

#ifdef __cplusplus
}
#endif