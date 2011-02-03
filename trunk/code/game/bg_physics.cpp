#include "bg_physics.h"
#include "dMath/dMatrix.h"

#include <map>
#include <vector>

#ifdef __cplusplus
extern "C" {
#endif

#include "bg_strap.h"
#include "../qcommon/qfiles.h"

static char* mapBuffer = NULL;

NewtonWorld* bg_physicsWorld;
int bg_physicsDefaultGroupId;

static float lastUpdateTime = 0.0f;

const float METRES_PER_UNIT = 32.0f;
const float UNITS_PER_METRE = 1.0f / METRES_PER_UNIT;

typedef std::map<int, NewtonBody*> LevelEntitiesMap;
LevelEntitiesMap mapEntities;

typedef std::vector<dmodel_t> BrushModelList;
BrushModelList brushModels;

typedef std::vector<dsurface_t> SurfaceList;
SurfaceList mapSurfaces;

typedef std::vector<drawVert_t> VertexList;
VertexList mapVertices;

typedef std::vector<int> IndexList;
IndexList mapIndices;

void BG_QuakeToSIUnits ( const vec3_t in, vec3_t out )
{
    VectorScale (in, UNITS_PER_METRE, out);
}

void BG_QuakeToSIUnits2 ( const vec3_t in, dVector& out )
{
    out.m_x = in[0] * UNITS_PER_METRE;
    out.m_y = in[1] * UNITS_PER_METRE;
    out.m_z = in[2] * UNITS_PER_METRE;
}

void BG_SIToQuakeUnits ( const vec3_t in, vec3_t out )
{
    VectorScale (in, METRES_PER_UNIT, out);
}

float BG_QuakeToSIUnitsFloat ( float value )
{
    return value * UNITS_PER_METRE;
}

static void BG_PhysicsEntityDie ( const NewtonBody* )
{
    // do nothing
}

static void LoadMapSurfaces ( byte* base, dsurface_t* surfaces, drawVert_t* vertices, dmodel_t* models, int* indices, dshader_t* shaders, int numSurfaces, int numModels )
{
    NewtonCollision* collision = NewtonCreateTreeCollision (bg_physicsWorld, -1);
    NewtonTreeCollisionBeginBuild (collision);
    
    int surfaceCount = 0, vertexCount = 0;
    
    int lastSurface;
    for ( int i = 0; i < numModels; ++i )
    {
        lastSurface = models[i].firstSurface + models[i].numSurfaces;
        for ( int j = models[i].firstSurface; j < lastSurface; ++j )
        {
            // Shift the surface type up by 0xF so we can restore it later.
            surfaces[j].surfaceType <<= 0xF;
        }
    }
    
    vec3_t v[3];
    for ( int i = 0; i < numSurfaces; ++i )
    {
        if ( shaders[surfaces[i].shaderNum].contentFlags & CONTENTS_WATER )
        {
            continue;
        }
    
        switch ( surfaces[i].surfaceType )
        {
            case MST_TRIANGLE_SOUP:
            case MST_PLANAR:
                assert ((surfaces[i].numIndexes % 3) == 0);
                for ( int j = 0; j < surfaces[i].numIndexes; j += 3 )
                {
                    VectorCopy (vertices[surfaces[i].firstVert + indices[surfaces[i].firstIndex + j + 2]].xyz, v[0]);
                    VectorCopy (vertices[surfaces[i].firstVert + indices[surfaces[i].firstIndex + j + 1]].xyz, v[1]);
                    VectorCopy (vertices[surfaces[i].firstVert + indices[surfaces[i].firstIndex + j + 0]].xyz, v[2]);
                    
                    BG_QuakeToSIUnits (v[0], v[0]);
                    BG_QuakeToSIUnits (v[1], v[1]);
                    BG_QuakeToSIUnits (v[2], v[2]);
                    
                    NewtonTreeCollisionAddFace (collision, 3, &v[0][0], sizeof (vec3_t), bg_physicsDefaultGroupId);
                }

                vertexCount += surfaces[i].numVerts;
                ++surfaceCount;
            break;
            
            case MST_PATCH:
            {
                /*int rowLimit = surfaces[i].patchHeight - 1;
                int colLimit = surfaces[i].patchWidth - 1;
                
                for ( int j = 0; j < rowLimit; ++j )
                {
                    for ( int k = 0; k < colLimit; ++k )
                    {
                        VectorCopy (vertices[surfaces[i].firstVert + (j * surfaces[i].patchWidth) + k].xyz, v[0]);
                        VectorCopy (vertices[surfaces[i].firstVert + (j * surfaces[i].patchWidth) + k + 1].xyz, v[1]);
                        VectorCopy (vertices[surfaces[i].firstVert + ((j + 1) * surfaces[i].patchWidth) + k].xyz, v[2]);
                        
                        BG_QuakeToSIUnits (v[0], v[0]);
                        BG_QuakeToSIUnits (v[1], v[1]);
                        BG_QuakeToSIUnits (v[2], v[2]);
                        
                        NewtonTreeCollisionAddFace (collision, 3, &v[0][0], sizeof (vec3_t), bg_physicsDefaultGroupId);

                        VectorCopy (vertices[surfaces[i].firstVert + (j * surfaces[i].patchWidth) + k + 1].xyz, v[0]);
                        VectorCopy (vertices[surfaces[i].firstVert + ((j + 1) * surfaces[i].patchWidth) + k].xyz, v[1]);
                        VectorCopy (vertices[surfaces[i].firstVert + ((j + 1) * surfaces[i].patchWidth) + k + 1].xyz, v[2]);
                        
                        BG_QuakeToSIUnits (v[0], v[0]);
                        BG_QuakeToSIUnits (v[1], v[1]);
                        BG_QuakeToSIUnits (v[2], v[2]);
                        
                        NewtonTreeCollisionAddFace (collision, 3, &v[0][0], sizeof (vec3_t), bg_physicsDefaultGroupId);
                    }
                }
                
                vertexCount += surfaces[i].numVerts;
                ++surfaceCount;*/
            }
            break;
            
            case MST_FLARE:
                // This is a billboard. Don't need to worry about collisions with these.
            break;
            
            default:
            break;
        }
    }
    
    for ( int i = 0; i < numModels; ++i )
    {
        lastSurface = models[i].firstSurface + models[i].numSurfaces;
        for ( int j = models[i].firstSurface; j < lastSurface; ++j )
        {
            // Just make it a 'bad' surface so it doesn't get added.
            surfaces[j].surfaceType >>= 0xF;
        }
    }
        
    NewtonTreeCollisionEndBuild (collision, 1);
    
    Com_Printf ("...%d static map surfaces loaded, %d vertices total\n", surfaceCount, vertexCount);
    
    dMatrix worldMatrix (GetIdentityMatrix());
    NewtonBody* worldBody = NewtonCreateBody (bg_physicsWorld, collision, &worldMatrix[0][0]);
    NewtonReleaseCollision (bg_physicsWorld, collision);
    
    vec3_t worldMin, worldMax;
    
    NewtonCollisionCalculateAABB (collision, &worldMatrix[0][0], &worldMin[0], &worldMax[0]);
    NewtonSetWorldSize (bg_physicsWorld, &worldMin[0], &worldMax[0]);
}

static qboolean LoadSurfaces ( byte* base, lump_t* surfaces, lump_t* vertices, lump_t* bmodels, lump_t* indices, lump_t* shaders )
{
    dsurface_t* surf = (dsurface_t*)(base + surfaces->fileofs);
    drawVert_t* vert = (drawVert_t*)(base + vertices->fileofs);
    dmodel_t* model = (dmodel_t*)(base + bmodels->fileofs);
    dshader_t* shader = (dshader_t*)(base + shaders->fileofs);
    int* index = (int*)(base + indices->fileofs);
    
    int numSurfaces = 0,
        numModels = 0,
        numVertices = 0,
        numIndices = 0;
    
    if ( surfaces->filelen % sizeof (*surf) )
    {
        Com_Printf ("Corrupt surface data\n");
        return qfalse;
    }
    
    if ( vertices->filelen % sizeof (*vert) )
    {
        Com_Printf ("Corrupt vertex data\n");
        return qfalse;
    }
    
    if ( bmodels->filelen % sizeof (*model) )
    {
        Com_Printf ("Corrupt bmodel data\n");
        return qfalse;
    }
    
    if ( shaders->filelen % sizeof (*shader) )
    {
        Com_Printf ("Corrupt shader data\n");
        return qfalse;
    }
    
    numSurfaces = surfaces->filelen / sizeof (*surf);
    numModels = bmodels->filelen / sizeof (*model);
    numVertices = vertices->filelen / sizeof (*vert);
    numIndices = indices->filelen / sizeof (*index);
    --numModels;
    
    if ( numModels > 0 )
    {
        // Skip the first model, which is the 'world' model.
        ++model;
    }
    
    bg_physicsWorld = NewtonCreate();
    bg_physicsDefaultGroupId = NewtonMaterialGetDefaultGroupID (bg_physicsWorld);
    
    LoadMapSurfaces (base, surf, vert, model, index, shader, numSurfaces, numModels);
    for ( int i = 0; i < numModels; ++i )
    {
        brushModels.push_back (model[i]);
    }
    
    for ( int i = 0; i < numSurfaces; ++i )
    {
        mapSurfaces.push_back (surf[i]);
    }
    
    for ( int i = 0; i < numVertices; ++i )
    {
        mapVertices.push_back (vert[i]);
    }
    
    for ( int i = 0; i < numIndices; ++i )
    {
        mapIndices.push_back (index[i]);
    }
    
    return qtrue;
}

static qboolean LoadMapData ( const char* mapname, char** mapBuffer )
{
    char filename[MAX_QPATH] = { 0 };
    int fileLen = 0;
    fileHandle_t f = 0;

    if ( !strstr (mapname, "maps/") && !strstr (mapname, ".bsp") )
    {
        Q_strncpyz (filename, "maps/", sizeof (filename));
        Q_strcat (filename, sizeof (filename), mapname);
        Q_strcat (filename, sizeof (filename), ".bsp");
    }
    else
    {
        Q_strncpyz (filename, mapname, sizeof (filename));
    }
    
    fileLen = strap_FS_FOpenFile (filename, &f, FS_READ);
    if ( fileLen == -1 || !f )
    {
        strap_FS_FCloseFile (f);
        
        Com_Printf ("%s was empty, or it could not be opened for reading.\n", filename);
        
        return qfalse;
    }
    
    strap_TrueMalloc (reinterpret_cast<void**>(mapBuffer), sizeof (char) * fileLen);
    
    strap_FS_Read (*mapBuffer, fileLen, f);
    strap_FS_FCloseFile (f);
    
    return qtrue;
}

static qboolean LoadCollisionMap ( const char* mapname )
{   
    if ( !LoadMapData (mapname, &mapBuffer) )
    {
        return qfalse;
    }
    
    dheader_t* header = (dheader_t*)mapBuffer;
    
    if ( header->ident != BSP_IDENT || header->version != BSP_VERSION )
    {
        // How this could possibly happen I have no idea :P
        Com_Printf ("Incorrect BSP Ident or Version (Expected %d and %d, found %d and %d)\n",
                    BSP_IDENT, BSP_VERSION, header->ident, header->version);
    
        return qfalse;
    }
    
    if ( !LoadSurfaces ((byte*)mapBuffer, &header->lumps[LUMP_SURFACES], &header->lumps[LUMP_DRAWVERTS], &header->lumps[LUMP_MODELS], &header->lumps[LUMP_DRAWINDEXES], &header->lumps[LUMP_SHADERS]) )
    {
        strap_TrueFree (reinterpret_cast<void**>(&mapBuffer));
        return qfalse;
    }
    
    return qtrue;
}

void BG_DestroyBodyForEntity ( int entityId )
{
    LevelEntitiesMap::iterator ent = mapEntities.find (entityId);
    if ( ent == mapEntities.end() )
    {
        return;
    }
    
    NewtonDestroyBody (bg_physicsWorld, ent->second);
    mapEntities.erase (ent);
}

void BG_OrientRigidBody ( int entityId, const vec3_t newAngles )
{
    LevelEntitiesMap::iterator ent = mapEntities.find (entityId);
    if ( ent == mapEntities.end() )
    {
        return;
    }
    
    vec3_t axis[3];
    
    AnglesToAxis (newAngles, axis);
    dMatrix newMatrix;
    
    NewtonBodyGetMatrix (ent->second, &newMatrix[0][0]);
    newMatrix.m_front.m_x = axis[0][0];
    newMatrix.m_front.m_y = axis[0][1];
    newMatrix.m_front.m_z = axis[0][2];
    
    newMatrix.m_up.m_x = axis[1][0];
    newMatrix.m_up.m_y = axis[1][1];
    newMatrix.m_up.m_z = axis[1][2];
    
    newMatrix.m_right.m_x = axis[2][0];
    newMatrix.m_right.m_y = axis[2][1];
    newMatrix.m_right.m_z = axis[2][2];
    
    NewtonBodySetMatrix (ent->second, &newMatrix[0][0]);
}

void BG_RigidBodySetVelocity ( int entityId, const vec3_t velocity )
{
    LevelEntitiesMap::iterator ent = mapEntities.find (entityId);
    if ( ent == mapEntities.end() )
    {
        return;
    }

    vec3_t scaledVelocity;
    
    BG_QuakeToSIUnits (velocity, scaledVelocity);
    NewtonBodySetVelocity (ent->second, &scaledVelocity[0]);
}

void BG_MoveRigidBodyTo ( int entityId, const vec3_t newPosition )
{
    LevelEntitiesMap::iterator ent = mapEntities.find (entityId);
    if ( ent == mapEntities.end() )
    {
        return;
    }

    dMatrix matrix;
    
    NewtonBodyGetMatrix (ent->second, &matrix[0][0]);
    BG_QuakeToSIUnits2 (newPosition, matrix.m_posit);
    
    NewtonBodySetMatrix (ent->second, &matrix[0][0]);
}

void BG_LoadBrushModelForEntity ( int entityId, int modelId )
{
    if ( mapEntities.find (entityId) != mapEntities.end() )
    {
        return;
    }
    
    dmodel_t& bmodel = brushModels.at (modelId - 1);
    if ( bmodel.numSurfaces == 0 )
    {
        return;
    }
    
    NewtonMesh* mesh = NewtonMeshCreate (bg_physicsWorld);
    NewtonMeshBeginFace (mesh);
    
    vec3_t v[3];    
    for ( int i = bmodel.firstSurface, endSurface = bmodel.firstSurface + bmodel.numSurfaces;
            i < endSurface;
            i++ )
    {
        switch ( mapSurfaces[i].surfaceType )
        {
            case MST_TRIANGLE_SOUP:
            case MST_PLANAR:
                assert ((mapSurfaces[i].numIndexes % 3) == 0);
                for ( int j = 0; j < mapSurfaces[i].numIndexes; j += 3 )
                {
                    VectorCopy (mapVertices[mapSurfaces[i].firstVert + mapIndices[mapSurfaces[i].firstIndex + j + 2]].xyz, v[0]);
                    VectorCopy (mapVertices[mapSurfaces[i].firstVert + mapIndices[mapSurfaces[i].firstIndex + j + 1]].xyz, v[1]);
                    VectorCopy (mapVertices[mapSurfaces[i].firstVert + mapIndices[mapSurfaces[i].firstIndex + j + 0]].xyz, v[2]);
                    
                    BG_QuakeToSIUnits (v[0], v[0]);
                    BG_QuakeToSIUnits (v[1], v[1]);
                    BG_QuakeToSIUnits (v[2], v[2]);
                    
                    NewtonMeshAddFace (mesh, 3, &v[0][0], sizeof (vec3_t), bg_physicsDefaultGroupId);
                }
            break;
            
            case MST_PATCH:
            {
                /*int rowLimit = mapSurfaces[i].patchHeight - 1;
                int colLimit = mapSurfaces[i].patchWidth - 1;
                
                for ( int j = 0; j < rowLimit; ++j )
                {
                    for ( int k = 0; k < colLimit; ++k )
                    {
                        VectorCopy (mapVertices[mapSurfaces[i].firstVert + (j * mapSurfaces[i].patchWidth) + k].xyz, v[0]);
                        VectorCopy (mapVertices[mapSurfaces[i].firstVert + (j * mapSurfaces[i].patchWidth) + k + 1].xyz, v[1]);
                        VectorCopy (mapVertices[mapSurfaces[i].firstVert + ((j + 1) * mapSurfaces[i].patchWidth) + k].xyz, v[2]);
                        
                        BG_QuakeToSIUnits (v[0], v[0]);
                        BG_QuakeToSIUnits (v[1], v[1]);
                        BG_QuakeToSIUnits (v[2], v[2]);
                        
                        NewtonMeshAddFace (mesh, 3, &v[0][0], sizeof (vec3_t), bg_physicsDefaultGroupId);

                        VectorCopy (mapVertices[mapSurfaces[i].firstVert + (j * mapSurfaces[i].patchWidth) + k + 1].xyz, v[0]);
                        VectorCopy (mapVertices[mapSurfaces[i].firstVert + ((j + 1) * mapSurfaces[i].patchWidth) + k].xyz, v[1]);
                        VectorCopy (mapVertices[mapSurfaces[i].firstVert + ((j + 1) * mapSurfaces[i].patchWidth) + k + 1].xyz, v[2]);
                        
                        BG_QuakeToSIUnits (v[0], v[0]);
                        BG_QuakeToSIUnits (v[1], v[1]);
                        BG_QuakeToSIUnits (v[2], v[2]);
                        
                        NewtonMeshAddFace (mesh, 3, &v[0][0], sizeof (vec3_t), bg_physicsDefaultGroupId);
                    }
                }*/
            }
            break;
            
            case MST_FLARE:
                // This is a billboard. Don't need to worry about collisions with these.
            break;
            
            default:
            break;
        }
    }
    NewtonMeshEndFace (mesh);
    
    NewtonCollision* collision = NewtonCreateConvexHullFromMesh (bg_physicsWorld, mesh, 0.02f, entityId);
    dMatrix matrix (GetIdentityMatrix());
    NewtonBody* body = NewtonCreateBody (bg_physicsWorld, collision, &matrix[0][0]);
    
    NewtonReleaseCollision (bg_physicsWorld, collision);
    NewtonBodySetMaterialGroupID (body, bg_physicsDefaultGroupId);
    
    NewtonBodySetDestructorCallback (body, BG_PhysicsEntityDie);
    NewtonBodySetContinuousCollisionMode (body, 1);

    mapEntities.insert (std::make_pair (entityId, body));

    NewtonMeshDestroy (mesh);
}

qboolean BG_InitPhysics ( const char* mapname )
{
    if ( !LoadCollisionMap (mapname) )
    {
        return qfalse;
    }
    
    return qtrue;
}

void BG_ShutdownPhysics()
{
    strap_TrueFree (reinterpret_cast<void**>(&mapBuffer));
    NewtonDestroy (bg_physicsWorld);
}

void BG_UpdatePhysics ( int time, float timestep )
{
    NewtonUpdate (bg_physicsWorld, timestep);
}

#ifdef __cplusplus
}
#endif