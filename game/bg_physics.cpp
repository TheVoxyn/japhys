#include "bg_physics.h"
#include "dMath/dMatrix.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "bg_strap.h"
#include "../qcommon/qfiles.h"

NewtonWorld* bg_physicsWorld;
int bg_physicsDefaultGroupId;

const float METRES_PER_UNIT = 32.0f;
const float UNITS_PER_METRE = 1.0f / METRES_PER_UNIT;

static qboolean LoadSurfaces ( byte* base, lump_t* surfaces, lump_t* vertices, lump_t* bmodels, lump_t* indices )
{
    dsurface_t* surf = (dsurface_t*)(base + surfaces->fileofs);
    drawVert_t* vert = (drawVert_t*)(base + vertices->fileofs);
    dmodel_t* model = (dmodel_t*)(base + bmodels->fileofs);
    int* index = (int*)(base + indices->fileofs);
    
    NewtonCollision* collision = NULL;
    
    int numSurfaces, numModels;
    int surfaceCount = 0, vertexCount = 0;
    
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
    
    numSurfaces = surfaces->filelen / sizeof (*surf);
    numModels = bmodels->filelen / sizeof (*model);
    --numModels;
    
    if ( numModels > 0 )
    {
        ++model;
    }
    
    bg_physicsWorld = NewtonCreate (NULL, NULL);
    
    bg_physicsDefaultGroupId = NewtonMaterialGetDefaultGroupID (bg_physicsWorld);
    
    collision = NewtonCreateTreeCollision (bg_physicsWorld, -1);
    NewtonTreeCollisionBeginBuild (collision);
    
    int lastSurface;
    for ( int i = 0; i < numModels; ++i, ++model )
    {
        lastSurface = model->firstSurface + model->numSurfaces;
        for ( int j = model->firstSurface; j < lastSurface; ++j )
        {
            // Just make it a 'bad' surface so it doesn't get added.
            surf[j].surfaceType = MST_BAD;
        }
    }
    
    vec3_t v[3];
    for ( int i = 0; i < numSurfaces; ++i, ++surf )
    {    
        switch ( surf->surfaceType )
        {
            case MST_TRIANGLE_SOUP:
            case MST_PLANAR:
                assert ((surf->numIndexes % 3) == 0);
                for ( int j = 0; j < surf->numIndexes; j += 3 )
                {
                    VectorCopy (vert[surf->firstVert + index[surf->firstIndex + j + 2]].xyz, v[0]);
                    VectorCopy (vert[surf->firstVert + index[surf->firstIndex + j + 1]].xyz, v[1]);
                    VectorCopy (vert[surf->firstVert + index[surf->firstIndex + j + 0]].xyz, v[2]);
                    
                    VectorScale (v[0], UNITS_PER_METRE, v[0]);
                    VectorScale (v[1], UNITS_PER_METRE, v[1]);
                    VectorScale (v[2], UNITS_PER_METRE, v[2]);
                    
                    NewtonTreeCollisionAddFace (collision, 3, &v[0][0], sizeof (vec3_t), bg_physicsDefaultGroupId);
                }

                vertexCount += surf->numVerts;
                ++surfaceCount;
            break;
            
            case MST_PATCH:
            {
                int rowLimit = surf->patchHeight - 1;
                int colLimit = surf->patchWidth - 1;
                
                for ( int j = 0; j < rowLimit; ++j )
                {
                    for ( int k = 0; k < colLimit; ++k )
                    {
                        VectorCopy (vert[surf->firstVert + (j * surf->patchWidth) + k].xyz, v[0]);
                        VectorCopy (vert[surf->firstVert + (j * surf->patchWidth) + k + 1].xyz, v[1]);
                        VectorCopy (vert[surf->firstVert + ((j + 1) * surf->patchWidth) + k].xyz, v[2]);
                        
                        VectorScale (v[0], UNITS_PER_METRE, v[0]);
                        VectorScale (v[1], UNITS_PER_METRE, v[1]);
                        VectorScale (v[2], UNITS_PER_METRE, v[2]);
                        
                        NewtonTreeCollisionAddFace (collision, 3, &v[0][0], sizeof (vec3_t), bg_physicsDefaultGroupId);

                        VectorCopy (vert[surf->firstVert + (j * surf->patchWidth) + k + 1].xyz, v[0]);
                        VectorCopy (vert[surf->firstVert + ((j + 1) * surf->patchWidth) + k].xyz, v[1]);
                        VectorCopy (vert[surf->firstVert + ((j + 1) * surf->patchWidth) + k + 1].xyz, v[2]);
                        
                        VectorScale (v[0], UNITS_PER_METRE, v[0]);
                        VectorScale (v[1], UNITS_PER_METRE, v[1]);
                        VectorScale (v[2], UNITS_PER_METRE, v[2]);
                        
                        NewtonTreeCollisionAddFace (collision, 3, &v[0][0], sizeof (vec3_t), bg_physicsDefaultGroupId);
                    }
                }
                
                vertexCount += surf->numVerts;
                ++surfaceCount;
            }
            break;
            
            case MST_FLARE:
                // This is a billboard. Don't need to worry about collisions with these.
            break;
            
            default:
            break;
        }
    }
        
    NewtonTreeCollisionEndBuild (collision, 1);
    
    Com_Printf ("...%d surfaces loaded, %d vertices total\n", surfaceCount, vertexCount);
    
    NewtonBody* worldBody = NewtonCreateBody (bg_physicsWorld, collision);
    NewtonReleaseCollision (bg_physicsWorld, collision);
    
    dMatrix worldMatrix (GetIdentityMatrix());
    NewtonBodySetMatrix (worldBody, &worldMatrix[0][0]);
    
    vec3_t worldMin, worldMax;
    
    NewtonCollisionCalculateAABB (collision, &worldMatrix[0][0], &worldMin[0], &worldMax[0]);
    NewtonSetWorldSize (bg_physicsWorld, &worldMin[0], &worldMax[0]);
    
    return qtrue;
}

static qboolean LoadCollisionData ( const char* mapname )
{
    char filename[MAX_QPATH] = { 0 };
    fileHandle_t f;
    dheader_t* header;
    char* buffer = NULL;
    int fileLen;
    
    Q_strncpyz (filename, "maps/", sizeof (filename));
    Q_strcat (filename, sizeof (filename), mapname);
    Q_strcat (filename, sizeof (filename), ".bsp");
    
    fileLen = strap_FS_FOpenFile (filename, &f, FS_READ);
    if ( fileLen == 0 )
    {
        strap_FS_FCloseFile (f);
        return qfalse;
    }
    
    buffer = (char*)malloc (fileLen);
    
    strap_FS_Read (buffer, fileLen, f);
    strap_FS_FCloseFile (f);
    
    header = (dheader_t*)buffer;
    
    if ( header->ident != BSP_IDENT || header->version != BSP_VERSION )
    {
        // How this could possibly happen I have no idea :P
        Com_Printf ("Incorrect BSP Ident or Version (Expected %d and %d, found %d and %d)\n",
                    BSP_IDENT, BSP_VERSION, header->ident, header->version);
    
        return qfalse;
    }
    
    if ( !LoadSurfaces ((byte*)buffer, &header->lumps[LUMP_SURFACES], &header->lumps[LUMP_DRAWVERTS], &header->lumps[LUMP_MODELS], &header->lumps[LUMP_DRAWINDEXES]) )
    {
        free (buffer);
        return qfalse;
    }
    
    free (buffer);
    
    return qtrue;
}

qboolean BG_InitPhysics ( const char* mapname )
{
    if ( !LoadCollisionData (mapname) )
    {
        return qfalse;
    }
    
    int defaultID = NewtonMaterialGetDefaultGroupID (bg_physicsWorld);
    
    return qtrue;
}

void BG_ShutdownPhysics()
{
    NewtonDestroy (bg_physicsWorld);
}

void BG_UpdatePhysics ( float timestep )
{    
    NewtonUpdate (bg_physicsWorld, timestep);
/*#ifdef _DEBUG
    //NewtonWorldForEachBodyInAABBDo (world, &worldMin[0], &worldMax[0], DebugBodies);
#endif*/
}

#ifdef __cplusplus
}
#endif