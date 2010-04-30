#include "bg_physics.h"

#include <vector>

#ifdef __cplusplus
extern "C" {
#endif

#include "cg_local.h"

enum JointType
{
    JOINT_BALL_AND_SOCKET = 0,
    JOINT_HINGE
};

struct Joint
{
    char*               name;
    JointType           jointType;
    int                 parentIndex;
    std::vector<int>    childIndices;
    
    float               twistAngleLimit;
    float               swingAngleLimit;
};

std::vector<Joint> joints;

qboolean CG_InitPhysics ( const char* mapname )
{
    if ( !BG_InitPhysics (mapname) )
    {
        return qfalse;
    }
    
    /*int boneIndex = 0;
    boneIndex = trap_G2API_AddBolt (*/
    
    return qtrue;
}

#ifdef __cplusplus
}
#endif