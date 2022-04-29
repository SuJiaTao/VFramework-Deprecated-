/******************************************************************************
* <framework.h>
* Bailey Jia-Tao Brown
* 2021/2022
* 
*	Header file for abstract graphics and utilites library
*	Contents:
*		- Header guard
*		- Preprocessor directives
*		- API definition
*		- Includes
*		- Definitions
*		- Typedefs
*		- Structs
*		- Module initialization function
*		- Module termination function
*		- Module threading related functions
*		- Struct creation functions
*		- Struct destruction functions
*		- Struct related functions
*		- Rendering functions
*		- Physics functions
*		- Particle functions
*		- Attribute functions
*		- Data related functions
* 
******************************************************************************/

#ifndef __VFRAMEWORK_INCLUDE__
#define __VFRAMEWORK_INCLUDE__

/* PREPROCESSOR DIRECTIVES */
#pragma pack(1)

/* API DEFINITIONS */
#ifdef VFRAMEWORK_EXPORTS
#define VFAPI __declspec(dllexport)
#else
#define VFAPI __declspec(dllimport)
#endif

/* INCLUDES */
#include <graphics.h> /* Graphics library */
#include <stdint.h>   /* Int Sizes */

/* DEFINITIONS */
#define VF_MAX_CHILDREN 0x10
#define VF_BUFFER_SIZE 0x800
#define VF_PARENT_SEARCH_THRESHOLD 0x20
#define VF_COLLISIONS_MAX 0x10
#define VF_NOPARENT NULL
#define VF_NOENTITY NULL
#define VF_PARTICLE_CULLEXTRA    0.05f
#define VF_PARTICLE_DESTROYEXTRA 1.5f
#define VF_ENTITY_CULLEXTRA      0.25f

#define VF_WMUTEX_TIMEOUT 0xFF
#define VF_RMUTEX_TIMEOUT 0xFF

#define VF_PUSHBACK_RATIO_NOENT 1.15f
#define VF_PUSHBACK_MAGNITUDE_MAX 0x40
#define VF_TOURQUE_MIN_VELOCITY 1.5f
#define VF_TOURQUE_MAX 5.0f
#define VF_VECTOR_SIMILARITY_THRESOLD 0.15f
#define VF_POSITION_SIMILARITY 0.03f

#define VF_PART_SIZE_DEFAULT      0x5000
#define VF_PART_COUNT_INCREMENT   0x100
#define VF_PART_INCREASE_THRESOLD 0x20
#define VF_PART_DECREASE_THRESOLD 0x40
#define VF_PART_COUNT_MAXIMUM 0x1000
#define VF_PART_STEP          0x40
#define VF_BOUND_PART_MAX     0x20
#define VF_PART_OVERAGE_TIME  0x80
#define VF_PART_OVERLAPSCALE  1.05f
#define VF_PART_SKIP_DAMPENER 2.5f
#define VF_PART_SKIP_MINAGE   0x80
#define VF_PART_RENDERLAYER   0x10

#define VF_PB_MAX 0x40
#define VF_PB_ERROR -1
#define VF_PB_NO_BEHAVIOR VF_PB_ERROR

#define VF_OBJ_TRANSFORM 0x10
#define VF_OBJ_BOUND 0x20
#define VF_OBJ_PARTICLE 0x30
#define VF_OBJ_ENTITY 0x40

#define VF_BUFF_TRANSFORM 0x1
#define VF_BUFF_BOUND     0x2
#define VF_BUFF_PARTICLE  0x3
#define VF_BUFF_ENTITY    0x4
#define VF_BUFF_PARTITION 0x5

#define VF_STATICCALLBACK_MAX 0x20

#define VF_ATTRIBS_MAX 0xFF
#define VF_ATTRIBTABLES_MAX 0x80

#define VECT(x, y)             vfCreateVector(x, y)
#define COLOR(r, g, b)         vfCreateColor(r, g, b, 255)
#define COLORA(r, g, b, a)     vfCreateColor(r, g, b, a)
#define COLOR_OPAQUE           vfCreateColor(255, 255, 255, 255)
#define COLOR_TRANSPARENT      vfCreateColor(255, 255, 255, 0)
#define PHYS(b, d, m)          vfCreatePhysics(b, d, m)
#define PHYSA(b, d, m, mv, rl) vfCreatePhysicsA(b, d, m, mv, rl)
#define TFORM(pos)             vfCreateTransformV(pos)
#define TFORMV(x, y)           vfCreateTransformV(VECT(x, y))
#define TFORMA(p, r, s)        vfCreateTransformA(p, r, s)

/* TYPEDEFS */
typedef uint8_t  vfFlag;
typedef uint8_t  vfLayer;
typedef uint32_t vfHandle;
typedef uint64_t vfTickCount;
typedef uint16_t vfLifeTime;

typedef void (*ENTCOLCALLBACK) (struct vfEntity* source, struct vfEntity* target);
typedef void (*ENTUPDCALLBACK) (struct vfEntity* source);
typedef void (*STATUPDCALLBACK)(vfTickCount tickCount);
typedef void (*PARTUPDCALLBACK)(struct vfParticleBehavior* thisBehavior,
	vfLifeTime particleAge);
typedef void* (*ATTRIBGETFUNC)(struct vfEntity* source, const char* attributeName);
typedef void (*PROJTILECOLCALLBACK)(struct vfProjectile projectile,
	struct vfEntity* hitObject);

/* STRUCTURE DEFINITIONS */
typedef struct vfVector
{
	float x;
	float y;
} vfVector;

typedef struct vfColor
{
	int16_t r;
	int16_t g;
	int16_t b;
	int16_t a;
} vfColor;

typedef struct vfTransform
{
	vfVector position;
	float rotation;
	float scale;

	struct vfTransform* parent;
} vfTransform;

typedef struct vfPhysics
{
	vfFlag active       : 1;
	vfFlag moveable     : 1;
	vfFlag rotationLock : 1;
	float bounciness;
	float drag;
	float mass;

	vfVector velocity;
	float tourque;

	vfTickCount age;
} vfPhysics;

typedef struct vfBound
{
	vfFlag active;
	vfTransform* body;
	vfVector position;
	vfVector dimensions;

	struct vfEntity* entity;
} vfBound;

typedef struct vfParticleBehavior
{
	vfVector velocity;     /* initial velocity */
	vfColor  filterChange; /* initial filterChange */
	float    torque;       /* initial torque */
	float    sizeChange;   /* initial sizeChange */

	struct vfParticle* parent; /* parent particle */
	PARTUPDCALLBACK updateBehavior; /* called every update */
} vfParticleBehavior;

typedef struct vfParticle
{
	vfLayer layer; /* particle layer */

	vfLifeTime lifeTime;  /* time allowed to live (in pticks) */
	vfLifeTime lifeAge;   /* time particle has existed (int pticks) */

	vgShape shape;     /* shape   */
	vgTexture texture; /* texture */
	vfColor filter;    /* filter  */

	vfTransform transform; /* transform data */

	vfParticleBehavior behavior; /* particle behavior */
} vfParticle;

typedef struct vfProjectileBehavior
{
	vgShape shape;     /* projectile shape   */
	vgTexture texture; /* projectile texture */

	uint8_t penetrationPower; /* # of objects projectile can pass through */
	float penetrationChance;  /* percent of penetration */
	float scatterMagnitude;   /* direction change per penetration */

	float accuracy; /* random offset on projectile creation */
	
	float speed; /* projectile speed */
	float speedVariation; /* random speed offset */

	float scale; /* shape scale */
	float scaleVariation; /* random scale offset */

	/* collision callback */
	PROJTILECOLCALLBACK collisionCallback;

	vfLifeTime maxAge; /* max time allowed to live */

} vfProjectileBehavior;

typedef struct vfProjectile
{
	/* object that projectile originated from */
	vfEntity* source;

	/* amount of objects projectile has passed through */
	uint8_t penetrations;
	
	float angle;       /* projectile movement angle */
	vfVector position; /* projectile position */
	vfLifeTime age;    /* projectile age */

	vfHandle behaviorHandle; /* projectile behavior */
} vfProjectile;

typedef struct vfAttributeTable
{
	uint32_t attribNameHash  [VF_ATTRIBS_MAX]; /* attribute name hash     */
	uint16_t attribByteOffset[VF_ATTRIBS_MAX]; /* memory block offset ptr */
	uint16_t attribByteCount; /* bytes used   */
	uint8_t  attribCount;     /* attribs used */
} vfAttributeTable;

typedef struct vfEntity
{
	vfFlag active;
	vfLayer layer;
	vgTexture texture;
	vgShape shape;
	vfColor filter;

	vfBound* bounds;
	vfPhysics physics;
	vfTransform* transform;

	ENTCOLCALLBACK collisionCallback;
	ENTUPDCALLBACK updateCallback;

	vfHandle attribHandle;
	uint8_t* attribBlock;
	ATTRIBGETFUNC getAttribute;
} vfEntity;

/* MODULE INIT AND TERMINATE FUNCTIONS */
VFAPI void vfInit(void);
VFAPI void vfTerminate(void);

/* THREADING RELATED FUNCTIONS */
VFAPI void vfThreadSleepTime(unsigned int miliseconds);

/* STRUCT CREATION FUNCTIONS */
VFAPI vfVector vfCreateVector(float x, float y);
VFAPI vfColor vfCreateColor(int r, int g, int b, int a);
VFAPI vfTransform* vfCreateTransformV(vfVector position);
VFAPI vfTransform* vfCreateTransformA(vfVector position, float rotation,
	float scale);
VFAPI vfTransform* vfCreateTransformP(vfTransform* parent);
VFAPI vfBound* vfCreateBoundT(vfTransform* body);
VFAPI vfBound* vfCreateBoundA(vfTransform* body, vfVector position,
	vfVector dimensions);
VFAPI vfPhysics vfCreatePhysics(float bounciness, float drag, float mass);
VFAPI vfPhysics vfCreatePhysicsA(float bounciness, float drag, float mass,
	int moveable, int rotationLock);
VFAPI vfEntity* vfCreateEntity(vfLayer layer, vgShape shape,
	vgTexture texture, vfPhysics physics, vfVector boundPosition, 
	vfVector boundDimensions);

/* STRUCT DESTRUCTION FUNCTIONS */
VFAPI void vfDestroyTransform(vfTransform* transform, int zero);
VFAPI void vfDestroyBound(vfBound* bound, int zero);
VFAPI void vfDestroyParticle(vfParticle* particle);
VFAPI void vfDestroyEntity(vfEntity* entity, int zero);
VFAPI void vfDestroyAllBounds(int zero);
VFAPI void vfDestroyAllNonEntityBounds(int zero);
VFAPI void vfDestroyAllParticles(int zero);
VFAPI void vfDestroyAllEntities(int zero);

/* STRUCT RELATED FUNCTIONS */
VFAPI vfHandle vfGetTransformHandle(vfTransform* transform);
VFAPI vfHandle vfGetBoundHandle(vfBound* bound);
VFAPI vfHandle vfGetParticleHandle(vfParticle* particle);
VFAPI vfHandle vfGetEntityHandle(vfEntity* entity);
VFAPI void* vfGetObject(vfHandle handle, int type);

/* RENDERING FUNCTIONS */
VFAPI void vfRenderParticles(void);
VFAPI void vfRenderEntities(void);
VFAPI void vfRenderBounds(void);
VFAPI void vfRenderPartitions(void);

/* PHYSICS RELATED FUNCTIONS */
VFAPI void vfSetPhysicsState(int value);
VFAPI void vfSetCollisionCallback(vfEntity* entity, ENTCOLCALLBACK callback);
VFAPI void vfSetUpdateCallback(vfEntity* entity, ENTUPDCALLBACK callback);
VFAPI int  vfSetUpdateCallbackStatic(STATUPDCALLBACK callback,
	int priorityRequest);
VFAPI void vfSetPartitionSize(int size);
VFAPI void vfSetPartitionMaxCount(int value);
VFAPI void vfGetPhysicsTickCount(vfTickCount* ticks);
VFAPI int  vfGetPhysicsUpdateTime(void);
VFAPI void vfGetPhysicsCollisionCounts(int* objCheckCount, 
	int* partCheckCount);
VFAPI void vfGetEntityPartitions(vfEntity* ent, int maxPartitions,
	int* xBuff, int* yBuff, int* xSize, int* ySize);
VFAPI void vfLogPhysicsPartitionData(FILE* file);
VFAPI void vfLogPhysicsCollisionData(FILE* file);

/* PARTICLE RELATED FUNCTIONS */
VFAPI void vfCreateParticle(vgShape shape, vgTexture texture,
	vfColor filter, vfLifeTime lifeTime, vfLayer layer, 
	vfVector position, vfHandle behavior);
VFAPI void vfCreateParticleT(vgShape shape, vgTexture texture,
	vfColor filter, vfLifeTime lifeTime, vfLayer layer, 
	vfTransform transform, vfHandle behavior);
VFAPI vfHandle vfCreateParticleBehavior(PARTUPDCALLBACK behavior);
VFAPI vfHandle vfCreateParticleBehaviorP(vfParticleBehavior reference);

/* ATTRIBUTE RELATED FUNCTIONS */
VFAPI vfHandle vfAttribTableRegister(vfAttributeTable toRegister);
VFAPI void vfAttribTableAdd(vfAttributeTable* target,
	const char* attributeName, size_t memSize);
VFAPI void vfSetEntityAttribHandle(vfEntity* entity, vfHandle handle);

/* DATA RELATED FUNCTIONS */
VFAPI void* vfGetBuffer(int type);
VFAPI void* vfGetBufferField(int type);
VFAPI int   vfGetObjectCount(int type);

#endif 
