/******************************************************************************
* <framework.h>
* Bailey Jia-Tao Brown
* 2021
* 
*	Header file for abstract graphics and utilites library
*	Contents:
*		- Header guard
*		- API definition
*		- Includes
*		- Definitions
*		- Structs
*		- Module initialization function
*		- Module termination function
*		- Module threading related functions
*		- Struct creation functions
*		- Struct destruction functions
*		- Struct related functions
*		- Rendering functions
*		- Physics functions
* 
******************************************************************************/

#ifndef __VFRAMEWORK_INCLUDE__
#define __VFRAMEWORK_INCLUDE__

/* API DEFINITIONS */
#ifdef VFRAMEWORK_EXPORTS
#define VFAPI __declspec(dllexport)
#else
#define VFAPI __declspec(dllimport)
#endif

/* INCLUDES */
#include <graphics.h> /* Graphics library */

/* DEFINITIONS */
#define VF_MAX_CHILDREN 0x10
#define VF_BUFFER_SIZE_INIT 0x60
#define VF_BUFFER_SIZE_INCREMENT 0x30
#define VF_PARENT_SEARCH_THRESHOLD 0x20
#define VF_COLLISIONS_MAX 0x10
#define VF_NOPARENT UINT_MAX
#define VF_NOENTITY UINT_MAX
#define VF_MUTEX_TIMEOUT_INTERVAL 0x100
#define VF_PUSHBACK_MAGNITUDE_MAX 0x20
#define VF_TOURQUE_MIN_VELOCITY 0.05f
#define VF_TOURQUE_MAX 8.0f
#define VF_VECTOR_SIMILARITY_THRESOLD 0.15f

#define VECT(x, y) vfCreateVector(x, y)
#define COLOR(r, g, b) vfCreateColor(r, g, b, 255)
#define PHYS(b, d, m) vfCreatePhysics(b, d, m)
#define PHYSA(b, d, m, mov, rot) vfCreatePhysicsa(b, d, m, mov, rot)
#define ENT(eHndl) vfGetEntity(eHndl)
#define PCLE(pHndl) vfGetParticle(pHndl)
#define ETRANSFORM(eHndl) vfGetTransformEnt(eHndl)
#define PTRANSFORM(pHndl) vfGetTransformEnt(vfGetParticle(pHndl)->transform)
#define EPHYSICS(eHndl) vfGetEntity(eHndl)->physics
#define TFORM(tHndl) vfGetTransform(tHndl)
#define EBOUND(eHndl) vfGetBound(vfGetEntity(eHndl)->bounds)

/* STRUCTURE DEFINITIONS */
typedef unsigned int vfHandle;
typedef void (*ENTCOLCALLBACK)(struct vfEntity* source, struct vfEntity* target);

typedef struct vfVector
{
	float x;
	float y;
} vfVector;

typedef struct vfColor
{
	int r;
	int g;
	int b;
	int a;
} vfColor;

typedef struct vfTransform
{
	vfVector position;
	float rotation;
	float scale;

	vfHandle parent;
} vfTransform;

typedef struct vfPhysics
{
	int active;
	int moveable;
	int rotationLock;
	float bounciness;
	float drag;
	float mass;

	vfVector velocity;
	float tourque;
} vfPhysics;

typedef struct vfBound
{
	int active;
	vfHandle body;
	vfVector position;
	vfVector dimensions;

	vfHandle entity;
} vfBound;

typedef struct vfParticle
{
	int active;
	unsigned char layer;
	vgShape shape;
	vgTexture texture;
	vfColor filter;

	vfHandle transform;
} vfParticle;

typedef struct vfEntity
{
	int active;
	unsigned char layer;
	vgTexture texture;
	vgShape shape;
	vfColor filter;

	vfHandle bounds;
	vfPhysics physics;
	vfHandle transform;
	ENTCOLCALLBACK collisionCallback;
} vfEntity;

/* MODULE INIT AND TERMINATE FUNCTIONS */
VFAPI void vfInit(void);
VFAPI void vfTerminate(void);

/* THREADING RELATED FUNCTIONS */
VFAPI void vfThreadSleepTime(unsigned int miliseconds);

/* STRUCT CREATION FUNCTIONS */
VFAPI vfVector vfCreateVector(float x, float y);
VFAPI vfColor vfCreateColor(int r, int g, int b, int a);
VFAPI vfHandle vfCreateTransformv(vfVector vector);
VFAPI vfHandle vfCreateTransforma(vfVector vector, float rotation,
	float scale);
VFAPI vfHandle vfCreateTransformp(vfHandle parent);
VFAPI vfHandle vfCreateBoundt(vfHandle body);
VFAPI vfHandle vfCreateBounda(vfHandle body, vfVector position,
	vfVector dimensions);
VFAPI vfHandle vfCreateParticlet(vfHandle transform);
VFAPI vfHandle vfCreateParticlea(vfHandle transform, vgTexture texture,
	vgShape shape, unsigned char layer);
VFAPI vfPhysics vfCreatePhysics(float bounciness, float drag, float mass);
VFAPI vfPhysics vfCreatePhysicsa(float bounciness, float drag, float mass,
	int moveable, int rotationLock);
VFAPI vfHandle vfCreateEntity(unsigned char layer, vgShape shape,
	vgTexture texture, vfPhysics physics, vfVector boundPosition, 
	vfVector boundDimensions);

/* STRUCT DESTRUCTION FUNCTIONS */
VFAPI void vfDestroyTransform(vfHandle transform);
VFAPI void vfDestroyBound(vfHandle bound);
VFAPI void vfDestroyParticle(vfHandle particle);
VFAPI void vfDestroyEntity(vfHandle entity);

/* STRUCT RELATED FUNCTIONS */
VFAPI vfHandle vfGetTransformHandle(vfTransform* transform);
VFAPI vfHandle vfGetBoundHandle(vfBound* bound);
VFAPI vfHandle vfGetParticleHandle(vfParticle* particle);
VFAPI vfHandle vfGetEntityHandle(vfEntity* entity);
VFAPI vfTransform* vfGetTransform(vfHandle hndl);
VFAPI vfTransform* vfGetTransformEnt(vfHandle hndl);
VFAPI vfBound* vfGetBound(vfHandle hndl);
VFAPI vfParticle* vfGetParticle(vfHandle hndl);
VFAPI vfEntity* vfGetEntity(vfHandle hndl);

/* RENDERING FUNCTIONS */
VFAPI void vfRenderParticles(void);
VFAPI void vfRenderEntities(void);
VFAPI void vfRenderBounds(void);

/* PHYSICS RELATED FUNCTIONS */
VFAPI void vfSetPhysicsState(int value);
VFAPI void vfSetCollisionCallback(vfHandle entity, ENTCOLCALLBACK callback);

#endif 
