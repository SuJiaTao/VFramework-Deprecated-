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
#define VF_COLLISIONS_MAX 0x20

#define VF_NOPARENT -1

#define VF_MUTEX_DEADLOCK_INTERVAL 0xfff
#define VF_MUTEX_RELEASE_SLEEP_TIME 0x1

#define VECT(x, y) vfCreateVector(x, y)
#define COLOR(r, g, b) vfCreateColor(r, g, b, 255)
#define PHYS(b, d, m) vfCreatePhysics(b, d, m)
#define ETRANSFORM(eHndl) vfGetTransform(vfGetEntity(eHndl)->transform)
#define TFORM(tHndl) vfGetTransform(tHndl)

/* STRUCTURE DEFINITIONS */

typedef unsigned int vfHandle;

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

	vfPhysics* physics;
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
VFAPI vfHandle vfCreateEntity(unsigned char layer, vgShape shape,
	vgTexture texture, vfPhysics physics, vfVector boundPosition, 
	vfVector boundDimensions);

/* STRUCT DESTRUCTION FUNCTIONS */
VFAPI void vfDestroyTransform(vfHandle transform);
VFAPI void vfDestroyBound(vfHandle bound);
VFAPI void vfDestroyParticle(vfHandle particle);

/* STRUCT RELATED FUNCTIONS */
VFAPI vfHandle vfGetTransformHandle(vfTransform* transform);
VFAPI vfHandle vfGetBoundHandle(vfBound* bound);
VFAPI vfHandle vfGetParticleHandle(vfParticle* particle);
VFAPI vfTransform* vfGetTransform(vfHandle hndl);
VFAPI vfBound* vfGetBound(vfHandle hndl);
VFAPI vfParticle* vfGetParticle(vfHandle hndl);
VFAPI vfEntity* vfGetEntity(vfHandle hndl);

/* RENDERING FUNCTIONS */
VFAPI void vfRenderParticles(void);
VFAPI void vfRenderEntities(void);
VFAPI void vfRenderBounds(void);

#endif 
