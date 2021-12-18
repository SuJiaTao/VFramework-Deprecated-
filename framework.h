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
*		- Struct creation functions
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

/* STRUCTURE DEFINITIONS */
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

	struct vfTransform* parent;
	struct vfTransform* child[VF_MAX_CHILDREN];
} vfTransform;

typedef struct vfBound
{
	int active;
	vfTransform* body;
	vfVector position;
	vfVector dimensions;
} vfBound;

typedef struct vfParticle
{
	int active;
	int visible;
	vgShape shape;
	vgTexture texture;
	vfColor bias;

	vfTransform transform;
	vfBound bound;
} vfParticle;

/* MODULE INIT AND TERMINATE FUNCTIONS */
VFAPI void vfInit(void);
VFAPI void vfTerminate(void);

/* STRUCT CREATION FUNCTIONS */
VFAPI vfVector vfCreateVector(float x, float y);
VFAPI vfTransform* vfCreateTransformv(vfVector vector);
VFAPI vfTransform* vfCreateTransforma(vfVector vector, float rotation,
	float scale);
VFAPI vfTransform* vfCreateTransformp(vfTransform* parent);
VFAPI vfBound* vfCreateBoundt(vfTransform* body);
VFAPI vfBound* vfCreateBounda(vfTransform* body, vfVector position,
	vfVector dimensions);

#endif 
