/******************************************************************************
* <framework.h>
* Bailey Jia-Tao Brown
* 2021
*
*	Source file for abstract graphics and utilites library
*	Contents:
*		- Preprocessor definitions
*		- Includes
*		- Internal definitions
*		- Internal functions
*		- Internal resources
*		- Init and terminate functions
*		- Struct creation functions
*		- Struct destruction functions
*		- Struct related functions
*		- Rendering functions
*
******************************************************************************/

#define _CRT_SECURE_NO_WARNINGS
#define _WIN32_LEAN_AND_MEAN 

#include <stdio.h> /* For file I/O */
#include <malloc.h> /* For memory management */
#include <string.h> /* For strlen function */
#include <math.h> /* For trig functions */

#include <Windows.h> /* For time related functions */

#include "framework.h" /* Header */

#define BL 0
#define TL 1
#define TR 2
#define BR 3

/*========== INTERNAL RESOURCES ==========*/

static HANDLE _fThread; /* thread handle */
static HANDLE _heap; /* heap handle */
static unsigned int _sleepTime;

/* internal buffers */
static vfTransform* _tBuffer; static int* _tBufferField; static int _tBSize;
static vfTransform* _tFinalBuffer;
static vfParticle* _pBuffer; static int* _pBufferField; static int _pBSize;
static vfBound* _bBuffer; static int* _bBufferField; static int _bBSize;
static vfEntity* _eBuffer; static int* _eBufferField; static int _eBSize;

typedef struct boundQuad
{
	vfVector vectors[4];
	vfVector average;
} boundQuad;

/* boundquad buffer, this buffer maps ever Bound object to a quad, which */
/* is the Bound object's dimensions and offset translated by it's transform */
static boundQuad* _bqBuffer;


/*========================================*/

/* INTERNAL BOUNDQUAD CREATION FUNCTION */
static inline boundQuad createBoundQuad(vfVector bL, vfVector tL, vfVector tR,
	vfVector bR)
{
	boundQuad bQ;
	bQ.vectors[BL] = bL;
	bQ.vectors[TL] = tL;
	bQ.vectors[TR] = tR;
	bQ.vectors[BR] = bR;
	bQ.average = VECT(0, 0);
	return bQ;
}

/* VERTEX ROTATION FUNCTION */
static inline vfVector vertRotateScale(vfVector vertex, float angle,
	float scale)
{
	/* convert to polar */
	const float sqrX = powf(vertex.x, 2);
	const float sqrY = powf(vertex.y, 2);
	float r = sqrtf(sqrX + sqrY);
	float theta = atan2f(vertex.y, vertex.x);

	/* offset by angle */
	const float degToRadians = 0.01745329f;
	r *= scale;
	theta += (angle * degToRadians);

	/* convert back to cartesian */
	const float posX = r * cosf(theta);
	const float posY = r * sinf(theta);

	return vfCreateVector(posX, posY);
}

/* VERTEX AVERAGING FUNCTIONS */
static inline vfVector vertexAverage(vfVector* vArr, int count)
{
	vfVector avg = vfCreateVector(0, 0);

	for (int i = 0; i < count; i++)
	{
		avg.x += vArr[i].x;
		avg.y += vArr[i].y;
	}

	avg.x /= count;
	avg.y /= count;

	return avg;
}

/* INTERNAL BUFFER SEARCHING FUNCTION */
static inline int findBufferSpot(void** buffer, int** field, int* size, 
	size_t structSize)
{
	/* check for NULL */
	if (*buffer == NULL || *field == NULL) return -1;

	/* find empty spot within field */
	int i;
	int bSize = *size;
	for (i = 0; i < bSize; i++)
	{
		if ((*field)[i] == 0) return i;
	}
	
	/* if loop exit, size too small */
	void* temp;
	bSize += VF_BUFFER_SIZE_INCREMENT;

	/* realloc buffer object */
	temp = HeapReAlloc(_heap, HEAP_ZERO_MEMORY,
		*buffer, bSize * structSize);
	if (temp == NULL) return -1;
	*buffer = temp;

	/* IF BUFFER OBJECT IS TRANSFORM BUFFER, REALLOC FINAL TBUFFER */
	if (buffer == &_tBuffer)
	{
		temp = HeapReAlloc(_heap, HEAP_ZERO_MEMORY, _tFinalBuffer,
			bSize * sizeof(vfTransform));
		if (temp == NULL) return -1;
		_tFinalBuffer = temp;
	}

	/* IF BUFFER IS BOUND BUFFER, REALLOC BOUNDQUAD BUFFER */
	if (buffer = &_bBuffer)
	{
		temp = HeapReAlloc(_heap, HEAP_ZERO_MEMORY, _bqBuffer,
			bSize * sizeof(boundQuad));
		if (temp == NULL) return -1;
		_bqBuffer = temp;
	}

	/* realloc buffer field */
	temp = HeapReAlloc(_heap, HEAP_ZERO_MEMORY,
		*field, bSize * sizeof(int));
	if (temp == NULL) return -1;
	*field = temp;

	/* update size and return */
	*size = bSize;
	return i;
}

/* MODULE MAIN FUNCTION */
static DWORD WINAPI vfMain(void* params)
{
	while(1)
	{
		/* sleep (optional) */
		if (_sleepTime)
		{
			Sleep(_sleepTime);
		}

		/* ===== update FINAL transform objects ===== */
		int iSize = _tBSize;
		int pCounter = 0;

		for (int i = 0; i < iSize; i++)
		{
			/* check if empty */
			if (!_tBufferField[i]) continue;

			/* else, update final transform values */
			/* recall that there's an identical transform buffer */
			/* except all it's values correspond to their global value */
			/* based on their parent's values */
			vfTransform tValue = _tBuffer[i];
			vfTransform* tParent = tValue.parent;

			while (tParent != NULL)
			{
				/* makes sure not to get stuck in an infinte loop */
				pCounter++;
				if (pCounter > VF_PARENT_SEARCH_THRESHOLD) break;

				/* recall that tValue is a LOCAL position */
				/* relative to the parent transform */
				/* the goal of this function is to convert it */
				/* into it's proper global coordinates */

				/* firstly, convert the transform's offset from */
				/* the parent position into a global offset, accounting */
				/* for rotation and scaling */
				const float sqrX = powf(tValue.position.x, 2);
				const float sqrY = powf(tValue.position.y, 2);
				float r = sqrtf(sqrX + sqrY);
				float theta = atan2f(tValue.position.y, tValue.position.x);

				/* update scaling and angle */
				const float degToRadians = 0.01745329f;
				r *= tParent->scale;
				theta += (tParent->rotation * degToRadians);

				/* convert back to cartesian */
				const float posX = r * cosf(theta);
				const float posY = r * sinf(theta);

				/* update tValue so that position member reflects GLOBAL */
				/* offset from PARENT POSITION rather than LOCAL */
				tValue.position = vfCreateVector(posX, posY);

				/* update all other members */

				/* update visual scale */
				tValue.position.x += tParent->position.x;
				tValue.position.y += tParent->position.y;
				tValue.scale *= tParent->scale;
				tValue.rotation += tParent->rotation;

				/* search for NEXT parent */
				tParent = tParent->parent;
			}

			/* set value */
			_tFinalBuffer[i] = tValue;
		}
		/* ================================================== */

		/* UPDATE ALL ENTITY TRANSFORMS BASED ON VELOCITY */
		int entityScan = _eBSize;
		for (int i = 0; i < entityScan; i++)
		{
			/* grab entity */
			if (!_eBufferField[i]) continue;

			vfEntity* ent = _eBuffer + i;
			if (!ent->physics.active) continue;

			/* update transform members */
			vfPhysics* pObj = &(ent->physics);
			ent->transform->position.x += pObj->velocity.x;
			ent->transform->position.y += pObj->velocity.y;
			ent->transform->rotation += pObj->tourque;

			/* update physics members */
			pObj->velocity.x *= (1.0f - pObj->drag);
			pObj->velocity.y *= (1.0f - pObj->drag);
			pObj->tourque *= (1.0f - pObj->drag);
		}

		/* ===================================================================== */
		/* UPDATE ALL BOUNDQUAD INDEXES */

		int boundScan = _bBSize;
		for (int i = 0; i < boundScan; i++)
		{
			if (!_bBufferField[i]) continue;

			/* grab bound to convert */
			vfBound toConvert = _bBuffer[i];

			/* grab final transform */
			int tfIndex = toConvert.body - _tBuffer;
			vfTransform tFinal = _tFinalBuffer[tfIndex];

			/* create initial boundQuad */
			vfVector pos = toConvert.position;
			vfVector dims = toConvert.dimensions;
			dims.x += pos.x; dims.y += pos.y;
			boundQuad bQuad = createBoundQuad(VECT(pos.x, pos.y), VECT(pos.x, dims.y),
				VECT(dims.x, dims.y), VECT(dims.x, pos.y));

			/* offset bQuad by transform and rotate by angle */
			for (int j = 0; j < 4; j++)
			{
				bQuad.vectors[j] = vertRotateScale(bQuad.vectors[j], tFinal.rotation,
					tFinal.scale);
				bQuad.vectors[j].x += tFinal.position.x;
				bQuad.vectors[j].y += tFinal.position.y;
			}

			/* calculate average */
			bQuad.average = vertexAverage(bQuad.vectors, 4);

			/* assign bQuad to respective buffer index */
			_bqBuffer[i] = bQuad;
		}

		/* =================================================================== */
	}
}

/* INIT AND TERMINATE FUNCTIONS */
VFAPI void vfInit(void)
{
	/* ger process heap */
	_heap = GetProcessHeap();

	/* init module internal data */
	_sleepTime = 0;

	/* INIT TRANSFORM BUFFER */
	_tBuffer = HeapAlloc(_heap, HEAP_ZERO_MEMORY,
		sizeof(vfTransform) * VF_BUFFER_SIZE_INIT);
	_tBufferField = HeapAlloc(_heap, HEAP_ZERO_MEMORY,
		sizeof(int) * VF_BUFFER_SIZE_INIT);
	_tBSize = VF_BUFFER_SIZE_INIT;

	/* INIT FINAL TRANSFORM BUFFER */
	_tFinalBuffer = HeapAlloc(_heap, HEAP_ZERO_MEMORY,
		sizeof(vfTransform) * VF_BUFFER_SIZE_INIT);

	/* INIT PARTICLE BUFFER */
	_pBuffer = HeapAlloc(_heap, HEAP_ZERO_MEMORY,
		sizeof(vfParticle) * VF_BUFFER_SIZE_INIT);
	_pBufferField = HeapAlloc(_heap, HEAP_ZERO_MEMORY,
		sizeof(int) * VF_BUFFER_SIZE_INIT);
	_pBSize = VF_BUFFER_SIZE_INIT;

	/* INIT BOUND BUFFER */
	_bBuffer = HeapAlloc(_heap, HEAP_ZERO_MEMORY,
		sizeof(vfBound) * VF_BUFFER_SIZE_INIT);
	_bBufferField = HeapAlloc(_heap, HEAP_ZERO_MEMORY,
		sizeof(int) * VF_BUFFER_SIZE_INIT);
	_bBSize = VF_BUFFER_SIZE_INIT;

	/* INIT BOUNDQUAD BUFFER */
	_bqBuffer = HeapAlloc(_heap, HEAP_ZERO_MEMORY,
		sizeof(boundQuad) * VF_BUFFER_SIZE_INIT);

	/* INIT ENTITY BUFFER */
	_eBuffer = HeapAlloc(_heap, HEAP_ZERO_MEMORY,
		sizeof(vfEntity) * VF_BUFFER_SIZE_INIT);
	_eBufferField = HeapAlloc(_heap, HEAP_ZERO_MEMORY,
		sizeof(int) * VF_BUFFER_SIZE_INIT);
	_eBSize = VF_BUFFER_SIZE_INIT;

	/* init module main thread */
	_fThread = CreateThread(0, 0, vfMain, 0, 0, &_fThread);
}

VFAPI void vfTerminate(void)
{
	TerminateThread(_fThread, 1);
}

/* THREADING RELATED FUNCTIONS */
VFAPI void vfThreadSleepTime(unsigned int miliseconds)
{
	_sleepTime = miliseconds;
}

/* STRUCT CREATION FUNCTIONS */

VFAPI vfVector vfCreateVector(float x, float y)
{
	vfVector rVec;
	rVec.x = x; rVec.y = y;
	return rVec;
}

VFAPI vfColor vfCreateColor(int r, int g, int b, int a)
{
	vfColor rCol;
	rCol.r = r;
	rCol.g = g;
	rCol.b = b;
	rCol.a = a;
	return rCol;
}

VFAPI vfTransform* vfCreateTransformv(vfVector vector)
{
	/* find transform buffer spot */
	int tIndex = findBufferSpot(&_tBuffer, &_tBufferField, &_tBSize,
		sizeof(vfTransform));
	_tBufferField[tIndex] = 1;

	/* set values */
	vfTransform* rTransform = _tBuffer + tIndex;
	rTransform->position = vector;

	return rTransform;
}

VFAPI vfTransform* vfCreateTransforma(vfVector vector, float rotation,
	float scale)
{
	/* find transform buffer spot */
	int tIndex = findBufferSpot(&_tBuffer, &_tBufferField, &_tBSize,
		sizeof(vfTransform));
	_tBufferField[tIndex] = 1;

	/* set values */
	vfTransform* rTransform = _tBuffer + tIndex;
	rTransform->position = vector;
	rTransform->rotation = rotation;
	rTransform->scale = scale;

	return rTransform;
}

VFAPI vfTransform* vfCreateTransformp(vfTransform* parent)
{
	/* find transform buffer spot */
	int tIndex = findBufferSpot(&_tBuffer, &_tBufferField, &_tBSize,
		sizeof(vfTransform));
	_tBufferField[tIndex] = 1;

	/* set values */
	vfTransform* rTransform = _tBuffer + tIndex;
	rTransform->parent = parent;

	return rTransform;
}

VFAPI vfBound* vfCreateBoundt(vfTransform* body)
{
	/* find bound buffer spot */
	int bIndex = findBufferSpot(&_bBuffer, &_bBufferField, &_bBSize,
		sizeof(vfBound));
	_bBufferField[bIndex] = 1;

	/* set value */
	vfBound* rBound = _bBuffer + bIndex;
	rBound->body = body;
	rBound->active = 1;

	return rBound;
}

VFAPI vfBound* vfCreateBounda(vfTransform* body, vfVector position,
	vfVector dimensions)
{
	/* find bound buffer spot */
	int bIndex = findBufferSpot(&_bBuffer, &_bBufferField, &_bBSize,
		sizeof(vfBound));
	_bBufferField[bIndex] = 1;

	/* set value */
	vfBound* rBound = _bBuffer + bIndex;
	rBound->body = body;
	rBound->position = position;
	rBound->dimensions = dimensions;
	rBound->active = 1;

	return rBound;
}

VFAPI vfParticle* vfCreateParticlet(vfTransform* transform)
{
	/* get free spot */
	int pIndex = findBufferSpot(&_pBuffer, &_pBufferField, &_pBSize,
		sizeof(vfParticle));
	_pBufferField[pIndex] = 1;

	/* set values */
	vfParticle* rParticle = _pBuffer + pIndex;
	rParticle->layer = 0;
	rParticle->active = TRUE;
	rParticle->filter = vfCreateColor(255, 255, 255, 255);
	rParticle->transform = transform;

	return rParticle;
}

VFAPI vfParticle* vfCreateParticlea(vfTransform* transform, vgTexture texture,
	vgShape shape, unsigned char layer)
{
	/* get free spot */
	int pIndex = findBufferSpot(&_pBuffer, &_pBufferField, &_pBSize,
		sizeof(vfParticle));
	_pBufferField[pIndex] = 1;

	/* set values */
	vfParticle* rParticle = _pBuffer + pIndex;
	rParticle->active = TRUE;
	rParticle->layer = layer;
	rParticle->filter = vfCreateColor(255, 255, 255, 255);
	rParticle->texture = texture;
	rParticle->shape = shape;
	rParticle->transform = transform;

	return rParticle;
}

VFAPI vfPhysics vfCreatePhysics(float bounciness, float drag, float mass)
{
	vfPhysics rPhys;
	rPhys.bounciness = bounciness;
	rPhys.drag = drag;
	rPhys.mass = mass;
	rPhys.active = TRUE;
	rPhys.moveable = TRUE;

	return rPhys;
}

VFAPI vfEntity* vfCreateEntity(unsigned char layer, vgShape shape,
	vgTexture texture, vfPhysics physics, vfVector boundPosition,
	vfVector boundDimensions)
{
	/* find free spot */
	int eIndex = findBufferSpot(&_eBuffer, &_eBufferField, &_eBSize,
		sizeof(vfEntity));
	_eBufferField[eIndex] = 1;
	
	/* get return entity and init values */
	vfEntity* rEnt = _eBuffer + eIndex;
	rEnt->active = 1;
	rEnt->transform = vfCreateTransforma(vfCreateVector(0, 0), 0, 1);
	rEnt->bounds = vfCreateBounda(rEnt->transform, boundPosition,
		boundDimensions);
	rEnt->layer = layer;
	rEnt->filter = vfCreateColor(255, 255, 255, 255);
	rEnt->physics = physics;
	rEnt->shape = shape;
	rEnt->texture = texture;
	
	return rEnt;
}

/* STRUCT DESTRUCTION FUNCTIONS */

VFAPI void vfDestroyTransform(vfTransform* transform)
{
	int tIndex = transform - _tBuffer;
	_tBufferField[tIndex] = 0;
}

VFAPI void vfDestroyBound(vfBound* bound)
{
	int bIndex = bound - _bBuffer;
	_bBufferField[bIndex] = 0;
}

VFAPI void vfDestroyParticle(vfParticle* particle)
{
	int pIndex = particle - _pBuffer;
	_pBufferField[pIndex] = 0;
}

/* STRUCT RELATED FUNCTIONS */

VFAPI vfHandle vfGetTransformHandle(vfTransform* transform)
{
	return transform - _tBuffer;
}

VFAPI vfHandle vfGetBoundHandle(vfBound* bound)
{
	return bound - _bBuffer;
}

VFAPI vfHandle vfGetParticleHandle(vfParticle* particle)
{
	return particle - _pBuffer;
}

VFAPI vfTransform* vfGetTransform(vfHandle hndl)
{
	return _tBuffer + hndl;
}

VFAPI vfBound* vfGetBound(vfHandle hndl)
{
	return _bBuffer + hndl;
}

VFAPI vfParticle* vfGetParticle(vfHandle hndl)
{
	return _pBuffer + hndl;
}

/* RENDERING FUNCTIONS */

VFAPI void vfRenderParticles(void)
{
	int rCount = _eBSize;

	for (int i = 0; i < rCount; i++)
	{
		if (!_pBufferField[i]) continue;
		
		vfParticle render = _pBuffer[i];
		if (!render.active) continue;

		/* get FINAL transform */
		vfTransform* rTransform = render.transform;
		int tIndex = rTransform - _tBuffer;
		vfTransform tFinal = _tFinalBuffer[tIndex];

		vgRenderLayer(render.layer);
		vgUseTexture(render.texture);
		vgTextureFilter(render.filter.r, render.filter.g, render.filter.b,
			render.filter.a);
		vgDrawShapeTextured(render.shape, tFinal.position.x, tFinal.position.y,
			tFinal.rotation, tFinal.scale);

		vgTextureFilterReset();
		vgRenderLayer(0);
	}
}

VFAPI void vfRenderEntities(void)
{
	int eCount = _eBSize;

	for (int i = 0; i < eCount; i++)
	{
		if (!_eBufferField[i]) continue;

		vfEntity renderEnt = _eBuffer[i];
		if (!renderEnt.active) continue;

		/* grab the FINAL transform of the entity */
		int tIndex = renderEnt.transform - _tBuffer;
		vfTransform tFinal = _tFinalBuffer[tIndex];

		/* render */
		vgRenderLayer(renderEnt.layer);
		vgUseTexture(renderEnt.texture);
		vgTextureFilter(renderEnt.filter.r, renderEnt.filter.g, renderEnt.filter.b,
			renderEnt.filter.a);
		vgDrawShapeTextured(renderEnt.shape, tFinal.position.x,
			tFinal.position.y, tFinal.rotation, tFinal.scale);

		vgTextureFilterReset();
		vgRenderLayer(0);

	}
}

VFAPI void vfRenderBounds(void)
{
	int bCount = _bBSize;

	for (int i = 0; i < bCount; i++)
	{
		if (!_bBufferField[i]) continue;

		vgColor3(0, 255, 64);
		vgRenderLayer(128);

		boundQuad bQ = _bqBuffer[i];

		vgPointf(bQ.average.x, bQ.average.y);
		for (int j = 0; j < 4; j++)
		{
			int iNext = (j + 1) % 4;
			vgLinef(bQ.vectors[j].x, bQ.vectors[j].y, bQ.vectors[iNext].x,
				bQ.vectors[iNext].y);
		}
		vgRenderLayer(0);
	}
}