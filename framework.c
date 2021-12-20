/******************************************************************************
* <framework.h>
* Bailey Jia-Tao Brown
* 2021
*
*	Source file for abstract graphics and utilites library
*	Contents:
*		- Preprocessor definitions
*		- Includes
*		- Internal functions
*		- Internal resources
*		- Init and terminate functions
*		- Struct creation functions
*		- Struct destruction functions
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

/*========== INTERNAL RESOURCES ==========*/

static HANDLE _fThread; /* thread handle */
static HANDLE _heap; /* heap handle */
/* internal buffers */
static vfTransform* _tBuffer; static int* _tBufferField; static int _tBSize;
static vfTransform* _tFinalBuffer;
static vfParticle* _pBuffer; static int* _pBufferField; static int _pBSize;
static vfBound* _bBuffer; static int* _bBufferField; static int _bBSize;

/*========================================*/

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
		/* update FINAL transform objects */
		int iSize = _tBSize;

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
	}
}

/* INIT AND TERMINATE FUNCTIONS */
VFAPI void vfInit(void)
{
	/* ger process heap */
	_heap = GetProcessHeap();

	/* init module internal data */

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

	/* init module main thread */
	_fThread = CreateThread(NULL, NULL, vfMain, NULL, NULL, &_fThread);
}

VFAPI void vfTerminate(void)
{
	TerminateThread(_fThread, 1);
}

/* STRUCT CREATION FUNCTIONS */

VFAPI vfVector vfCreateVector(float x, float y)
{
	vfVector rVec;
	rVec.x = x; rVec.y = y;
	return rVec;
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
	rParticle->transform = transform;

	return rParticle;
}

VFAPI vfParticle* vfCreateParticlea(vfTransform* transform, vgTexture texture,
	vgShape* shape)
{
	/* get free spot */
	int pIndex = findBufferSpot(&_pBuffer, &_pBufferField, &_pBSize,
		sizeof(vfParticle));
	_pBufferField[pIndex] = 1;

	/* set values */
	vfParticle* rParticle = _pBuffer + pIndex;
	rParticle->texture = texture;
	rParticle->shape = shape;
	rParticle->transform = transform;

	return rParticle;
}

/* STRUCT DESTRUCTION FUNCTIONS */

VFAPI void vfDestroyTransform(vfTransform* transform)
{
	/* search for matching pointer */
	for (int i = 0; i < _tBSize; i++)
	{
		if (_tBuffer + i == transform)
		{
			_tBufferField[i] = 0;
			return;
		}
	}
}

VFAPI void vfDestroyBound(vfBound* bound)
{
	/* search for matching pointer */
	for (int i = 0; i < _bBSize; i++)
	{
		if (_bBuffer + i == bound)
		{
			_bBufferField[i] = 0;
			return;
		}
	}
}

VFAPI void vfDestroyParticle(vfParticle* particle)
{
	/* search for matching pointer */
	for (int i = 0; i < _pBSize; i++)
	{
		if (_pBuffer + i == particle)
		{
			_pBufferField[i] = 0;
			return;
		}
	}
}

/* RENDERING FUNCTIONS */

VFAPI void vfRenderParticles(void)
{
	int rCount = _pBSize;

	for (int i = 0; i < rCount; i++)
	{
		if (_pBufferField[i] == 0) continue;

		vfParticle render = _pBuffer[i];

		/* get FINAL transform */
		vfTransform* rTransform = render.transform;
		int tIndex = rTransform - _tBuffer;
		vfTransform tFinal = _tFinalBuffer[tIndex];

		vgUseTexture(render.texture);
		vgDrawShapeTextured(render.shape, tFinal.position.x, tFinal.position.y,
			tFinal.rotation, tFinal.scale);
	}
}