/******************************************************************************
* <framework.h>
* Bailey Jia-Tao Brown
* 2021
*
*	Source file for abstract graphics and utilites library
*	Contents:
*		- Preprocessor definitions
*		- Includes
*		- Internal buffer structure
*		- Internal buffer struct related functions
*		- Internal definitions
*		- Internal functions
*		- Internal resources
*		- Init and terminate functions
*		- Struct creation functions
*		- Struct destruction functions
*		- Struct related functions
*		- Rendering functions
*		- Physics functions
*		- Data functions
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

typedef unsigned char field;

/*========== INTERNAL RESOURCES ==========*/

static HANDLE _fThread; /* thread handle */
static HANDLE _heap; /* heap handle */
static HANDLE _writeMutex; /* buffer use mutex */
static HANDLE _drawMutex;  /* draw mutex */
static HANDLE _killMutex;  /* kill mutex */
static unsigned int _sleepTime;
static int _pEnabled; /* physics toggle */
static int _killSignal;   /* thread kill signal */
static int _killRecieved; /* kill recieved signal */

/* internal buffers */

/* BIG AUXILLARY BUFFER */
static BYTE _mTank[VF_MEMTANK_SIZE + VF_MEMTANK_EXCESS];
static BYTE _mTField[VF_MEMTANK_FIELDSIZE];

/* TRANSFORM RELATED DATA */
static vfTransform* _tBuffer; static field* _tBufferField;
static vfTransform* _tFinalBuffer;
static int _tCount;

/* PARTICLE RELATED DATA */
static vfParticle* _pBuffer; static field* _pBufferField;
static int _pCount;

/* BOUND RELATED DATA */
static vfBound* _bBuffer; static field* _bBufferField;
static int _bCount;

/* ENTITY RELATED DATA */
static vfEntity* _eBuffer; static field* _eBufferField;
static int _eCount;

/* STATIC CALLBACK DATA */
static STATUPDCALLBACK _sCallBuff[VF_STATICCALLBACK_MAX];
static int _sCallSize;
static long long _tickCount = 0;

/* boundQuad struct definition */
typedef struct boundQuad
{
	unsigned short collisions;
	vfVector collisionData[VF_COLLISIONS_MAX];
	vfPhysics* collisionPhysics[VF_COLLISIONS_MAX];
	vfVector collisionEdge[VF_COLLISIONS_MAX];
	vfVector collisionTargetAverage[VF_COLLISIONS_MAX];
	vfVector collisionAccumulator;

	vfVector verts[4];
	vfVector average;

	vfBound staticData;

} boundQuad;

/* projVect struct definition */
typedef struct projVect 
{
	vfVector p1;
	vfVector p2;
	vfVector vector;
	float mag;
} projVect;

/* MUTEX RELATED FUNCTIONS */
static void showMutexError(const char* mName, const char* description)
{
	char cBuff[0xFF] = { 0 };
	sprintf(cBuff, "Could not create new object!\n"
		"Process is stuck!\nMUTEX:%s\nDESC:%s\n", mName,
		description);

	MessageBoxA(NULL, cBuff, "FATAL ENGINE ERROR",
		MB_OK);
	exit(1);
}

static inline void captureMutex(const char* errMsg)
{
	/* get write and draw mutex*/
	int result;
	result = WaitForSingleObject(_writeMutex, VF_WMUTEX_TIMEOUT);
	if (result != WAIT_OBJECT_0) showMutexError("WriteMutex", errMsg);
	result = WaitForSingleObject(_drawMutex, VF_RMUTEX_TIMEOUT);
	if (result != WAIT_OBJECT_0) showMutexError("DrawMutex", errMsg);
	return;
}

static inline void releaseMutex(void)
{
	/* release draw and write mutex */
	ReleaseMutex(_drawMutex);
	ReleaseMutex(_writeMutex);
}

/* boundquad buffer, this buffer maps ever Bound object to a quad, which */
/* is the Bound object's dimensions and offset translated by it's transform */
static boundQuad* _bqBuffer;

/* SPACE PARTITION BUFFERS */
typedef struct partition
{
	INT16  x; /* partition x */
	INT16  y; /* partition y */
	INT8   bqCount;
	INT16  bqMemSize;
	INT16* bqIndexes; /* dynamic array */
} partition;

static partition _partBuff[VF_PARTITION_COUNT];
static int _partitionCount = 0;
static int _partitionSize = 10000;

/* SPECALLOC */
static void* specAlloc(int size)
{
	void* memLoc = vfMTAlloc(size, FALSE);
	/* check mtalloc fail */
	if (!memLoc)
	{
		memLoc = HeapAlloc(_heap, NULL, size);
	}
	return memLoc;
}
/* SPECFREE (returns true on used HeapFree) */
static int specFree(void* ptr, int size)
{
	int fCheck = vfMTFree(ptr, size, FALSE);
	/* on !fcheck, malloc was used */
	if (!fCheck)
	{
		HeapFree(_heap, NULL, ptr);
		return TRUE;
	}
	return FALSE;
}
/* SPECCOPY (returns new ptr of new size) */
static void* specCopy(BYTE* ptr, int oldSize, int newSize)
{
	BYTE* newPtr = specAlloc(newSize);
	memcpy(newPtr, ptr, oldSize);
	specFree(ptr, oldSize);
	return newPtr;
}

/* PARTCHECK */
static inline int partCheck(boundQuad* bq, int rangeMax,
	int* partCountX, int* partCountY, int* partX, int* partY)
{
	/* for each vert, get it's partition coordinate */
	int pBuff[0x10][2];
	int pBuffUse = 0;
	for (int i = 0; i < 0x4; i++)
	{
		/* get each vertex but scale them out from average */
		/* to overstep each partition (intentional) */
		float centerOffsetX = bq->verts[i].x - bq->average.x;
		float centerOffsetY = bq->verts[i].y - bq->average.y;
		float posXScaled = bq->average.x + 
			(centerOffsetX * VF_PARTITION_OVERLAPSCALE);
		float posYScaled = bq->average.y +
			(centerOffsetY * VF_PARTITION_OVERLAPSCALE);

		/* get partition coordinates */
		float partPosX = posXScaled / (float)_partitionSize;
		float partPosY = posYScaled / (float)_partitionSize;

		/* assign coordinates (floored) */
		pBuff[pBuffUse][0] = (int)floorf(partPosX);
		pBuff[pBuffUse][1] = (int)floorf(partPosY);
		pBuffUse++;
		/* assign coordinates (ceiled) */
		pBuff[pBuffUse][0] = (int)ceilf(partPosX);
		pBuff[pBuffUse][1] = (int)ceilf(partPosY);
		pBuffUse++;
	}

	/* assign average position to pBuff (floored) */
	pBuff[pBuffUse][0] = (int)floorf(bq->average.x / (float)_partitionSize);
	pBuff[pBuffUse][1] = (int)floorf(bq->average.y / (float)_partitionSize);
	pBuffUse++;
	/* assign average position to pBuff (ceiled) */
	pBuff[pBuffUse][0] = (int)ceilf(bq->average.x / (float)_partitionSize);
	pBuff[pBuffUse][1] = (int)ceilf(bq->average.y / (float)_partitionSize);
	pBuffUse++;

	/* create bounding box around boundQuad */
	int minPartX = pBuff[0][0]; int maxPartX = pBuff[0][0];
	int minPartY = pBuff[0][1]; int maxPartY = pBuff[0][1];

	/* loop to find min/max */
	for (int i = 0; i < pBuffUse; i++)
	{
		minPartX = min(minPartX, pBuff[i][0]);
		maxPartX = max(maxPartX, pBuff[i][0]);
		minPartY = min(minPartY, pBuff[i][1]);
		maxPartY = max(maxPartY, pBuff[i][1]);
	}

	/* get range of X and Y*/
	int rangeX = min(rangeMax, max(maxPartX - minPartX, 1));
	int rangeY = min(rangeMax, max(maxPartY - minPartY, 1));

	/* return ranges */
	*partCountX = rangeX;
	*partCountY = rangeY;

	/* assign all in buffers */
	for (int i = 0; i < rangeX; i++)
		partX[i] = minPartX + i;
	for (int i = 0; i < rangeY; i++)
		partY[i] = minPartY + i;
}

/* ADD TO PARTITION */
static ULONGLONG resizeCheckTimer = 0;
static inline void addToPartition(boundQuad* bq, int x, int y)
{
	/* increment resize counter */
	resizeCheckTimer++;

	/* check for existing partition */
	for (int i = 0; i < _partitionCount; i++)
	{
		partition* part = _partBuff + i;
		/* skip if not collide */
		if (part->x != x || part->y != y) continue;

		/* if partition is empty, allocate for it */
		if (part->bqMemSize == 0)
		{
			part->bqMemSize += VF_PARTITION_STEP;
			part->bqIndexes = HeapAlloc(_heap, FALSE, 
				0xFF * sizeof(UINT16));
		}

		/* add next boundquad to partition */
		int bqIndex = bq - _bqBuffer;
		part->bqIndexes[part->bqCount] = bqIndex;

		/* return, since it's done */
		part->bqCount++;
		return;
	}

	/* on no partitions left, report err and exit */
	if (_partitionCount == VF_PARTITION_COUNT)
	{
		MessageBoxA(NULL, "No physics partitions left!\n",
			"CRITICAL ENGINE FAILURE", MB_OK);
		exit(1);
	}

	/* on exit loop without adding to partition, create new partition */
	partition* part = _partBuff + _partitionCount; /* gets next free part */
	part->x = x; part->y = y;
	_partitionCount++;

	/* if partition is empty, allocate for it */
	if (part->bqMemSize == 0)
	{
		part->bqMemSize += VF_PARTITION_STEP;
		part->bqIndexes = specAlloc(part->bqMemSize * sizeof(INT16));
	}

	/* add next boundquad to partition */
	int bqIndex = bq - _bqBuffer;
	part->bqIndexes[part->bqCount] = bqIndex;
	part->bqCount++;
}

/* ASSIGN PARTITION */
static void assignPartition(boundQuad* bq)
{
	/* buffers for later */
	int pCountX = 0;
	int pCountY = 0;
	int pBuffX[VF_ENT_PARTITIONS_MAX];
	int pBuffY[VF_ENT_PARTITIONS_MAX];

	/* check which part the bq is part of */
	partCheck(bq, VF_ENT_PARTITIONS_MAX, &pCountX, &pCountY, pBuffX,
		pBuffY);

	/* add bq to all given parts */
	for (int rows = 0; rows < pCountX; rows++)
		for (int cols = 0; cols < pCountY; cols++)
			addToPartition(bq, pBuffX[rows], pBuffY[cols]);
}

/*========================================*/

/* INTERNAL BOUNDQUAD CREATION FUNCTION */
static inline boundQuad createBoundQuad(vfVector bL, vfVector tL, vfVector tR,
	vfVector bR)
{
	boundQuad bQ;
	bQ.collisions = 0;
	bQ.verts[BL] = bL;
	bQ.verts[TL] = tL;
	bQ.verts[TR] = tR;
	bQ.verts[BR] = bR;
	bQ.average = VECT(0, 0);
	bQ.collisionAccumulator = VECT(0, 0);
	
	return bQ;
}

/* INTERNAL PROJVECT CREATION FUNCTION */
static inline projVect createProjVect(vfVector p1, vfVector p2) 
{
	projVect rVec;
	rVec.p1 = p1;
	rVec.p2 = p2;
	rVec.vector = VECT(p2.x - p1.x, p2.y - p1.y);
	rVec.mag = sqrtf(powf(rVec.vector.x, 2) + powf(rVec.vector.y, 2));
	return rVec;
}

/* VERTEX ROTATION FUNCTION */
static inline vfVector vertRotateScale(vfVector vertex, float angle,
	float scale)
{
	/* convert to polar */
	const float sqrX = powf(vertex.x, 2.0f);
	const float sqrY = powf(vertex.y, 2.0f);
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

/* VECTOR DOT PRODUCT FUNCTION */
static inline float vectorDotProduct(vfVector v1, vfVector v2)
{
	return (v1.x * v2.x) + (v1.y * v2.y);
}

/* VECTOR MAGNITUDE FUNCTION */
static inline float vectorMagnitude(vfVector vec)
{
	return sqrtf(powf(vec.x, 2) + powf(vec.y, 2));
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
static inline int findBufferSpot(void* buffer, field* field, 
	size_t structSize)
{
	/* check for NULL */
	if (buffer == NULL || field == NULL)
	{
		/* CRITICAL ENGINE FAILURE! */
		MessageBoxA(NULL, "Engine Not Initialized Properly!",
			"CRITICAL ENGINE FAILURE!", MB_OK);
		exit(1);
	}

	/* get buffer size */
	int bufSize = 0;
	if (buffer == _tBuffer) bufSize = _tCount;
	if (buffer == _bBuffer) bufSize = _bCount;
	if (buffer == _pBuffer) bufSize = _pCount;
	if (buffer == _eBuffer) bufSize = _eCount;


	/* find empty spot within field */
	int startIndex = bufSize / 2;
	for (int i = 0; i < VF_BUFFER_SIZE; i++)
	{
		/* calculate searchIndex */
		int searchIndex = (startIndex + i) % VF_BUFFER_SIZE;

		/* empty spot */
		if ((field)[searchIndex] == 0) return searchIndex;
	}

	/* CRITICAL ENGINE FAILURE: */
	char bufferName[0x20] = { 0 };
	char errMsg[0x80] = { 0 };
	
	/* set buffername */
	strcpy(bufferName, "Unkown");
	if (buffer == _tBuffer) strcpy(bufferName, "Transform");
	if (buffer == _pBuffer) strcpy(bufferName, "Particle");
	if (buffer == _bBuffer) strcpy(bufferName, "Bound");
	if (buffer == _eBuffer) strcpy(bufferName, "Entity");

	/* messagebox failure and exit */
	sprintf(errMsg, "%s Buffer Full!", bufferName);
	MessageBoxExA(NULL, errMsg,
		"CRITICAL ENGINE FAILURE", MB_OK, 0);
	exit(1);
}

/* ========== FINAL TRANSFORM HANDLING FUNCTION ========== */
static inline void updateFinalTransforms(void)
{
	/* ===== update FINAL transform objects ===== */
	for (int i = 0; i < VF_BUFFER_SIZE; i++)
	{
		/* check if empty */
		if (!_tBufferField[i]) continue;

		/* else, update final transform values */
		/* recall that there's an identical transform buffer */
		/* except all it's values correspond to their global value */
		/* based on their parent's values */
		vfTransform tValue = _tBuffer[i];
		vfTransform* tParent = tValue.parent;
		int pCounter = 0; /* parent counter */

		while (tParent != VF_NOPARENT)
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
}

/* ========= ENTITY VELOCITY DAMPENING FUNCTION ========== */
static inline void updateEntityVelocities(void)
{
	for (int i = 0; i < VF_BUFFER_SIZE; i++)
	{
		/* grab entity */
		if (!_eBufferField[i]) continue;

		vfEntity* ent = _eBuffer + i;

		/* if inactive, skip */
		if (!ent->physics.active) continue;

		/* update transform members */
		vfPhysics* pObj = &(ent->physics);
		vfTransform* entityTransform = ent->transform;
		entityTransform->position.x += pObj->velocity.x;
		entityTransform->position.y += pObj->velocity.y;
		entityTransform->rotation += pObj->tourque;

		/* update physics members */
		pObj->velocity.x *= (1.0f - pObj->drag);
		pObj->velocity.y *= (1.0f - pObj->drag);
		pObj->tourque *= (1.0f - pObj->drag);

		/* clamp tourque */
		if (pObj->tourque > VF_TOURQUE_MAX) pObj->tourque = VF_TOURQUE_MAX;
		if (pObj->tourque < -VF_TOURQUE_MAX) pObj->tourque = -VF_TOURQUE_MAX;

		/* if exists, invoke callback */
		if (ent->updateCallback != NULL)
		{
			ent->updateCallback(ent);
		}
	}
}

/* ========== BOUNDQUAD HANDLING FUNCTION ========== */
static inline void updateBoundquadValues(void)
{
	for (int i = 0; i < VF_BUFFER_SIZE; i++)
	{
		if (!_bBufferField[i]) continue;

		/* grab bound to convert */
		vfBound toConvert = _bBuffer[i];

		/* skip if inactive */
		if (!toConvert.active) continue;

		/* grab final transform */
		vfTransform tFinal = _tFinalBuffer[toConvert.body - _tBuffer];

		/* create initial boundQuad */
		vfVector pos = toConvert.position;
		vfVector dims = toConvert.dimensions;
		dims.x += pos.x; dims.y += pos.y;
		boundQuad bQuad = createBoundQuad(VECT(pos.x, pos.y), VECT(pos.x, dims.y),
			VECT(dims.x, dims.y), VECT(dims.x, pos.y));
		bQuad.staticData = toConvert;

		/* offset bQuad by transform and rotate by angle */
		for (int j = 0; j < 4; j++)
		{
			bQuad.verts[j] = vertRotateScale(bQuad.verts[j], tFinal.rotation,
				tFinal.scale);
			bQuad.verts[j].x += tFinal.position.x;
			bQuad.verts[j].y += tFinal.position.y;
		}

		/* clear bQuad collision data */
		bQuad.collisionAccumulator = VECT(0, 0);
		bQuad.collisions = 0;

		/* calculate average */
		bQuad.average = vertexAverage(bQuad.verts, 4);

		/* assign bQuad to respective buffer index */
		_bqBuffer[i] = bQuad;
	}
}

/* ========== COLLISIONCHECK FUNCTION ========== */
static inline int collisionCheck(boundQuad* source, boundQuad* target)
{
	/* check if max collisions reached */
	if (source->collisions == VF_COLLISIONS_MAX) return 0;
	if (target->collisions == VF_COLLISIONS_MAX) return 0;

	/* if bounds not active, ignore */
	if (!source->staticData.active) return 0; 
	if (!target->staticData.active) return 0;

	/* if the two average positions are very similar, move one over */
	if (fabsf(source->average.x - target->average.x) < VF_POSITION_SIMILARITY
		|| fabsf(source->average.y - target->average.y) <
		VF_POSITION_SIMILARITY)
	{
		source->average.x += VF_POSITION_SIMILARITY;
		source->average.y += VF_POSITION_SIMILARITY;
		target->average.x -= VF_POSITION_SIMILARITY;
		target->average.y -= VF_POSITION_SIMILARITY;
	}

	/* grab the vector from target to source, this will be */
	/* handy later */
	vfVector avgVect = VECT(source->average.x - target->average.x,
		source->average.y - target->average.y);

	/* if avgVect is greater than we can reasonably expect 2 colliding objects */
	/* to be in proximity of each other, then skip */
	float avgVectSize = fabsf(avgVect.x + avgVect.y);
	float sourceMag = (source->staticData.dimensions.x +
		source->staticData.dimensions.y) * source->staticData.body->scale;
	float targetMag = (target->staticData.dimensions.x +
		target->staticData.dimensions.y) * target->staticData.body->scale;
	float magThreshold = sourceMag + targetMag;
	if (avgVectSize > magThreshold) return 0;

	/* variable to keep track of smallest pushback vector */
	float smallestMag = -1;
	vfVector smallestPVect = VECT(0, 0);
	vfVector smallestEdgeVect = VECT(0, 0);

	/* firstly, get all edges to project vertexes onto */
	projVect projBuff[8];
	for (int i = 0; i < 4; i++)
	{
		projBuff[i] = createProjVect(source->verts[i],
			source->verts[(i + 1) % 4]);
		projBuff[i + 4] = createProjVect(target->verts[i],
			target->verts[(i + 1) % 4]);
	}

	/* FOR EVERY EDGE TO PROJECT */
	int collisionCount = 0;
	for (int i = 0; i < 8; i++) 
	{
		/* grab edge vec and normalize */
		projVect edgeData = projBuff[i];
		vfVector edgeVector = edgeData.vector;
		edgeVector.x /= edgeData.mag;
		edgeVector.y /= edgeData.mag;

		/* FOR ALL SOURCE VERTS */
		float sMin;
		float sMax;
		for (int j = 0; j < 4; j++) 
		{
			/* get vector to project and normalize */
			vfVector toProject = VECT(source->verts[j].x - edgeData.p1.x,
				source->verts[j].y - edgeData.p1.y);

			toProject.x /= edgeData.mag;
			toProject.y /= edgeData.mag;

			/* get dot product */
			float dProduct = vectorDotProduct(edgeVector, toProject);

			/* set new min/max */
			if (j == 0)
			{
				sMin = dProduct;
				sMax = dProduct;
			}
			else
			{
				sMin = min(sMin, dProduct);
				sMax = max(sMax, dProduct);
			}
		} /* SOURCE VERT LOOP END */

		/* FOR ALL TARGET VERTS */
		float tMin;
		float tMax;
		for (int j = 0; j < 4; j++)
		{
			/* get vector to project and normalize */
			vfVector toProject = VECT(target->verts[j].x - edgeData.p1.x,
				target->verts[j].y - edgeData.p1.y);

			toProject.x /= edgeData.mag;
			toProject.y /= edgeData.mag;

			/* get dot product */
			float dProduct = vectorDotProduct(edgeVector, toProject);

			/* set new min/max */
			if (j == 0)
			{
				tMin = dProduct;
				tMax = dProduct;
			}
			else
			{
				tMin = min(tMin, dProduct);
				tMax = max(tMax, dProduct);
			}
		} /* TARGET VERT LOOP END */

		/* compare if source/target projections are overlapping */
		if (max(sMax, tMax) - min(sMin, tMin) <= (sMax - sMin) + (tMax - tMin))
		{
			collisionCount++;

			/* get gap value */
			float gap = min(sMax, tMax) - max(sMin, tMin);

			/* grab pushback vector */
			vfVector pVect = edgeData.vector;
			pVect.x *= gap;
			pVect.y *= gap;
			float pVectMag = vectorMagnitude(pVect); /* get magnitude */

			/* make sure vector is relevant */
			if (vectorDotProduct(pVect, avgVect) >= 0)
			{
				/* discard if not the smallest possible vector */
				if (smallestMag == -1)
				{
					smallestMag = pVectMag;
					smallestPVect = pVect;
					smallestEdgeVect = edgeVector;
				}
				else
				{
					if ((min(smallestMag, pVectMag)) == pVectMag)
					{
						smallestMag = pVectMag;
						smallestPVect = pVect;
						smallestEdgeVect = edgeVector;
					}
				}

			} /* END VECTOR CHECK */
		} /* END OVERLAP CHECK */
		else
		{
			return 0;
		}

	} /* EDGE LOOP END */

	/* IF OVERLAP */
	if (collisionCount == 8)
	{
		/* check if target is missing entity */
		if (target->staticData.entity == VF_NOENTITY) return 1;

		/* carry over physics data */
		if (source->staticData.entity == VF_NOENTITY)
		{
			/* if bound is missing physics, set ptr to null */
			target->collisionPhysics[target->collisions] = NULL;
		}
		else
		{
			target->collisionPhysics[target->collisions] =
				&source->staticData.entity->physics;
		}

		/* check if target is static */
		if (!target->staticData.entity->physics.moveable) return 1;
		if (!target->staticData.entity->physics.active) return 1;

		/* check for NaN */
		if (isnan(smallestPVect.x)) smallestPVect.x = 0;
		if (isnan(smallestPVect.y)) smallestPVect.y = 0;

		/* get pushvector ratio */
		vfPhysics* targetPhys = &(target->staticData.entity->physics);
		vfPhysics* sourcePhys = &(target->staticData.entity->physics);
		float massTotal = targetPhys->mass + sourcePhys->mass;
		float massPercent = 1.0f - (targetPhys->mass / massTotal);

		/* if current is immovable or physics is inactive, massPercent is 1.0 */
		if (!sourcePhys->moveable ||
			!sourcePhys->active)
		{
			massPercent = 1.0f;
		}

		/* negative pushback vector */
		target->collisionData[target->collisions].x = -(smallestPVect.x * massPercent);
		target->collisionData[target->collisions].y = -(smallestPVect.y * massPercent);
		target->collisionEdge[target->collisions] = smallestEdgeVect;
		target->collisionTargetAverage[target->collisions] = source->average;
		target->collisions++;

		/* call collision callback */
		if (source->staticData.entity != VF_NOENTITY)
		{
			vfEntity* cbObject = source->staticData.entity;
			if (cbObject->collisionCallback != NULL)
			{
				cbObject->collisionCallback(source->staticData.entity,
					target->staticData.entity);
			}
		}
	}

	return 1;
}

/* ======== COLLISION HANDLING FUNCTION ======== */
static inline void updateCollisions(void)
{
	/* first, clear partition buffer data */
	_partitionCount = 0;
	for (int i = 0; i < VF_PARTITION_COUNT; i++)
		_partBuff[i].bqCount = 0;

	/* update all partitions */
	int boundCounter = 0;
	for (int i = 0; i < VF_BUFFER_SIZE; i++)
	{
		/* break if boundCounter exceeded boundCount */
		if (boundCounter > _bCount) break;

		/* if bound is not used, skip */
		if (!_bBufferField[i]) continue;

		/* if bounds is not active, skip */
		if (!_bqBuffer[i].staticData.active) continue;

		/* assign bounds to a partition */
		assignPartition(_bqBuffer + i);

		/* increment boundCounter*/
		boundCounter++;
	}

	/* loop all partitions */
	for (int partIndex = 0; partIndex < _partitionCount; partIndex++)
	{
		/* get current physics partition */
		partition currentPart = _partBuff[partIndex];

		/* if partition has size 1 or less, skip */
		if (currentPart.bqCount <= 1) continue;

		/* loop all boundquads within partition */
		/* sbIndex stands for "source boundquad index" */
		for (int sbIndex = 0; sbIndex < currentPart.bqCount; sbIndex++)
		{
			/* if source bounds unused, skip */
			/* this shouldn't happen but i check anyways */
			if (!_bBufferField[currentPart.bqIndexes[sbIndex]])
				continue;

			/* if source bounds inactive, skip */
			if (!_bqBuffer[currentPart.bqIndexes[sbIndex]]
				.staticData.active) continue;

			/* compare source boundquad against all other boundquads */
			/* within the partition tbIndex stands for "target bq indx" */
			for (int tbIndex = 0; tbIndex < currentPart.bqCount; tbIndex++)
			{
				/* if target bound is unused, continue */
				if (!_bBufferField[currentPart.bqIndexes[tbIndex]]) continue;

				/* if target bound is inactive, continue */
				if (!_bqBuffer[currentPart.bqIndexes[tbIndex]]
					.staticData.active) continue;

				/* if target and source are the same, continue */
				if (sbIndex == tbIndex) continue;

				/* if for some reason the source and target are repeated */
				/* multiple times in the partition, also skip it */
				boundQuad* sourcePtr = _bqBuffer +
					currentPart.bqIndexes[sbIndex];
				boundQuad* targetPtr = _bqBuffer +
					currentPart.bqIndexes[tbIndex];
				if (sourcePtr == targetPtr) continue;

				/* perform collision check */
				collisionCheck(sourcePtr, targetPtr);
			}
		}
	}

	/* ACCUMULATE COLLISION DATA */
	int accCount = 0;
	for (int i = 0; i < VF_BUFFER_SIZE; i++)
	{
		/* if not active, skip */
		if (accCount >= _bCount) break;
		if (!_bBufferField[i]) continue;
		if (_bqBuffer[i].staticData.entity == VF_NOENTITY) continue;
		if (!_bqBuffer[i].staticData.entity->physics.active) continue;

		for (int k = 0; k < _bqBuffer[i].collisions; k++)
		{
			if (isnan(_bqBuffer[i].collisionData[k].x) ||
				isnan(_bqBuffer[i].collisionData[k].y)) continue;
			_bqBuffer[i].collisionAccumulator.x += _bqBuffer[i].collisionData[k].x;
			_bqBuffer[i].collisionAccumulator.y += _bqBuffer[i].collisionData[k].y;
		}

		accCount++;
	}

	/* APPLY COLLISION DATA */
	int applCount = 0;
	for (int i = 0; i < VF_BUFFER_SIZE; i++)
	{
		/* if not used or not active, skip */
		if (applCount >= _bCount) break;
		if (!_bBufferField[i]) continue;
		if (_bqBuffer[i].staticData.entity == VF_NOENTITY) continue;
		if (_bqBuffer[i].staticData.entity == NULL) continue;
		if (!_bqBuffer[i].staticData.entity->physics.active) continue;

		vfVector pushBack = _bqBuffer[i].collisionAccumulator;
		float pMag = vectorMagnitude(pushBack);
		if (pMag > VF_PUSHBACK_MAGNITUDE_MAX)
		{
			pushBack.x /= pMag;
			pushBack.y /= pMag;
			pushBack.x *= VF_PUSHBACK_MAGNITUDE_MAX;
			pushBack.y *= VF_PUSHBACK_MAGNITUDE_MAX;
		}

		/* test for NaN */
		if (!isnan(pushBack.x) && !isnan(pushBack.y))
		{
			_bBuffer[i].body->position.x += pushBack.x;
			_bBuffer[i].body->position.y += pushBack.y;
		}

		applCount++;
	} /* COLLISION APPLICATION LOOP END */
}

/* ========== ENERGY TRANSFER SIMULATION FUNCTION ========== */
static inline void updateCollisionVelocities(void)
{
	/* loop through all objects with collisions */
	int bCheckCount = 0;
	for (int i = 0; i < VF_BUFFER_SIZE; i++)
	{
		/* if checked all bounds, end */
		if (bCheckCount >= _bCount) return;
		if (!_bBufferField[i]) continue; /* check active */
		if (!_bqBuffer[i].collisions) continue; /* check collided */
		/* if physics inactive, avoid */
		if (_bqBuffer[i].staticData.entity == VF_NOENTITY) continue;
		if (!_bqBuffer[i].staticData.entity->physics.active) continue; 

		/* loop through all collision data */
		for (int j = 0; j < _bqBuffer[i].collisions; j++)
		{
			/* get physics and quad data */
			boundQuad* currentQuad = _bqBuffer + i;
			vfPhysics* currentPhysics = &(currentQuad->staticData.entity->physics);

			/* account for missing physics object */
			vfPhysics targetPhysics;
			if (currentQuad->collisionPhysics[j] == NULL)
			{
				targetPhysics = PHYSA(0, 0, 0, 0, 1);
			}
			else
			{
				targetPhysics = *(currentQuad->collisionPhysics[j]);
			}

			/* get mass ratio */
			float massTotal = currentPhysics->mass + targetPhysics.mass;
			float currentRatio = currentPhysics->mass / massTotal;
			float targetRatio = targetPhysics.mass / massTotal;

			/* if target is immovable or physics inactive */
			/* change ratio */
			if (!targetPhysics.active || !targetPhysics.moveable)
			{
				currentRatio = 0;
				targetRatio = 1;
			}

			/* weighted average the velocities accordingly */
			vfVector velCurrent = currentPhysics->velocity;
			vfVector velTarget = targetPhysics.velocity;
			float weightedVX = (velCurrent.x * currentRatio) + 
				(velTarget.x * targetRatio);
			float weightedVY = (velCurrent.y * currentRatio) +
				(velTarget.y * targetRatio);
			
			/* get direction towards current */
			vfVector targetAverage = _bqBuffer[i].collisionTargetAverage[j];
			vfVector offsetVec = VECT(targetAverage.x - currentQuad->average.x,
				targetAverage.y - currentQuad->average.y);

			/* only update velocity if it is towards current */
			if (vectorDotProduct(VECT(weightedVX, weightedVY), offsetVec) 
				< VF_VECTOR_SIMILARITY_THRESOLD)
			{
				currentPhysics->velocity = VECT(weightedVX, weightedVY);
			}

			/* === BOUNCE === */
			/* only bounce if current velocity is headed towards target */
			if (vectorDotProduct(currentPhysics->velocity, offsetVec) > 
				-VF_VECTOR_SIMILARITY_THRESOLD)
			{
				/* account for bounce */
				/* grab current velocity and reflect it based on collision edge */
				vfVector toReflect = currentPhysics->velocity;

				/* get collision edge and normalize */
				vfVector collisionEdge = currentQuad->collisionEdge[j];
				float cEdgeMag = vectorMagnitude(collisionEdge);
				collisionEdge.x /= cEdgeMag;
				collisionEdge.y /= cEdgeMag;

				/* reflect current vector */
				float dProd = 2.0f * vectorDotProduct(toReflect, collisionEdge);
				collisionEdge.x *= dProd;
				collisionEdge.y *= dProd;
				toReflect.x -= collisionEdge.x;
				toReflect.y -= collisionEdge.y;

				float weightX = (currentPhysics->velocity.x * 
					(1.0f - currentPhysics->bounciness)) + 
					(toReflect.x * (currentPhysics->bounciness));
				float weightY = (currentPhysics->velocity.y *
					(1.0f - currentPhysics->bounciness)) +
					(toReflect.y * (currentPhysics->bounciness));

				currentPhysics->velocity.x = weightX;
				currentPhysics->velocity.y = weightY;

			} /* END BOUNCE CONDITION */

			/* get target inital and predicted velocity */
			vfVector tPosInitial = targetAverage;
			vfVector tPosPredict = VECT(tPosInitial.x + targetPhysics.velocity.x,
				tPosInitial.y + targetPhysics.velocity.y);

			/* magnitude and dot product gate */
			float tVelMag = vectorMagnitude(targetPhysics.velocity);
			float dProduct = vectorDotProduct(offsetVec, targetPhysics.velocity);

			/* APPLY TOURQUE */
			if (tVelMag > VF_TOURQUE_MIN_VELOCITY && dProduct < 0)
			{
				/* get position relative to current average */
				vfVector v0 = VECT(tPosInitial.x - currentQuad->average.x,
					tPosInitial.y - currentQuad->average.y);
				vfVector v1 = VECT(tPosPredict.x - currentQuad->average.x,
					tPosPredict.y - currentQuad->average.y);

				/* get change in angle */
				float angle0 = atan2f(v0.y, v0.x);
				float angle1 = atan2f(v1.y, v1.x);
				float tourque = angle1 - angle0;

				/* scale by mass */
				float cTourque = tourque * targetRatio;
				cTourque *= tVelMag;
				float tTourque = tourque * currentRatio;
				tTourque *= vectorMagnitude(currentPhysics->velocity);

				if (!currentPhysics->rotationLock)
				{
					currentPhysics->tourque += cTourque;
				}
				if (!targetPhysics.rotationLock)
				{
					currentQuad->collisionPhysics[j]->tourque += tTourque;
				}
			} /* TOURQUE GATE END */
		} /* END COLLISION DATA LOOP */

		bCheckCount++;
	} /* END COLLISION OBJECT LOOP */
}

/* ===== MODULE MAIN FUNCTION ===== */
static int _updateTime = 0;
static DWORD WINAPI vfMain(void* params)
{
	ULONGLONG lastTime = 0;
	
	/* get ahold of kill mutex */
	int result = WaitForSingleObject(_killMutex, NULL);

	while (TRUE)
	{
		/* sleep (optional) */
		if (_sleepTime)
		{
			ULONGLONG currentTime = GetTickCount64();
			if (currentTime < lastTime + _sleepTime)
			{
				Sleep(1);
				continue;
			}
			else
			{
				lastTime = currentTime;
			}
		}

		/* WAIT FOR BUFFER OWNERSHIP */
		int mutStatus = WaitForSingleObject(_writeMutex, VF_WMUTEX_TIMEOUT);
		if (mutStatus != WAIT_OBJECT_0)
		{
			char errBuff[0x200];

			/* for every bound there is a boundQuad */
			int bMemSize = (sizeof(vfBound) + sizeof(boundQuad)) *
				VF_BUFFER_SIZE;
			/* multiply by 2 for tbuffer & tfinal */
			int tMemSize = sizeof(vfTransform) * VF_BUFFER_SIZE * 2;
			int eMemSize = sizeof(vfEntity) * VF_BUFFER_SIZE;

			/* err log */
			sprintf(errBuff, "Error Code: %d\nBounds Used/Allocated: %d/%d\n"
				"Transforms Used/Allocated: %d/%d\nEntities Used/Allocated: %d/%d\n"
				"Bytes allocated for Bounds: %d\n"
				"Bytes allocated for Transforms: %d\n"
				"Bytes allocated for Entities: %d\n", GetLastError(),
				_bCount, VF_BUFFER_SIZE, _tCount, VF_BUFFER_SIZE,
				_eCount, VF_BUFFER_SIZE, bMemSize,
				tMemSize, eMemSize);

			/* send messageboxes */
			MessageBoxA(NULL, "[vfMain] Thread timewout in VFramework.dll",
				"FATAL ERROR", MB_OK);
			MessageBoxA(NULL, errBuff, "ERROR INFO", MB_OK);
			exit(1);
		}

		if (_pEnabled)
		{
			ULONGLONG startTickCount = GetTickCount64();

			/* update fTransform objects */
			updateFinalTransforms();

			/* update boundquad values */
			updateBoundquadValues();

			/* dampen entity velocities */
			updateEntityVelocities();

			/* handle all collisions */
			updateCollisions();

			/* handle velocity changes from collisions */
			updateCollisionVelocities();

			/* call all callbacks */
			int callCount = 0;
			for (int i = 0; i < VF_STATICCALLBACK_MAX; i++)
			{
				/* if null, continue */
				if (_sCallBuff[i] == NULL) continue;

				/* get callback and invoke */
				STATUPDCALLBACK callBck = _sCallBuff[i];
				callBck(_tickCount);

				/* increment callcount and exit if done */
				callCount++;
				if (callCount >= _sCallSize) break;
			}

			ULONGLONG endTickCount = GetTickCount64();
			_updateTime = endTickCount - startTickCount;
		}
		else
		{
			_updateTime = -1;
		}

		/* RELEASE BUFFER OWNERSHIP */
		if (!ReleaseMutex(_writeMutex))
		{
			char errBuffer[0xFF];
			sprintf(errBuffer, "[vfmain] Mutex relase failed! Err code: %d",
				GetLastError());
			MessageBoxA(NULL, errBuffer, "FATAL ERROR", MB_OK);
			exit(1);
		}

		/* increment tick count */
		_tickCount++;

		/* check for kill signal */
		if (_killSignal)
		{
			_killRecieved = TRUE;
			ReleaseMutex(_killMutex);
			ExitThread(0);
		}
	}
}

/* INIT AND TERMINATE FUNCTIONS */
VFAPI void vfInit(void)
{
	/* get process heap */
	_heap = GetProcessHeap();

	/* clear callback and callback size */
	ZeroMemory(_sCallBuff, sizeof(_sCallBuff));
	_sCallSize = 0;
	_tickCount = 0;

	/* init module internal data */
	_sleepTime = 0;
	_pEnabled = 0;
	_tCount = 0;
	_bCount = 0;
	_pCount = 0;
	_eCount = 0;

	/* init mutexs */
	_writeMutex = CreateMutexW(NULL, FALSE, NULL);
	_killMutex  = CreateMutexW(NULL, FALSE, NULL);
	_drawMutex  = CreateMutexW(NULL, FALSE, NULL);
	if (_writeMutex == NULL || _killMutex == NULL
		|| _writeMutex == NULL)
	{
		MessageBoxA(NULL, "Failed to create thread mutex object",
			"FATAL ERR", MB_OK);
		exit(1);
	}

	/* INIT TRANSFORM BUFFER */
	_tBuffer = HeapAlloc(_heap, HEAP_ZERO_MEMORY,
		sizeof(vfTransform) * VF_BUFFER_SIZE);
	_tBufferField = HeapAlloc(_heap, HEAP_ZERO_MEMORY,
		sizeof(field) * VF_BUFFER_SIZE);

	/* INIT FINAL TRANSFORM BUFFER */
	_tFinalBuffer = HeapAlloc(_heap, HEAP_ZERO_MEMORY,
		sizeof(vfTransform) * VF_BUFFER_SIZE);

	/* INIT PARTICLE BUFFER */
	_pBuffer = HeapAlloc(_heap, HEAP_ZERO_MEMORY,
		sizeof(vfParticle) * VF_BUFFER_SIZE);
	_pBufferField = HeapAlloc(_heap, HEAP_ZERO_MEMORY,
		sizeof(field) * VF_BUFFER_SIZE);

	/* INIT BOUND BUFFER */
	_bBuffer = HeapAlloc(_heap, HEAP_ZERO_MEMORY,
		sizeof(vfBound) * VF_BUFFER_SIZE);
	_bBufferField = HeapAlloc(_heap, HEAP_ZERO_MEMORY,
		sizeof(field) * VF_BUFFER_SIZE);

	/* INIT BOUNDQUAD BUFFER */
	_bqBuffer = HeapAlloc(_heap, HEAP_ZERO_MEMORY,
		sizeof(boundQuad) * VF_BUFFER_SIZE);

	/* INIT ENTITY BUFFER */
	_eBuffer = HeapAlloc(_heap, HEAP_ZERO_MEMORY,
		sizeof(vfEntity) * VF_BUFFER_SIZE);
	_eBufferField = HeapAlloc(_heap, HEAP_ZERO_MEMORY,
		sizeof(field) * VF_BUFFER_SIZE);

	/* init module main thread */
	_killSignal   = FALSE;
	_killRecieved = FALSE;
	_fThread = CreateThread(0, 0, vfMain, 0, 0, &_fThread);
}

VFAPI void vfTerminate(void)
{
	/* send kill signal */
	_killSignal = TRUE;

	/* wait until signal is recieved */
	while (_killRecieved == FALSE);

	/* wait for kill mutex */
	WaitForSingleObject(_killMutex, 0xFF);

	/* free all buffers */
	HeapFree(_heap, 0, _tBuffer);
	HeapFree(_heap, 0, _bBuffer);
	HeapFree(_heap, 0, _pBuffer);
	HeapFree(_heap, 0, _eBuffer);
	HeapFree(_heap, 0, _bqBuffer);
	HeapFree(_heap, 0, _tFinalBuffer);

	/* free all buffer fields */
	HeapFree(_heap, 0, _tBufferField);
	HeapFree(_heap, 0, _bBufferField);
	HeapFree(_heap, 0, _pBufferField);
	HeapFree(_heap, 0, _eBufferField);

	/* close handles */
	CloseHandle(_writeMutex);
	CloseHandle(_killMutex);
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
	int tIndex = findBufferSpot(_tBuffer, _tBufferField,
		sizeof(vfTransform));

	/* fill field and increment counter */
	_tBufferField[tIndex] = 1;
	_tCount++;

	/* set values */
	vfTransform* rTransform = _tBuffer + tIndex;
	rTransform->position = vector;
	rTransform->parent = VF_NOPARENT;

	return rTransform;
}

VFAPI vfTransform* vfCreateTransforma(vfVector vector, float rotation,
	float scale)
{

	/* find transform buffer spot */
	int tIndex = findBufferSpot(_tBuffer, _tBufferField,
		sizeof(vfTransform));
	_tBufferField[tIndex] = 1;
	_tCount++;

	/* set values */
	vfTransform* rTransform = _tBuffer + tIndex;
	rTransform->position = vector;
	rTransform->rotation = rotation;
	rTransform->scale = scale;
	rTransform->parent = VF_NOPARENT;

	return rTransform;
}

VFAPI vfTransform* vfCreateTransformp(vfTransform* parent)
{

	/* find transform buffer spot */
	int tIndex = findBufferSpot(_tBuffer, _tBufferField,
		sizeof(vfTransform));
	_tBufferField[tIndex] = 1;
	_tCount++;

	/* set values */
	vfTransform* rTransform = _tBuffer + tIndex;
	rTransform->parent = parent;

	return rTransform;
}

VFAPI vfBound* vfCreateBoundt(vfTransform* body)
{

	/* find bound buffer spot */
	int bIndex = findBufferSpot(_bBuffer, _bBufferField,
		sizeof(vfBound));
	_bBufferField[bIndex] = 1;
	_bCount++;

	/* set value */
	vfBound* rBound = _bBuffer + bIndex;
	rBound->body = body;
	rBound->active = TRUE;
	rBound->entity = VF_NOENTITY;

	return rBound;
}

VFAPI vfBound* vfCreateBounda(vfTransform* body, vfVector position,
	vfVector dimensions)
{

	/* find bound buffer spot */
	int bIndex = findBufferSpot(_bBuffer, _bBufferField,
		sizeof(vfBound));

	_bBufferField[bIndex] = 1;
	_bCount++;

	/* set value */
	vfBound* rBound = _bBuffer + bIndex;
	rBound->body = body;
	rBound->position = position;
	rBound->dimensions = dimensions;
	rBound->active = TRUE;
	rBound->entity = VF_NOENTITY;

	return rBound;
}

VFAPI vfParticle* vfCreateParticlet(vfTransform* transform)
{
	captureMutex("Particle Creation Timeout");

	/* get free spot */
	int pIndex = findBufferSpot(_pBuffer, _pBufferField,
		sizeof(vfParticle));
	_pBufferField[pIndex] = 1;
	_pCount++;

	/* set values */
	vfParticle* rParticle = _pBuffer + pIndex;
	rParticle->layer = 0;
	rParticle->active = TRUE;
	rParticle->filter = vfCreateColor(255, 255, 255, 255);
	rParticle->transform = transform;

	releaseMutex( );
	return rParticle;
}

VFAPI vfParticle* vfCreateParticlea(vfTransform* transform, vgTexture texture,
	vgShape shape, unsigned char layer)
{
	captureMutex("Particle creation timeout");

	/* get free spot */
	int pIndex = findBufferSpot(_pBuffer, _pBufferField,
		sizeof(vfParticle));
	_pBufferField[pIndex] = 1;
	_pCount++;

	/* set values */
	vfParticle* rParticle = _pBuffer + pIndex;
	rParticle->layer = layer;
	rParticle->filter = vfCreateColor(255, 255, 255, 255);
	rParticle->texture = texture;
	rParticle->shape = shape;
	rParticle->transform = transform;

	releaseMutex( );
	return rParticle;
}

VFAPI vfPhysics vfCreatePhysics(float bounciness, float drag, float mass)
{
	vfPhysics rPhys;
	rPhys.bounciness = bounciness;
	rPhys.drag = drag;
	rPhys.mass = mass;
	rPhys.moveable = TRUE;
	rPhys.velocity = VECT(0, 0);
	rPhys.tourque = FALSE;
	rPhys.rotationLock = FALSE;
	rPhys.active = TRUE;

	return rPhys;
}

VFAPI vfPhysics vfCreatePhysicsa(float bounciness, float drag, float mass,
	int moveable, int rotationLock)
{
	vfPhysics rPhys;
	rPhys.bounciness = bounciness;
	rPhys.drag = drag;
	rPhys.mass = mass;
	rPhys.moveable = moveable;
	rPhys.velocity = VECT(0, 0);
	rPhys.tourque = 0;
	rPhys.rotationLock = rotationLock;
	rPhys.active = TRUE;
	
	return rPhys;
}

VFAPI vfEntity* vfCreateEntity(unsigned char layer, vgShape shape,
	vgTexture texture, vfPhysics physics, vfVector boundPosition,
	vfVector boundDimensions)
{
	captureMutex("Entity creation timeout");

	/* find free spot */
	int eIndex = findBufferSpot(_eBuffer, _eBufferField,
		sizeof(vfEntity));
	_eBufferField[eIndex] = 1;
	_eCount++;
	
	/* get return entity and init values */
	vfEntity* rEnt = _eBuffer + eIndex;
	rEnt->active = TRUE;

	rEnt->transform = vfCreateTransforma(vfCreateVector(0, 0), 0, 1);
	rEnt->bounds = vfCreateBounda(rEnt->transform, boundPosition,
		boundDimensions);
	rEnt->layer = layer;
	rEnt->filter = vfCreateColor(255, 255, 255, 255);
	rEnt->physics = physics;
	rEnt->shape = shape;
	rEnt->texture = texture;
	vfBound* entBounds = rEnt->bounds;
	entBounds->entity = rEnt;
	rEnt->collisionCallback = NULL;

	releaseMutex( );
	return rEnt;
}

/* STRUCT DESTRUCTION FUNCTIONS */

VFAPI void vfDestroyTransform(vfTransform* transform, int zero)
{
	/* get index and update buffer field */
	int index = transform - _tBuffer;
	_tBufferField[index] = 0;
	_tCount--;

	/* zero memory */
	if (zero)
	{
		ZeroMemory(transform, sizeof(vfTransform));
	}
}

VFAPI void vfDestroyBound(vfBound* bound, int zero)
{
	/* get index and update buffer field */
	int index = bound - _bBuffer;
	_bBufferField[index] = 0;
	_bCount--;

	/* zero memory */
	if (zero)
	{
		ZeroMemory(bound, sizeof(vfBound));
	}
}

VFAPI void vfDestroyParticle(vfParticle* particle, int zero)
{
	captureMutex("Particle Destruction Timeout");

	/* get index and update buffer field */
	int index = particle - _pBuffer;
	_pBufferField[index] = 0;
	_pCount--;
	
	/* zero memory */
	if (zero)
	{
		ZeroMemory(particle, sizeof(vfParticle));
	}

	releaseMutex();
}

VFAPI void vfDestroyEntity(vfEntity* entity, int zero)
{
	captureMutex("Entity destruction timeout");

	/* get index and update buffer field */
	int index = entity - _eBuffer;
	_eBufferField[index] = 0;
	_eCount--;

	/* destroy sub components */
	vfDestroyTransform(entity->transform, zero);
	vfDestroyBound(entity->bounds, zero);

	/* zero memory */
	if (zero)
	{
		ZeroMemory(entity, sizeof(vfEntity));
	}

	releaseMutex();
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

VFAPI vfHandle vfGetEntityHandle(vfEntity* entity)
{
	return entity - _eBuffer;
}

VFAPI void* vfGetObject(vfHandle handle, int type)
{
	/* check for invalid handle */
	if (handle > VF_BUFFER_SIZE) return NULL;

	/* retu based on type */
	switch (type)
	{
	case VF_OBJ_TRANSFORM:
		return _tBuffer + handle;
		break;

	case VF_OBJ_BOUND:
		return _bBuffer + handle;
		break;

	case VF_OBJ_PARTICLE:
		return _pBuffer + handle;
		break;

	case VF_OBJ_ENTITY:
		return _eBuffer + handle;
		break;

	default:
		return NULL;
		break;
	}

	/* bad state */
	return NULL;
}

/* RENDERING FUNCTIONS */

VFAPI void vfRenderParticles(void)
{
	/* check for render skip */
	if (vgGetRenderSkipState()) return;

	/* get render permission */
	int mResult = WaitForSingleObject(_drawMutex, VF_RMUTEX_TIMEOUT);
	if (mResult != WAIT_OBJECT_0) showMutexError("DrawMutex",
		"Particle render timout");

	int rendered = 0;

	for (int i = 0; i < VF_BUFFER_SIZE; i++)
	{
		if (rendered >= _pCount) break;
		if (!_pBufferField[i]) continue;

		vfParticle render = _pBuffer[i];
		if (!render.active) continue;

		/* get FINAL transform */
		vfTransform* rTransform = render.transform;

		/* null check */
		if (rTransform == NULL) continue;

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

		/* increment render count */
		rendered++;
	}

	/* release mutex */
	ReleaseMutex(_drawMutex);
}

VFAPI void vfRenderEntities(void)
{
	/* check for render skip */
	if (vgGetRenderSkipState()) return;

	/* get render permission */
	int mResult = WaitForSingleObject(_drawMutex, VF_RMUTEX_TIMEOUT);
	if (mResult != WAIT_OBJECT_0) showMutexError("DrawMutex",
		"Entity render timeout");

	int eRendered = 0;

	for (int i = 0; i < VF_BUFFER_SIZE; i++)
	{
		if (eRendered >= _eCount) break;
		if (_eBufferField[i] == 0) continue;

		vfEntity renderEnt = _eBuffer[i];
		if (!renderEnt.active) continue;

		/* grab the current transform of the entity */
		vfTransform* tTemp = renderEnt.transform;

		/* grab the final transform of the entity */
		int tIndex = tTemp - _tBuffer;
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

		/* increment render count */
		eRendered++;
	}

	/* release mutex */
	ReleaseMutex(_drawMutex);
}

VFAPI void vfRenderBounds(void)
{
	/* check for framskip */
	if (vgGetRenderSkipState()) return;

	/* render bound */
	vgLineSize(2.5f);
	int bRendered = 0;

	for (int i = 0; i < VF_BUFFER_SIZE; i++)
	{
		if (bRendered >= _bCount) break;
		if (!_bBufferField[i]) continue;

		boundQuad bQ = _bqBuffer[i];
		
		/* change color based on collision properties */
		if (bQ.collisions > 0)
		{
			vgColor3(255, 128, 64);
		}
		else
		{
			vgColor3(64, 200, 64);
		}

		/* purple for no entity, blue for static/inactive */
		if (bQ.staticData.entity == VF_NOENTITY)
		{
			vgColor3(180, 64, 200);
		}
		else if (!bQ.staticData.entity->physics.active ||
			!bQ.staticData.entity->physics.moveable)
		{
			vgColor3(64, 180, 200);
		}

		vgRenderLayer(128);

		vgPointf(bQ.average.x, bQ.average.y);
		for (int j = 0; j < 4; j++)
		{
			int iNext = (j + 1) % 4;
			vgLinef(bQ.verts[j].x, bQ.verts[j].y, bQ.verts[iNext].x,
				bQ.verts[iNext].y);
		}

		vgColor3(255, 0, 64);
		for (int j = 0; j < bQ.collisions; j++)
		{
			vfVector colDataVis = VECT(bQ.average.x + bQ.collisionData[j].x,
				bQ.average.y + bQ.collisionData[j].y);
			vgLinef(bQ.average.x, bQ.average.y, colDataVis.x, colDataVis.y);
		}
		
		vgRenderLayer(0);

		/* increment render count */
		bRendered++;
	}
	vgLineSize(1.0f);
}

VFAPI void vfRenderPartitions(void)
{
	/* check for framskip */
	if (vgGetRenderSkipState()) return;

	/* set render layer*/
	vgRenderLayer(0x10);

	/* render lines */
	vgLineSize(1.5f);
	/* render x and y axis */
	vgColor3(0xFF, 0x20, 0x20); vgLine(INT_MAX, 0, INT_MIN, 0);
	vgColor3(0x20, 0x20, 0xFF); vgLine(0, INT_MAX, 0, INT_MIN, 0);
	/* render all partition boundaries */
	vgColor3(0x20, 0xFF, 0x20);
	for (int i = 0; i < _partitionCount; i++)
	{
		/* get partition bounding box */
		partition renderPart = _partBuff[i];
		int pMinX = renderPart.x * _partitionSize;
		int pMaxX = pMinX + _partitionSize;
		int pMinY = renderPart.y * _partitionSize;
		int pMaxY = pMinY + _partitionSize;

		vgLine(pMinX, pMinY, pMinX, pMaxY); /* left   */
		vgLine(pMinX, pMaxY, pMaxX, pMaxY); /* top    */
		vgLine(pMaxX, pMinY, pMaxX, pMaxY); /* right  */
		vgLine(pMinX, pMinY, pMaxX, pMinY); /* bottom */
	}
	vgLineSize(1);

	/* render all partitions */
	for (int i = 0; i < _partitionCount; i++)
	{
		partition renderPart = _partBuff[i];

		/* if partition only has one or less, skip rect */
		if (renderPart.bqCount <= 1) continue;

		/* draw rect */
		vgColor4(min(renderPart.bqCount * 0x40, 0xFF), 0x80, 0x80, 0x40);
		vgRect(renderPart.x * _partitionSize,
			   renderPart.y * _partitionSize,
			_partitionSize,
			_partitionSize);
	}
}

/* PHYSICS RELATED FUNCTIONS */
VFAPI void vfSetPhysicsState(int value)
{
	_pEnabled = value;
}

VFAPI void vfSetCollisionCallback(vfEntity* entity, ENTCOLCALLBACK callback)
{
	entity->collisionCallback = callback;
}

VFAPI void vfSetUpdateCallback(vfEntity* entity, ENTUPDCALLBACK callback)
{
	entity->updateCallback = callback;
}

VFAPI int vfSetUpdateCallbackStatic(STATUPDCALLBACK callback,
	int priorityRequest)
{
	/* check for callback full */
	if (_sCallSize == VF_STATICCALLBACK_MAX) return -1;

	/* check if callback priority is free */
	if (_sCallBuff[priorityRequest] == NULL)
	{
		_sCallBuff[priorityRequest] = callback;
		return priorityRequest;
	}

	/* if reached here, priority is not free */
	/* look for free spot */
	for (int i = priorityRequest; i < VF_STATICCALLBACK_MAX; i++)
	{
		if (_sCallBuff[i] == NULL)
		{
			_sCallBuff[i] = callback;
			return i;
		}
	}

	/* if reached here, no free spot :( */
	return -1;
}

VFAPI void vfSetPartitionSize(int size)
{
	_partitionSize = size;
}

VFAPI ULONGLONG vfGetPhysicsUpdateTime(void)
{
	return _updateTime;
}

/* DATA RELATED FUNCTIONS */
VFAPI int vfGetBuffer(void* buffer, int size, int type)
{
	/* bad size check */
	if (size < 0) return 0;

	/* select buffer to read from */
	unsigned char* pbuff;
	switch (type)
	{
	case VF_BUFF_TRANSFORM:
		pbuff = (BYTE*)_tBuffer;
		break;

	case VF_BUFF_BOUND:
		pbuff = (BYTE*)_bBuffer;
		break;

	case VF_BUFF_PARTICLE:
		pbuff = (BYTE*)_pBuffer;
		break;

	case VF_BUFF_ENTITY:
		pbuff = (BYTE*)_eBuffer;
		break;
	
	/* fail condition */
	default:
		return 0;
		break;
	}

	/* write to user buffer */
	memcpy(buffer, pbuff, size);
	return 1;
}

VFAPI int vfGetBufferField(void* field, int size, int type)
{
	/* bad size check */
	if (size < 0) return 0;

	/* select buffer to read from */
	unsigned char* pbuff;
	switch (type)
	{
	case VF_BUFF_TRANSFORM:
		pbuff = _tBufferField;
		break;

	case VF_BUFF_BOUND:
		pbuff = _bBufferField;
		break;

	case VF_BUFF_PARTICLE:
		pbuff = _pBufferField;
		break;

	case VF_BUFF_ENTITY:
		pbuff = _eBufferField;
		break;

		/* fail condition */
	default:
		return 0;
		break;
	}

	/* write to user buffer */
	memcpy(field, pbuff, size);
	return 1;
}

VFAPI int vfGetObjectCount(int type)
{
	int count = -1;

	switch (type)
	{
	case VF_OBJ_TRANSFORM:
		count = _tCount;
		break;

	case VF_OBJ_BOUND:
		count = _bCount;
		break;

	case VF_OBJ_PARTICLE:
		count = _pCount;
		break;

	case VF_OBJ_ENTITY:
		count = _eCount;
		break;

	default:
		break;
	}

	return count;
}

/* ALLOCATING AND FREEING FUNCTION */
static int ensureFreeMTBlock(int startIndex, int size)
{
	for (int i = 0; i < size; i++)
	{
		/* check for contig, if !, ret 0 */
		if (_mTField[startIndex + i] == 1) return 0;
	}
	return 1;
}

static int findFreeMTIndex(int size)
{
	for (int i = 0; i < VF_MEMTANK_FIELDSIZE; i++)
	{
		/* on find free spot, ret index */
		if (_mTField[i] == 0 && ensureFreeMTBlock(i, size))
			return i;
	}

	/* on fail, ret -1 */
	return -1;
}

VFAPI void* vfMTAlloc(int size, int zero)
{
	/* bad size condition */
	if (size < 0 || size / VF_MEMTANK_INTERVAL > VF_BUFFER_SIZE) 
		return NULL;

	/* find lowest multiple of interval which satisfies size */
	float fsize = (float)size;
	int sizeActual = (int)ceilf(fsize / VF_MEMTANK_INTERVAL);

	/* check for the impossible */
	if (sizeActual * VF_MEMTANK_INTERVAL < size) sizeActual++;

	/* find index */
	int index = findFreeMTIndex(sizeActual);

	/* fail condition */
	if (index == -1) return NULL;

	/* fill field */
	for (int i = 0; i < sizeActual; i++)
		_mTField[index + i] = 1;

	/* check zero and return ptr */
	if (zero) ZeroMemory(_mTank + index, 
		sizeActual * VF_MEMTANK_INTERVAL);
	return _mTank + index;
}

VFAPI int vfMTFree(void* ptr, int size, int zero)
{
	/* cast ptr */
	BYTE* cPtr = ptr;

	/* check for bad ptr */
	if (cPtr == NULL) return 0;

	/* check more for bad ptr */
	if (cPtr < _mTank || cPtr > _mTank + VF_MEMTANK_SIZE) return 0;

	/* find lowest multiple of 8 which satisfies size */
	float fsize = (float)size;
	int sizeActual = (int)ceilf(fsize / VF_MEMTANK_INTERVAL);

	/* check for the impossible */
	if (sizeActual * VF_MEMTANK_INTERVAL < size) sizeActual++;

	/* free field */
	int startIndex = ((char*)ptr - _mTank) / VF_MEMTANK_INTERVAL;
	for (int i = 0; i < sizeActual; i++)
		_mTField[startIndex + i] = 0;

	/* zero */
	if (zero) ZeroMemory(ptr, sizeActual * VF_MEMTANK_INTERVAL);

	return 1;
}