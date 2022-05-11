/******************************************************************************
* <framework.h>
* Bailey Jia-Tao Brown
* 2021/2022
*
*	Source file for abstract graphics and utilites library
*	Contents:
*		- Preprocessor definitions
*		- Includes
*		- Internal definitions
*		- Internal resources
*		- Internal physics functions
*		- Internal particle functions
*		- Internal attribute functions
*		- Internal projectile functions
*		- Internal explosion functions
*		- Init and terminate functions
*		- Struct creation functions
*		- Struct destruction functions
*		- Struct related functions
*		- Rendering functions
*		- Physics functions
*		- Particle functions
*		- Attribute functions
*		- Projectile functions
*		- Explosion functions
*		- Data functions
*
******************************************************************************/

#define _CRT_SECURE_NO_WARNINGS
#define _WIN32_LEAN_AND_MEAN 

#include <stdio.h>  /* For file I/O */
#include <malloc.h> /* For memory management */
#include <string.h> /* For strlen function */
#include <math.h>   /* For trig functions */

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
static int _pUpdateTime = 0; /* physics update time taken */
static int _pCollisionCheckCount = 0; /* amount of col checks */
static int _pPartitionCheckCount = 0; /* amount of part jmps */
static int _pCollisionCheckTime  = 0;

static int _killSignal;   /* thread kill signal */
static int _killRecieved; /* kill recieved signal */

/* internal buffers */

/* TRANSFORM RELATED DATA */
static vfTransform* _tBuffer; static field* _tBufferField;
static vfTransform* _tFinalBuffer;
static int _tCount;

/* PARTICLE RELATED DATA */
static vfParticle* _pBuffer; static field* _pBufferField;
static int _pCount;
static vfParticleBehavior* _pbBuffer;
static int _pbCount;

/* BOUND RELATED DATA */
static vfBound* _bBuffer; static field* _bBufferField;
static int _bCount;

/* ENTITY RELATED DATA */
static vfEntity* _eBuffer; static field* _eBufferField;
static int _eCount;

/* STATIC CALLBACK DATA */
static STATUPDCALLBACK _sCallBuff[VF_STATICCALLBACK_MAX];
static int _sCallSize;
static vfTickCount _tickCount = 0;

/* ATTRIBUTE TABLE DATA */
static vfAttributeTable _atBuffer[VF_ATTRIBTABLES_MAX];
static int _atCount;

/* PROJECTILE DATA */
static vfProjectileBehavior _projBBuffer[VF_PROJBEHAVIORS_MAX];
static int _projBCount;
static vfProjectile* _projBuffer; static field* _projBufferField;
static int _projAllocated;
static int _projCount;

/* EXPLOSION BUFFERS */
static vfExplosionBehavior _expBBuffer[VF_EXPBEHAVIORS_MAX];
static int _expBCount;
static vfExplosion* _expBuffer;
static field* _expBufferField;
static int _expCount;
static int* _expPushBuffer;

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

/* boundquad buffer, this buffer maps ever Bound object to a quad, which */
/* is the Bound object's dimensions and offset translated by it's transform */
static boundQuad* _bqBuffer;

/* projVect struct definition */
typedef struct projVect 
{
	vfVector p1;
	vfVector p2;
	vfVector vector;
	float mag;
} projVect;

/* ========== INTERNAL ALLOC FUNCTIONS ======== */
static void* vAlloc(size_t allocSize, int zero)
{
	/* setup allocation flags */
	int allocFlags = 0; if (zero) allocFlags = HEAP_ZERO_MEMORY;
	void* checkAlloc = HeapAlloc(_heap, allocFlags,
		allocSize);

	/* if null, report err and exit */
	if (checkAlloc == NULL)
	{
		MessageBoxA(NULL, "Failed to allocate more memory!",
			"CRITICAL FAILURE!", MB_OK);
		exit(0);
	}

	/* return ptr*/
	return checkAlloc;
}

static void vFree(void* toFree)
{
	BOOL result = HeapFree(_heap, 0, toFree);
	if (!result)
	{
		MessageBoxA(NULL, "Failed to free memory!\n",
			"CRITICAL FAILURE!", MB_OK);
		return;
	}
}

/* ========== MUTEX RELATED FUNCTIONS ========== */
static void showMutexError(const char* mName, const char* description)
{
	char cBuff[0xFF] = { 0 };
	sprintf(cBuff, "Failed to capture mutex!\n"
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

/* SPACE PARTITION BUFFERS */
typedef struct partition
{
	UINT16  overfitAge; /* time with excess memory allocated */
	INT16   x; /* partition x */
	INT16   y; /* partition y */
	UINT16  bqCount; /* boundquad count */
	UINT16  bqBuffSize; /* allocated buffer size */
	UINT16* bqIndexes; /* dynamic array */
	float   velSum; /* sum of object velocities */
} partition;

static partition* _partBuff;
static int _partitionCount = 0;
static int _partitionsAllocated = 0;
static int _partitionSize = 0;
static int _partitionsExtraRequested = 0;
static int _partitionsMax = 0;

/* GET BQ VEL MAG (gets speed heuristic for a given bq) */
static float getBQVelMag(boundQuad* bq, int minageCheck)
{
	if (bq->staticData.entity == NULL) return 0;

	float velX = fabsf(bq->staticData.entity->physics.velocity.x);
	float velY = fabsf(bq->staticData.entity->physics.velocity.y);
	float velT = fabsf(bq->staticData.entity->physics.tourque);

	/* if age is young, return positive num */
	if (bq->staticData.entity->physics.age < VF_PART_SKIP_MINAGE
		&& minageCheck)
	{
		return 0xFF;
	}

	/* else, return standard value*/
	return (velX + velY + velT) * VF_PART_SKIP_DAMPENER;
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
			(centerOffsetX * VF_PART_OVERLAPSCALE);
		float posYScaled = bq->average.y +
			(centerOffsetY * VF_PART_OVERLAPSCALE);

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

/* ENSURE PARTITION SIZE */
static inline void ensurePartitionSize(partition* part)
{
	/* if overfitage is too great, de-alloc some memory */
	if (part->overfitAge > VF_PART_OVERAGE_TIME)
	{
		/* decrement and free */
		part->bqBuffSize -= VF_PART_STEP;
		vFree(part->bqIndexes);

		/* if size is 0, set buffer to null */
		if (part->bqBuffSize == 0)
		{
			part->bqIndexes  = NULL;
			part->overfitAge = 0;
		}
		else /* else, realloc */
		{
			/* realloc */
			part->bqIndexes = vAlloc(part->bqBuffSize * sizeof(UINT16),
				FALSE);

			/* reset overfit age */
			part->overfitAge = 0;
		}
	} /* overfit check exit */

	/* if size if ok, return */
	if (part->bqBuffSize > part->bqCount) return;

	/* if index buffer is null */
	if (part->bqIndexes == NULL)
	{
		/* increment and alloc */
		part->bqBuffSize += VF_PART_STEP;
		part->bqIndexes = vAlloc(part->bqBuffSize * 
			sizeof(UINT16), FALSE);
	}

	/* if size is not big enough */
	if (part->bqBuffSize <= part->bqCount)
	{
		/* increment step and alloc new */
		int oldSize = part->bqBuffSize;
		part->bqBuffSize += VF_PART_STEP;
		void* temp = vAlloc(part->bqBuffSize * sizeof(UINT16), FALSE);

		/* copy data and free old pointer */
		memcpy(temp, part->bqIndexes, oldSize * sizeof(UINT16));
		vFree(part->bqIndexes);

		/* reassign new ptr */
		part->bqIndexes = temp;
	}
}

/* ADD TO PARTITION */
static inline void addToPartition(boundQuad* bq, int x, int y)
{
	/* check for existing partition */
	for (int i = 0; i < _partitionCount; i++)
	{
		partition* part = _partBuff + i;
		/* skip if not collide */
		if (part->x != x || part->y != y) continue;

		/* ensure partition capacity */
		ensurePartitionSize(part);

		/* add bqindex to partition */
		int bqIndex = bq - _bqBuffer;
		part->bqIndexes[part->bqCount] = bqIndex;

		/* add velocity sum to it */
		part->velSum += getBQVelMag(bq, TRUE);

		/* return, since it's done */
		part->bqCount++;
		return;
	}

	/* on no partitions left, request more partitions and return */
	if (_partitionCount == _partitionsAllocated)
	{
		_partitionsExtraRequested++;
		return;
	}

	/* on exit loop without adding to partition, create new partition */
	partition* part = _partBuff + _partitionCount; /* gets next free part */
	part->x = x; part->y = y;
	_partitionCount++;

	/* ensure partition size */
	ensurePartitionSize(part);

	/* add next boundquad to partition */
	int bqIndex = bq - _bqBuffer;
	part->bqIndexes[part->bqCount] = bqIndex;
	part->bqCount++;

	/* add velocity sum to it */
	part->velSum += getBQVelMag(bq, TRUE);
}

/* ASSIGN PARTITION */
static void assignPartition(boundQuad* bq)
{
	/* buffers for later */
	int pCountX = 0;
	int pCountY = 0;
	int pBuffX[VF_BOUND_PART_MAX];
	int pBuffY[VF_BOUND_PART_MAX];

	/* check which part the bq is part of */
	partCheck(bq, VF_BOUND_PART_MAX, &pCountX, &pCountY, pBuffX,
		pBuffY);

	/* add bq to all given parts */
	for (int rows = 0; rows < pCountX; rows++)
		for (int cols = 0; cols < pCountY; cols++)
			addToPartition(bq, pBuffX[rows], pBuffY[cols]);
}

/*==================== PHYSICS FUNCTIONS ====================*/

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
		float sourceMass = 0; vfPhysics* sourcePhys = NULL;
		float targetMass = 0; vfPhysics* targetPhys = NULL;
		float massTotal = 0; float massPercent = 0;

		/* if source has no entity, ratio is 100% */
		if (source->staticData.entity == NULL)
		{
			/* instant jalf displacement */
			target->staticData.entity->transform->position.x
				-= smallestPVect.x / 2.0f;
			target->staticData.entity->transform->position.y
				-= smallestPVect.y / 2.0f;

			/* set ratio to 1 */
			massPercent = 1.0f;
		}
		else
		{
			/* if source has entity, weight mass properly */
			sourcePhys = &(target->staticData.entity->physics);
			targetPhys = &(source->staticData.entity->physics);
			massTotal = targetPhys->mass + sourcePhys->mass;

			/* percentage to apply to target */
			massPercent = 1.0f - (sourcePhys->mass / massTotal);

			/* if current is immovable or physics is inactive */
			/* set massPercent to 1.0 */
			if (!sourcePhys->moveable ||
				!sourcePhys->active)
			{
				massPercent = 1.0f;
			}
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
	/* check if extra partitions have been requested */
	/* or if partition limit will soon be reached */
	if (_partitionsExtraRequested || 
		_partitionCount > _partitionsAllocated - VF_PART_INCREASE_THRESOLD)
	{
		/* get highest multiple to allocate extra */
		int allocCount = (int)ceilf((float)(_partitionsExtraRequested)
			/ (float)(VF_PART_COUNT_INCREMENT));

		/* increment alloc count */
		int oldSize = _partitionsAllocated * sizeof(partition);
		_partitionsAllocated += (allocCount * VF_PART_COUNT_INCREMENT);

		/* if max part size reached, report error */
		if (_partitionsAllocated >= _partitionsMax)
		{
			MessageBoxA(NULL, "Maximum amount of partitions allocated!",
				"CRITICAL ENGINE FAILURE!", MB_OK);
			exit(1);
		}

		captureMutex("Reallocating Partition Buffer");

		/* free and reallocate */
		void* temp = vAlloc(_partitionsAllocated * sizeof(partition),
			TRUE);
		memcpy(temp, _partBuff, oldSize);
		vFree(_partBuff);
		_partBuff = temp;

		/* clear extra requested */
		_partitionsExtraRequested = 0;

		releaseMutex();
	}

	/* check for excess partitions to free */
	if (_partitionCount < _partitionsAllocated - VF_PART_COUNT_INCREMENT
		- VF_PART_DECREASE_THRESOLD)
	{
		/* get oldsize and decrease allocation size */
		int oldSize = _partitionsAllocated * sizeof(partition);
		_partitionsAllocated -= VF_PART_COUNT_INCREMENT;

		captureMutex("Reallocating Partition Buffer");

		/* alloc new and copy over memory */
		void* temp = vAlloc(_partitionsAllocated *
			sizeof(partition), FALSE);
		memcpy(temp, _partBuff, _partitionsAllocated * sizeof(partition));

		/* free excess memory */
		for (int i = 0; i < VF_PART_COUNT_INCREMENT; i++)
		{
			int index = _partitionsAllocated + i;
			if (_partBuff[index].bqIndexes != NULL &&
				_partBuff[index].bqBuffSize != 0)
			{
				vFree(_partBuff[index].bqIndexes);
			}
		}

		/* free and reassign */
		vFree(_partBuff);
		_partBuff = temp;

		releaseMutex();
	}

	/* first, clear partition buffer data */
	_partitionCount = 0;
	for (int i = 0; i < _partitionsAllocated; i++)
	{
		/* clear partition members */
		_partBuff[i].velSum  = 0;
		_partBuff[i].bqCount = 0;
		for (int j = 0; j < _partBuff[i].bqBuffSize; j++)
		{
			_partBuff[i].bqIndexes[j] = 0xFFFF;
		}
	}

	/* update all partitions */
	for (int i = 0; i < VF_BUFFER_SIZE; i++)
	{
		/* if bound is not used, skip */
		if (!_bBufferField[i]) continue;

		/* if bounds is not active, skip */
		if (!_bqBuffer[i].staticData.active) continue;

		/* assign bounds to a partition */
		assignPartition(_bqBuffer + i);
	}

	/* check all partitions for overfit, and then update values */
	for (int i = 0; i < _partitionCount; i++)
	{
		/* get partition, if partition allocated memory is */
		/* greater than 1 step above memory needed + extra, */
		/* then mark partition by incrementing overfitage */
		partition* checkPart = _partBuff + i;
		if (checkPart->bqBuffSize > 
			checkPart->bqCount + VF_PART_STEP)
		{
			/* when overfitage is checked to be over a certain thres, */
			/* memory will be freed up. */
			checkPart->overfitAge++;
		}
	}

	/* loop all partitions */
	int colCheckCounter = 0;
	int partCheckCounter = 0;
	
	for (int partIndex = 0; partIndex < _partitionCount; partIndex++)
	{
		/* get current physics partition */
		partition currentPart = _partBuff[partIndex];

		/* if partition velocity is 0, skip */
		if (floorf(currentPart.velSum) == 0) continue;

		/* if partition has size 1 or less, skip */
		if (currentPart.bqCount <= 1) continue;

		/* if reached here, will check partition */
		partCheckCounter++;

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

				/* ensuer bounds have entity attatched */
				if (sourcePtr->staticData.entity != NULL &&
					targetPtr->staticData.entity != NULL)
				{
					if (floorf(getBQVelMag(sourcePtr, TRUE)) == 0 &&
						floorf(getBQVelMag(targetPtr, TRUE)) == 0   )
							continue;
				}

				/* perform collision check */
				collisionCheck(sourcePtr, targetPtr);
				colCheckCounter++;
			}
		}
	}

	/* reassign col/part check amount */
	_pCollisionCheckCount = colCheckCounter;
	_pPartitionCheckCount = partCheckCounter;

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

/* ==== UPDATE AGE FUNCTION ==== */
static inline void updatePhyiscsAges(void)
{
	int updateCount = 0;
	for (int i = 0; i < VF_BUFFER_SIZE; i++)
	{
		/* get entity */
		vfEntity* ent = _eBuffer + i;

		/* if checked all entities, break */
		if (updateCount >= _eCount) break;

		/* if entity doesn't exist, continue */
		if (!_eBufferField[i]) continue;

		/* if entity is inactive, continue */
		if (!ent->active) { updateCount++; continue; };

		/* if ent physics inactive, continue */
		if (!ent->physics.active) { updateCount++; continue; };

		/* update phyiscs age */
		ent->physics.age++;
		updateCount++;
	}
}

/* ============= INTERNAL PARTICLE UPDATE FUNCTIONS ============ */
static inline vfVector vectorAdd(vfVector vect1, vfVector vect2)
{
	return VECT(vect1.x + vect2.x, vect1.y + vect2.y);
}

static void updateParticles(void)
{
	/* loop all particles */
	int updateCount = 0;
	for (int i = 0; i < VF_BUFFER_SIZE; i++)
	{
		/* if checked all, break */
		if (updateCount >= _pCount) break;

		/* continue if particle is not used */
		if (!_pBufferField[i]) continue;

		/* get particle */
		vfParticle* partRef = _pBuffer + i;

		/* check if particle is too old */
		/* OR if particle is unseeable  */
		if (partRef->lifeAge > partRef->lifeTime ||
			!vgCheckIfViewable(partRef->transform.position.x,
				partRef->transform.position.y, VF_PARTICLE_DESTROYEXTRA))
		{
			/* clear buffer and zero memory */
			_pBufferField[i] = 0;
			ZeroMemory(partRef, sizeof(vfParticle));
			
			/* decrement count and continue */
			_pCount--;
			continue;
		}

		/* call behavior callbacks (if exists) */
		if (partRef->behavior.updateBehavior != NULL &&
			partRef->behavior.updateBehavior != VF_PB_ERROR)
			partRef->behavior.updateBehavior(&(partRef->behavior),
				partRef->lifeAge);

		/* update all values */
		partRef->transform.position = vectorAdd(partRef->transform.position,
			partRef->behavior.velocity);
		partRef->transform.rotation += partRef->behavior.torque;
		partRef->transform.scale += partRef->behavior.sizeChange;
		
		/* update filter values and clamp */
		vfColor cInitial = partRef->filter;
		vfColor cChange = partRef->behavior.filterChange;
		int nextR = max(0, min(255, cInitial.r + cChange.r));
		int nextG = max(0, min(255, cInitial.g + cChange.g));
		int nextB = max(0, min(255, cInitial.b + cChange.b));
		int nextA = max(0, min(255, cInitial.a + cChange.a));
		partRef->filter = COLORA(nextR, nextG, nextB, nextA);

		/* increase particle age */
		partRef->lifeAge++;

		/* increment update count */
		updateCount++;
	}
}

/* ========== INTERNAL ATTRIBUTE RELATED FUNCTIONS ========== */
static uint32_t generateStringHash(const char* string)
{
	uint32_t hash = 0;
	int counter = 0;
	while (TRUE)
	{
		/* check for null terminate */
		if (!string[counter]) break;

		/* build unique string hash */
		hash += string[counter] * (counter + 1);
		counter++;
	}
	return hash;
}

static void* getAttributeInternal(vfEntity* target, const char* attribName)
{
	/* get name hash */
	uint32_t nameHash = generateStringHash(attribName);

	/* get entity attribute layout */
	vfAttributeTable table = _atBuffer[target->attribHandle];

	/* ensure attribute exists */
	for (int i = 0; i < table.attribCount; i++)
	{
		/* on exists */
		if (nameHash == table.attribNameHash[i])
		{
			/* get byte offset and return entity memory block + offset */
			int byteOffset = table.attribByteOffset[i];
			return target->attribBlock + byteOffset;
		}
	}

	/* if loop broken, fail*/
	return NULL;
}

/* ========== INTERNAL PROJECTILE FUNCTIONS ========== */

static inline void ensureProjectileBufferSize(void)
{
	/* check if projectile buffer must be increased */
	if (_projCount > _projAllocated - VF_PROJCOUNT_CHANGETHRES)
	{
		/* get write mutex */
		int result = WaitForSingleObject(_writeMutex, VF_WMUTEX_TIMEOUT);
		if (result != WAIT_OBJECT_0) showMutexError("Write Mutex",
			"Projectile Creation");

		/* increment projbuffer size */
		int oldSize = _projAllocated;
		_projAllocated += VF_PROJCOUNT_STEP;

		/* reallocate projectile buffer */
		void* temp = vAlloc(sizeof(vfProjectile) * _projAllocated, FALSE);
		memcpy(temp, _projBuffer, sizeof(vfProjectile) * oldSize);
		vFree(_projBuffer);
		_projBuffer = temp;

		/* reallocate projectile buffer field */
		temp = vAlloc(sizeof(field) * _projAllocated, TRUE);
		memcpy(temp, _projBufferField, sizeof(field) * oldSize);
		vFree(_projBufferField);
		_projBufferField = temp;

		/* release write mutex */
		ReleaseMutex(_writeMutex);
	}

	/* check if projectile buffer must be decreased */
	if (_projCount < _projAllocated - VF_PROJCOUNT_CHANGETHRES
		- VF_PROJCOUNT_STEP && _projCount >= VF_PROJCOUNT_DEFAULT)
	{
		/* get write mutex */
		int result = WaitForSingleObject(_writeMutex, VF_WMUTEX_TIMEOUT);
		if (result != WAIT_OBJECT_0) showMutexError("Write Mutex",
			"Projectile Creation");

		/* decrement projbuffer size */
		_projAllocated -= VF_PROJCOUNT_STEP;

		/* reallocate projectile buffer */
		void* temp = vAlloc(sizeof(vfProjectile) * _projAllocated, FALSE);
		memcpy(temp, _projBuffer, sizeof(vfProjectile) * _projAllocated);
		vFree(_projBuffer);
		_projBuffer = temp;

		/* reallocate projectile buffer field */
		temp = vAlloc(sizeof(field) * _projAllocated, FALSE);
		memcpy(temp, _projBufferField, sizeof(field) * _projAllocated);
		vFree(_projBufferField);
		_projBufferField = temp;

		/* release write mutex */
		ReleaseMutex(_writeMutex);
	}
}

static int __randTable[] = /* random table */
{
	48, 00, 22, 31, 63, 57, 49, 91,
	25, 89, 19, 49, 75, 68, 24, 86,
	53, 84, 41, 23, 76, 35, 89, 60,
	86, 73, 98, 48, 82, 28, 57, 30,
	68, 48, 78, 21, 81, 83, 56, 78,
	27, 67,  9, 27, 43, 60, 84, 80,
	33, 43, 88, 61, 61, 52, 40, 54,
	70, 93, 91, 27, 99, 96, 44, 86,
	55, 82, 25, 05, 68, 57, 36, 24,
	37, 85, 68, 44, 64, 93,  8, 36,
	51, 90, 04, 48, 84,  9, 44, 95,
	96, 18, 68, 11, 99, 80,  8, 21,
	06, 48, 86, 67, 89, 38, 31, 41,
	13, 24, 35, 43, 59, 64, 21, 03,
	16, 74, 54, 95, 26, 43, 58, 02,
	04, 89, 15, 48, 34, 40,  9, 97,
	60, 57, 34, 41, 35, 76, 59, 03,
	34, 93, 90, 31, 82, 56, 66, 33,
	17, 06, 72, 03, 73, 80, 71, 18,
	46, 39, 95, 55, 74, 71, 32, 89,
	43, 26, 30, 17, 26, 67, 79, 51,
	18, 94, 96, 46, 71, 32, 25, 74,
	47, 02, 96, 13, 11, 65,  9, 14,
	48, 14, 07, 79, 51, 48, 03, 73,
	93, 88, 49, 16, 27, 11, 77, 17,
	27, 07, 90, 45, 56, 78, 99, 05,
	54, 01, 79, 39, 39, 15, 35, 11,
	02, 34, 57, 25, 23, 83, 67, 72,
	76, 23, 91, 19, 75, 73, 17, 69,
	05, 84, 17, 13, 35,  8, 79, 50,
	46, 85, 87, 54, 21, 73, 81, 87,
	48, 00, 01, 01, 21, 68, 73, 02
};

static inline int seededRandomUINT(int seed)
{
	return __randTable[seed % (sizeof(__randTable) /
		sizeof(__randTable[0]))];
}

static inline int seededRandomINT(int seed)
{
	int randVal = seededRandomUINT(seed);
	randVal -= 50;
	randVal *= 2;
	return randVal;
}

static inline float seededRandomNORMALIZED(int seed)
{
	float percent = (float)seededRandomINT(seed);
	return percent /= 100.0f;
}

static inline float seededRandomFLOAT(int seed, float clamp)
{
	return seededRandomNORMALIZED(seed) * clamp;
}

static int __randSeed = 0;
static inline float randomFLOAT(float clamp)
{
	return seededRandomFLOAT(__randSeed++, clamp);
}

static inline vfVector cartesianToPolar(vfVector src)
{
	vfVector polar;
	polar.x = hypotf(src.x, src.y);
	polar.y = atan2f(src.y, src.x);
	return polar;
}

static inline vfVector polarToCartesian(float r, float theta)
{
	vfVector cart;
	cart.x = r * cosf(theta);
	cart.y = r * sinf(theta);
	return cart;
}

/* PROJECTILE UPDATE FUNCTION AND HELPERS */

/* get projectile partition. if none, returns NULL */
static partition* getProjectilePartiton(vfProjectile* proj)
{
	/* get projectile partition */
	int partX; int partY;
	if (proj->position.x > 0)
		partX = proj->position.x / (float)_partitionSize;
	else
		partX = floorf(proj->position.x / (float)_partitionSize);

	if (proj->position.y > 0)
		partY = proj->position.y / (float)_partitionSize;
	else
		partY = floorf(proj->position.y / (float)_partitionSize);

	/* get partition (if exists) */
	partition* colPart = NULL;
	for (int j = 0; j < _partitionCount; j++)
	{
		/* if not found, continue */
		if (_partBuff[j].x != partX ||
			_partBuff[j].y != partY) continue;

		/* on found, set and break */
		colPart = _partBuff + j;
		break;
	}

	/* return partition */
	return colPart;
}

/* projectile collision is like "ray marching"           */
/* the projectile will take maxsteps or LESS             */
/* and each step it will check if it has collided with a */
/* bound in it's current partition. if it collides, it   */
/* will return early as collided bound. if step without  */
/* any collisions, will return NULL                      */
static vfBound* checkProjectileCollide(vfProjectile* proj)
{
	/* get projectile behavior object */
	vfProjectileBehavior pb = _projBBuffer[proj->behaviorHandle];

	/* calculate step count */
	int stepCount = 1;
	float stepX = proj->movement.x;
	float stepY = proj->movement.y;
	float moveMax = max(fabsf(proj->movement.x),
		fabsf(proj->movement.y));

	/* get final position */
	vfVector positionFinal = vectorAdd(proj->movement, proj->position);

	/* only do subdivisions if movement is > MAXSTEP*/
	if (moveMax > VF_PROJ_PRECISION * pb.boundScale)
	{
		stepCount = min(moveMax / VF_PROJ_PRECISION, VF_PROJ_MAXSTEPS);

		stepX /= (float)stepCount;
		stepY /= (float)stepCount;
	}

	/* do steps */
	for (int i = 0; i < stepCount; i++)
	{
		/* move projectile forward */
		proj->position = vectorAdd(proj->position, VECT(stepX, stepY));

		/* get projectile partition */
		partition* projPart = getProjectilePartiton(proj);
		if (!projPart) continue; /* on empty, continue */

		/* loop all bounds within projPart */
		for (int j = 0; j < projPart->bqCount; j++)
		{
			/* get bound */
			vfBound* checkBound = _bBuffer + projPart->bqIndexes[j];

			/* skip if inactive or physics is inactive */
			if (!checkBound->active) continue;
			if (checkBound->entity)
				if (!checkBound->entity->active) continue;

			/* if bound's entity is source, skip */
			if (checkBound->entity == proj->source) continue;

			/* if already collided with entity, skip */
			if (checkBound->entity == proj->lastPenetrated &&
				proj->penetrations != 0) continue;

			/* on overlap, set collidedBound and break */
			if (vfCheckPointOverlap(proj->position, checkBound,
				pb.boundScale))
			{
				/* do callback */
				if (pb.collisionCallback)
					pb.collisionCallback(proj, checkBound->entity);
				/* set new final position */
				proj->position = positionFinal;
				return checkBound;
			}
		}
	}

	/* set new final position */
	proj->position = positionFinal;

	/* return collided bound */
	return NULL;
}

/* main projectile update function */
static void updateProjectiles(void)
{
	int counter = 0;

	/* for every projectile */
	for (int i = 0; i < _projAllocated; i++)
	{
		if (counter >= _projBCount) break; /* on checked all, break */

		if (!_projBufferField[i]) continue; /* on empty, continue */

		/* get projectile and behavior */
		vfProjectile* proj = _projBuffer + i;
		vfProjectileBehavior pb = _projBBuffer[proj->behaviorHandle];

		/* check if projectile should be destroyed due to age */
		if (proj->age > pb.maxAge + seededRandomFLOAT(i, pb.maxAgeVariation))
		{
			/* do callback */
			if (pb.maxAgeCallback)
				pb.maxAgeCallback(proj);
			/* set flag and decrement count */
			_projBufferField[i] = 0;
			_projCount--;
		}

		/* increase projectile age */
		proj->age++;

		/* apply drag to projectile */
		float dragVariation = seededRandomFLOAT(_projBuffer - proj,
			pb.dragVariation);
		proj->movement.x *= (1.0f - (1.0f * max(0.0f, pb.drag + dragVariation)));
		proj->movement.y *= (1.0f - (1.0f * max(0.0f, pb.drag + dragVariation)));

		/* if stopped moving, destroy */
		if (pb.destroyOnStopMoving &&
			floorf(fabsf(proj->movement.x)) <= pb.stopMoveThresh &&
			floorf(fabsf(proj->movement.y)) <= pb.stopMoveThresh   )
		{
			/* try callback */
			if (pb.stopMoveCallback)
				pb.stopMoveCallback(proj);

			/* set flag and decrement count */
			_projBufferField[i] = 0;
			_projCount--;
			continue;
		}

		/* get collided bound of projectile */
		vfBound* collisionBound = checkProjectileCollide(proj);
		if (!collisionBound) continue;

		/*if there is no entity, and cannot penetrate* /
		/* non entities, destroy the projectile */
		if (!collisionBound->entity && !pb.nonEntityPenetration)
		{
			_projBufferField[i] = 0;
			_projCount--;
			continue;
		}

		/* calculate if can't penetrate */
		if ((pb.penetrationChance < 1.0f &&
			fabsf(seededRandomNORMALIZED(_tickCount))
		> pb.penetrationChance)
			|| pb.penetrationChance == 0.0f)
		{
			/* destroy projectile */
			_projBufferField[i] = 0;
			_projCount--;
			continue;
		}

		/* increment penetration counter */
		proj->penetrations++;
		
		/* offset projectile scatter angle */
		int scatterSeed = i + (collisionBound - _bBuffer)
			+ proj->penetrations;
		float scatterAngle = seededRandomFLOAT(scatterSeed,
			pb.penetrationScatter);

		/* get polar coordiantes of movement vector and add */
		vfVector polarMoveVector = cartesianToPolar(proj->movement);
		polarMoveVector.y += (scatterAngle * 0.0174533f);
		
		/* slow down movement vector */
		float moveMagnitudeUpdated =
			polarMoveVector.x * (1.0f - pb.penetrationSlowdown);

		/* assign new movement vector */
		proj->movement = polarToCartesian(moveMagnitudeUpdated,
			polarMoveVector.y);

		/* call collision callback */
		if (pb.collisionCallback)
			pb.collisionCallback(proj, collisionBound->entity);

		/* if there's an entity attatched to the bound, set */
		/* entity to new last collided */
		if (collisionBound->entity)
			proj->lastPenetrated = collisionBound->entity;

		/* check for ricochet chance */
		if (fabsf(seededRandomNORMALIZED(_tickCount)) <
			pb.ricochetChance)
		{
			/* flip x and y movement */
			proj->movement.x *= -1;
			proj->movement.y *= -1;

			/* convert to polar coordiantes and scatter */
			vfVector ricoPolar = cartesianToPolar(proj->movement);
			ricoPolar.y += seededRandomFLOAT(i + _tickCount,
				pb.ricochetScatter) * 0.0174533f;

			/* slow down vector magnitude */
			ricoPolar.x *= (1.0f - pb.ricochetSlowdown);

			/* convert back to cartesian */
			proj->movement = polarToCartesian(ricoPolar.x,
				ricoPolar.y);
		}

		/* create shrapnel */
		if (pb.useShrapnel)
		{
			/* calculate amount of shrapnel to create */
			int sCount = max(0, pb.shrapnelCount -
				proj->penetrations * pb.shrapnelFalloff);

			for (int k = 0; k < sCount; k++)
			{
				/* no shrapnel condition */
				if (fabsf(seededRandomNORMALIZED(_tickCount))
			> pb.shrapnelChance
					&& pb.shrapnelChance != 1.0f
					|| pb.shrapnelChance == 0.0f) continue;

				/* get rand offset*/
				int scatterSeed = _tickCount + i + 
					(collisionBound - _bBuffer) + k;
				float scatterR = seededRandomFLOAT(scatterSeed,
					pb.shrapnelScatter);

				/* get rotation */
				float angle = atan2f(proj->movement.y,
					proj->movement.x);
				angle *= 57.2958f;

				/* get source entity */
				vfEntity* srcEnt = proj->lastPenetrated;

				/* can't create shrapnel on non-entity collide */
				if (!srcEnt) continue;

				/* create new projectile */
				vfCreateProjectileR(srcEnt,
					pb.shrapnelBehavior,
					angle + scatterR);
			} /* shrapnel loop end */
		} /* shrapnel check */

		/* check if projectile should be destroyed due to penetration */
		if (proj->penetrations >= pb.penetrationPower &&
			!pb.infinitePenetration)
		{
			/* set flag and decrement count */
			_projBufferField[i] = 0;
			_projCount--;
		}
	}
}


/* ========== INTERNAL EXPLOSION FUNCTIONS ========== */


static void getPartitionBoundEntIndexes(int partX, int partY, 
	int* indexBuffer, int* bufferUse, int bufferMax)
{
	/* search for partition */
	partition* part = NULL;
	for (int i = 0; i < _partitionCount; i++)
	{
		/* on collide, break */
		partition* checkPart = _partBuff + i;
		if (checkPart->x == partX &&
			checkPart->y == partY   )
		{
			part = checkPart;
			break;
		}
	}

	/* on not found, return */
	if (part == NULL) return;

	/* loop through partition and add bound index */
	for (int i = 0; i < part->bqCount; i++)
	{
		/* get check index */
		int checkIndex = part->bqIndexes[i];
		int collisionResult = 0;

		/* check for collide */
		for (int j = 0; j < *bufferUse; j++)
		{
			if (indexBuffer[j] != checkIndex) continue;

			/* on collide, set flag and break */
			collisionResult = 1;
			break;
		}

		/* on collision, skip */
		if (collisionResult) continue;

		/* add index and increment bufferuse */
		if (*bufferUse + 1 > bufferMax) return;
		indexBuffer[*bufferUse] = checkIndex;
		(*bufferUse)++;
	}
}

static void getAllAffected(vfExplosion* source,
	vfExplosionBehavior bhv)
{
	/* set bufuse for pushbuffer */
	int buffUse = 0;

	/* get minimum explosion partitions */
	float minPosX = source->position.x - bhv.radius;
	float minPosY = source->position.y - bhv.radius;
	int minpartX, minpartY;

	/* account for sign */
	if (minPosX > 0)
		minpartX = minPosX / _partitionSize;
	else
		minpartX = floorf(minPosX / _partitionSize);

	if (minPosY > 0)
		minpartY = minPosY / _partitionSize;
	else
		minpartY = floorf(minPosY / _partitionSize);

	/* account for range */
	int partRange = (bhv.radius * 2.0f) / _partitionSize;

	/* populate buffer based on partition range */
	for (int i = minpartX; i < minpartX + partRange; i++)
	{
		for (int j = minpartY; j < minpartY + partRange; j++)
		{
			getPartitionBoundEntIndexes(i, j,
				_expPushBuffer, &buffUse, VF_EXPPUSH_MAX);
		}
	}

	/* allocate explosion entity buffer */
	if (source->pushBounds)
		vFree(source->pushBounds);
	source->pushBounds = vAlloc(sizeof(vfBound*) * buffUse, FALSE);

	/* populate entity buffer */
	for (int i = 0; i < buffUse; i++)
	{
		source->pushBounds[i] = _bBuffer + _expPushBuffer[i];
	}
	source->boundCount = buffUse;
}

static float getOcclusionFalloff(vfExplosion* source, vfBound* target,
	float angle, float dist, vfExplosionBehavior bhv,
	int* collisionCount)
{
	/* get stepcount */
	int stepCount = dist / VF_EXPOCCLUSION_STEP;

	/* get walk vector */
	vfVector moveVect = polarToCartesian(VF_EXPOCCLUSION_STEP,
		(angle + 180.0f) * 0.0174533f);
	
	/* get falloff values */
	float falloff = bhv.occlusionFalloff;
	float entFalloff = bhv.occlusionEntDampen;

	/* walk and get collision count */
	int cCount = 0;
	vfVector checkPosition = source->position;
	float percent = 1.0f; /* percent accumulator */

	/* occlusion buffer, a bound can't occlude more than once */
	vfBound* occlusions[VF_EXPOCCLUSION_MAX] = { NULL };

	for (int i = 0; i < stepCount; i++)
	{
		/* get point partition */
		int pPartX, pPartY;

		if (checkPosition.x > 0)
			pPartX = checkPosition.x / _partitionSize;
		else
			pPartX = floorf(checkPosition.x / _partitionSize);

		if (checkPosition.y > 0)
			pPartY = checkPosition.y / _partitionSize;
		else
			pPartY = floorf(checkPosition.y / _partitionSize);

		/* get partition */
		partition* checkPart = NULL;
		for (int k = 0; k < _partitionCount; k++)
		{
			checkPart = _partBuff + k;
			if (checkPart->x == pPartX &&
				checkPart->y == pPartY) break;
		}

		/* if no partition, skip */
		if (checkPart == NULL) continue;
		
		/* check all bounds */
		for (int j = 0; j < checkPart->bqCount; j++)
		{
			/* on max cCount, break */
			if (cCount >= VF_EXPOCCLUSION_MAX ||
				(cCount >= bhv.occlusionsMax && bhv.useOcclusionMax)) break;

			/* get bound */
			vfBound* checkBound = _bBuffer + checkPart->bqIndexes[j];

			/* on missing bound, skip */
			if (checkBound == NULL) continue;

			/* on same bound, continue */
			if (checkBound == target) continue;

			/* on collide, mark */
			if (vfCheckPointOverlap(checkPosition,
				checkBound, VF_EXPOCCLUSION_LEN))
			{
				/* check all pre-occlusions, on collision, continue */
				int alreadyOccluded = 0;
				for (int k = 0; k < cCount; k++)
				{
					if (occlusions[k] == checkBound)
					{
						alreadyOccluded = 1;
						break;
					}
				}

				/* on already occluded, skip */
				if (alreadyOccluded) continue;

				/* mark and increment */
				occlusions[cCount] = checkBound;
				cCount++;

				/* apply dampening */
				float falloffActual = max(0.0f, falloff);
				if (checkBound->entity)
				{
					falloffActual *= max(0.0f, entFalloff);
				}
				
				percent *= (1.0f - falloffActual);
			}
		}

		/* on max cCount, break */
		if (cCount >= VF_EXPOCCLUSION_MAX ||
			(cCount >= bhv.maxLife && bhv.useOcclusionMax)) break;

		/* on percent = 0, end */
		if (percent == 0.0f) break;

		/* increment walk */
		checkPosition = vectorAdd(checkPosition, moveVect);
	}

	*collisionCount = cCount;
	return percent;
}

static void updateExplosions(void)
{
	int counter = 0;
	for (int i = 0; i < VF_EXPLOSIONS_MAX; i++)
	{
		/* on checked all, break */
		if (counter >= _expCount) break;

		/* on unused, skip */
		if (!_expBufferField[i]) continue;

		/* get explosion object and behavior */
		vfExplosion* exp = _expBuffer + i;
		vfExplosionBehavior bhv = _expBBuffer[exp->behavior];

		/* if explosion age is 0, populate buffer */
		/* and call creation callback             */
		if (exp->age == 0)
		{
			getAllAffected(exp, bhv);
			exp->previousShockwaveDistance = 0;
			if (bhv.createCallback)
				bhv.createCallback(exp);
		}

		/* get actual speed and dampen */
		float speedActual = bhv.shockwaveSpeed +
			seededRandomFLOAT(exp->spawnAge, bhv.shockwaveSpeedVariation);
		float decayFactor = min(1.0f,
			powf(1.0f - bhv.shockwaveSpeedDecay, exp->age));
		speedActual *= decayFactor;
		speedActual = max(0.0f, speedActual);

		/* increment shockwave distance by actual speed */
		exp->shockwaveDistance += speedActual;

		/* check all affected entities */
		for (int j = 0; j < exp->boundCount; j++)
		{
			/* if already collided or not exists, skip */
			if (exp->pushBounds[j] == NULL) continue;

			/* get entity to push */
			vfEntity* pushEnt = exp->pushBounds[j]->entity;

			/* if no entity, skip */
			if (pushEnt == NULL) continue;

			/* get offset */
			float distX = exp->position.x - pushEnt->transform->position.x;
			float distY = exp->position.y - pushEnt->transform->position.y;

			/* if floor of distance is 0, add random value to it */
			if (floorf(min(distX, distY)) == 0)
			{
				distX += randomFLOAT(1.0f);
				distY += randomFLOAT(1.0f);
			}

			/* get distance magnitude and check if in range  */
			float dist = hypotf(distX, distY);

			if (roundf(dist) < roundf(exp->shockwaveDistance) &&
				roundf(dist) > roundf(exp->previousShockwaveDistance
				- bhv.shockwaveThickness))
			{
				/* get angle */
				float angle = atan2f(distY, distX) * 57.2958;

				/* check for occlusion (if possible) */
				float occlusionFalloff = 1.0f;
				int collisions = 0;
				
				if (bhv.useOcclusion)
				{
					occlusionFalloff = getOcclusionFalloff(exp,
						exp->pushBounds[j],
						angle, dist, bhv, &collisions);
				}

				/* apply pushforce variation */
				angle += seededRandomFLOAT(exp->spawnAge + j,
					bhv.pushAngleVariation);

				/* calculate pushfactor */
				float pushfactor = bhv.pushFactor +
					seededRandomFLOAT(exp->spawnAge,
						bhv.pushFactorVariation);

				/* calculate falloff and apply falloff */
				float pushfalloff = bhv.pushFalloff +
					seededRandomFLOAT(exp->spawnAge + j,
						bhv.pushFactorVariation);
				pushfactor *= max(1.0f, powf(1.0f - pushfalloff,
					exp->age));
				pushfactor = max(0.0f, pushfactor);

				/* account for mass */
				float massDampen = (pushEnt->physics.mass *
					bhv.pushMassScale);
				massDampen = max(0, massDampen);

				/* get pushvector magnitude */
				float pushMag = (speedActual * pushfactor * occlusionFalloff)
					- massDampen;
				pushMag = max(0, pushMag);

				/* convert back to cartesian and apply vector to entity */
				vfVector pushVector = polarToCartesian(pushMag,
					angle * 0.0174533f);

				pushEnt->physics.velocity.x -= pushVector.x;
				pushEnt->physics.velocity.y -= pushVector.y;

				/* try callback */
				if (bhv.pushCallback)
					bhv.pushCallback(exp, pushEnt);

				/* once collided, now mark as null */
				exp->pushBounds[j] = NULL;
			} /* PUSH CHECK END */
		} /* ENT CHECK LOOP END */

		/* increment age */
		exp->age++;

		/* on max radius or speed == 0, destroy */
		if (exp->shockwaveDistance > bhv.radius ||
			floorf(speedActual) == 0.0f)
		{
			vfDestroyExplosion(exp);
		}

		/* on min speed or max life, destroy */
		if ((exp->age > bhv.maxLife && bhv.useMaxLife) ||
			(speedActual < bhv.minShockwaveSpeed && bhv.useMinSpeed))
		{
			vfDestroyExplosion(exp);
		}

		/* update previous shockwave distance */
		exp->previousShockwaveDistance = exp->shockwaveDistance;

		counter++;
	} /* EXPLOSION LOOP END */
}

/* ============ MODULE MAIN FUNCTION ============ */
static DWORD WINAPI vfMain(void* params)
{
	ULONGLONG lastTime = 0;
	
	/* get ahold of kill mutex */
	int result = WaitForSingleObject(_killMutex, 0);

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
			ULONGLONG startTime = GetTickCount64();
			updateCollisions();
			ULONGLONG endTime = GetTickCount64();
			_pCollisionCheckTime = (int)(endTime - startTime);

			/* handle velocity changes from collisions */
			updateCollisionVelocities();

			/* update projectiles */
			updateProjectiles();

			/* update phyiscs ages */
			updatePhyiscsAges();

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
			_pUpdateTime = (int)(endTickCount - startTickCount);

			/* update particles */
			updateParticles();

			/* update explosions */
			updateExplosions();
		}
		else
		{
			_pCollisionCheckCount = -1;
			_pPartitionCheckCount = -1;
			_pCollisionCheckTime = -1;
			_pUpdateTime = -1;
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
	_pEnabled  = 0;
	_tCount  = 0;
	_bCount  = 0;
	_pCount  = 0;
	_pbCount = 0;
	_eCount  = 0;
	_atCount = 0;
	_projCount = 0; _projBCount = 0;
	_expCount = 0;  _expBCount = 0;

	/* set first attribute to be empty */
	vfAttributeTable attribTableDefault = { 0 };
	vfAttribTableRegister(attribTableDefault);

	/* init partition data */
	_partitionsAllocated = VF_PART_COUNT_INCREMENT;
	_partitionCount = 0;
	_partBuff = vAlloc(sizeof(partition) * _partitionsAllocated,
		TRUE);
	_partitionSize = VF_PART_SIZE_DEFAULT;
	_partitionsExtraRequested = 0;
	_partitionsMax = VF_PART_COUNT_MAXIMUM;

	/* clear attribute table data */
	ZeroMemory(_atBuffer, sizeof(_atBuffer));

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
	_tBuffer = vAlloc(sizeof(vfTransform) * VF_BUFFER_SIZE,
		TRUE);
	_tBufferField = vAlloc(sizeof(field) * VF_BUFFER_SIZE,
		TRUE);

	/* INIT FINAL TRANSFORM BUFFER */
	_tFinalBuffer = vAlloc(sizeof(vfTransform) * VF_BUFFER_SIZE,
		TRUE);

	/* INIT PARTICLE BUFFERS */
	_pBuffer = vAlloc(sizeof(vfParticle) * VF_BUFFER_SIZE,
		TRUE);
	_pBufferField = vAlloc(sizeof(field) * VF_BUFFER_SIZE,
		TRUE);
	_pbBuffer = vAlloc(sizeof(vfParticleBehavior) *
		VF_PB_MAX, TRUE);

	/* INIT BOUND BUFFER */
	_bBuffer = vAlloc(sizeof(vfBound) * VF_BUFFER_SIZE,
		TRUE);
	_bBufferField = vAlloc(sizeof(field) * VF_BUFFER_SIZE,
		TRUE);

	/* INIT BOUNDQUAD BUFFER */
	_bqBuffer = vAlloc(sizeof(boundQuad) * VF_BUFFER_SIZE,
		TRUE);

	/* INIT ENTITY BUFFER */
	_eBuffer = vAlloc(sizeof(vfEntity) * VF_BUFFER_SIZE,
		TRUE);
	_eBufferField = vAlloc(sizeof(field) * VF_BUFFER_SIZE,
		TRUE);

	/* INIT PROJECTILE BUFFER */
	_projBuffer = vAlloc(sizeof(vfProjectile) * VF_PROJCOUNT_DEFAULT,
		TRUE);
	_projBufferField = vAlloc(sizeof(field) * VF_PROJCOUNT_DEFAULT,
		TRUE);
	_projAllocated = VF_PROJCOUNT_DEFAULT;

	/* INIT EXPLOSION BUFFER */
	_expBuffer = vAlloc(sizeof(vfExplosion) * VF_EXPLOSIONS_MAX,
		TRUE);
	_expBufferField = vAlloc(sizeof(field) * VF_EXPLOSIONS_MAX,
		TRUE);
	_expPushBuffer = vAlloc(sizeof(int) * VF_EXPPUSH_MAX,
		FALSE);

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
	vFree(_tBuffer);
	vFree(_bBuffer);
	vFree(_pBuffer);
	vFree(_pbBuffer);
	vFree(_bqBuffer);
	vFree(_tFinalBuffer);
	vFree(_projBuffer);
	vFree(_expBuffer);
	vFree(_expPushBuffer);

	/* free all entity attribute data */
	for (int i = 0; i < VF_BUFFER_SIZE; i++)
	{
		if (_eBuffer[i].attribBlock != NULL)
			HeapFree(_heap, FALSE, _eBuffer[i].attribBlock);
	}

	/* free entity buffer */
	vFree(_eBuffer);

	/* free all buffer fields */
	vFree(_tBufferField);
	vFree(_bBufferField);
	vFree(_pBufferField);
	vFree(_eBufferField);
	vFree(_projBufferField);
	vFree(_expBufferField);

	/* free all partitions */
	for (int i = 0; i < _partitionsAllocated; i++)
	{
		if (_partBuff[i].bqIndexes != NULL)
			vFree(_partBuff[i].bqIndexes);
	}
	vFree(_partBuff);

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

VFAPI vfTransform* vfCreateTransformV(vfVector vector)
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
	rTransform->rotation = 0;
	rTransform->scale    = 1;
	rTransform->parent = VF_NOPARENT;

	return rTransform;
}

VFAPI vfTransform* vfCreateTransformA(vfVector vector, float rotation,
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

VFAPI vfTransform* vfCreateTransformP(vfTransform* parent)
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

VFAPI vfBound* vfCreateBoundT(vfTransform* body)
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

VFAPI vfBound* vfCreateBoundA(vfTransform* body, vfVector position,
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
	rPhys.age = 0;

	return rPhys;
}

VFAPI vfPhysics vfCreatePhysicsA(float bounciness, float drag, float mass,
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
	rPhys.age = 0;
	
	return rPhys;
}

VFAPI vfEntity* vfCreateEntity(vfLayer layer, vgShape shape,
	vgTexture texture, vfPhysics physics, vfVector boundPosition,
	vfVector boundDimensions)
{
	captureMutex("Entity creation timeout");

	/* find free spot */
	int eIndex = findBufferSpot(_eBuffer, _eBufferField,
		sizeof(vfEntity));
	_eCount++;
	
	/* get return entity and init values */
	vfEntity* rEnt = _eBuffer + eIndex;

	rEnt->transform = vfCreateTransformA(vfCreateVector(0, 0), 0, 1);
	rEnt->bounds = vfCreateBoundA(rEnt->transform, boundPosition,
		boundDimensions);
	rEnt->layer = layer;
	rEnt->filter = vfCreateColor(255, 255, 255, 255);
	rEnt->physics = physics;
	rEnt->shape = shape;
	rEnt->texture = texture;
	vfBound* entBounds = rEnt->bounds;
	entBounds->entity = rEnt;
	rEnt->collisionCallback = NULL;
	
	/* set attributes data */
	rEnt->attribHandle = 0;
	rEnt->attribBlock  = NULL;
	rEnt->getAttribute = getAttributeInternal;

	/* set active last */
	rEnt->active = TRUE;

	/* mark field */
	_eBufferField[eIndex] = 1;

	releaseMutex( );
	return rEnt;
}

/* STRUCT DESTRUCTION FUNCTIONS */

VFAPI void vfDestroyTransform(vfTransform* transform, int zero)
{
	/* check already destroyed */
	if (_tBufferField[transform - _tBuffer] == 0) return;

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
	/* check already destroyed */
	if (_bBufferField[bound - _bBuffer] == 0) return;

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

VFAPI void vfDestroyParticle(vfParticle* particle)
{
	/* check already destroyed */
	if (_pBufferField[particle - _pBuffer] == 0) return;

	captureMutex("Particle Destruction Timeout");

	/* get index and update buffer field */
	int index = particle - _pBuffer;
	_pBufferField[index] = 0;
	_pCount--;

	ZeroMemory(particle, sizeof(vfParticle));

	releaseMutex();
}

VFAPI void vfDestroyEntity(vfEntity* entity, int zero)
{
	/* check already destroyed */
	if (_eBufferField[entity - _eBuffer] == 0) return;

	captureMutex("Entity destruction timeout");

	/* free entity attribute buffer (if exists) */
	if (entity->attribBlock)
	{
		HeapFree(_heap, FALSE,
			entity->attribBlock);
	}
	entity->attribBlock = NULL; /* zero block ptr */

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

VFAPI void vfDestroyAllBounds(int zero)
{
	for (int i = 0; i < VF_BUFFER_SIZE; i++)
		vfDestroyBound(_bBuffer + i, zero);
}

VFAPI void vfDestroyAllNonEntityBounds(int zero)
{
	for (int i = 0; i < VF_BUFFER_SIZE; i++)
	{
		vfBound* boundToDestroy = _bBuffer + i;
		if (boundToDestroy->entity == NULL)
			vfDestroyBound(boundToDestroy, zero);
	}	
}

VFAPI void vfDestroyAllParticles(int zero)
{
	for (int i = 0; i < VF_BUFFER_SIZE; i++)
		vfDestroyParticle(_pBuffer + i, zero);
}

VFAPI void vfDestroyAllEntities(int zero)
{
	for (int i = 0; i < VF_BUFFER_SIZE; i++)
		vfDestroyEntity(_eBuffer + i, zero);
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
static void renderParticleAlphaGroup(int low, int high)
{
	int checkCount = 0;
	for (int i = 0; i < VF_BUFFER_SIZE; i++)
	{
		/* if checked all particles, break */
		if (checkCount >= _pCount) break;

		if (!_pBufferField[i]) continue;
		checkCount++;
		vfParticle render = _pBuffer[i];

		/* get FINAL transform */
		vfTransform tFinal = render.transform;

		/* if filter alpha is 0, skip render */
		if (render.filter.a == 0) continue;

		/* if alpha does not satisfy range, continue */
		if (render.filter.a < low)  continue;
		if (render.filter.a > high) continue;

		/* if particle is out of viewable zone, continue */
		if (!vgCheckIfViewable(render.transform.position.x,
			render.transform.position.y, VF_PARTICLE_CULLEXTRA)) continue;

		vgRenderLayer(render.layer);
		vgUseTexture(render.texture);

		vgTextureFilter(render.filter.r, render.filter.g,
			render.filter.b, render.filter.a);

		vgDrawShapeTextured(render.shape, tFinal.position.x, tFinal.position.y,
			tFinal.rotation, tFinal.scale);
	}
}

VFAPI void vfRenderParticles(void)
{
	/* check for render skip */
	if (vgGetRenderSkipState()) return;

	/* get render permission */
	int mResult = WaitForSingleObject(_drawMutex, VF_RMUTEX_TIMEOUT);
	if (mResult != WAIT_OBJECT_0) showMutexError("DrawMutex",
		"Particle render timout");

	/* RENDER 6 ALPHA GROUPS */
	renderParticleAlphaGroup(0xC0, 0xFF);
	renderParticleAlphaGroup(0x80, 0xC0);
	renderParticleAlphaGroup(0x60, 0x80);
	renderParticleAlphaGroup(0x40, 0x60);
	renderParticleAlphaGroup(0x20, 0x40);
	renderParticleAlphaGroup(0x00, 0x20);

	vgTextureFilterReset();
	vgRenderLayer(0);

	/* release mutex */
	ReleaseMutex(_drawMutex);
}

static int renderEntityAlphaGroup(int low, int high) 
{
	int eChecked = 0;
	int eRendered = 0;
	for (int i = 0; i < VF_BUFFER_SIZE; i++)
	{
		if (eRendered >= _eCount) return 0; /* all done */
		if (eChecked >= _eCount)  return 1; /* more to render */

		if (_eBufferField[i] == 0) continue;

		vfEntity renderEnt = _eBuffer[i];
		if (!renderEnt.active) { eChecked++; continue; }

		/* grab the current transform of the entity */
		vfTransform* tTemp = renderEnt.transform;

		/* grab the final transform of the entity */
		int tIndex = tTemp - _tBuffer;
		vfTransform tFinal = _tFinalBuffer[tIndex];

		/* check if should cull */
		if (!vgCheckIfViewable(tFinal.position.x,
			tFinal.position.y, VF_ENTITY_CULLEXTRA))
		{
			eChecked++; eRendered++; continue;
		}

		/* check if just spawned in (don't render) */
		if (renderEnt.physics.age <= 1)
		{
			eChecked++; eRendered++; continue;
		}

		/* check if should skip */
		if (renderEnt.filter.a < low) { eChecked++; continue; }
		if (renderEnt.filter.a > high) { eChecked++; continue; }

		/* render */
		vgRenderLayer(renderEnt.layer);
		vgUseTexture(renderEnt.texture);
		vgTextureFilter(renderEnt.filter.r, renderEnt.filter.g, renderEnt.filter.b,
			renderEnt.filter.a);
		vgDrawShapeTextured(renderEnt.shape, tFinal.position.x,
			tFinal.position.y, tFinal.rotation, tFinal.scale);

		/* increment render and check count */
		eRendered++; eChecked++;
	}
	return 1;
}

VFAPI void vfRenderEntities(void)
{
	/* check for render skip */
	if (vgGetRenderSkipState()) return;

	/* get render permission */
	int mResult = WaitForSingleObject(_drawMutex, VF_RMUTEX_TIMEOUT);
	if (mResult != WAIT_OBJECT_0) showMutexError("DrawMutex",
		"Entity render timeout");

	/* render entity alpha groups */
	int renderCheck = renderEntityAlphaGroup(0x90, 0xFF);
	if (renderCheck)
	{
		renderEntityAlphaGroup(0x40, 0x90);
		renderEntityAlphaGroup(0x00, 0x40);
	}

	vgTextureFilterReset();
	vgRenderLayer(0);

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

	/* render all bounds */
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
	

	/* render all explosion bounds */
	for (int i = 0; i < VF_EXPLOSIONS_MAX; i++)
	{
		vfExplosion* exp = _expBuffer + i;
		vfExplosionBehavior bhv = _expBBuffer[exp->behavior];

		/* on unused, skip */
		if (!_expBufferField[i]) continue;

		/* render center point */
		vgColor3(0xFF, 0x80, 0x20);
		vgPointf(exp->position.x, exp->position.y);

		/* render OUTER circumfrence  */
		float angleStep = 360.0f / VF_EXPRENDER_VERTS;
		for (int j = 0; j < VF_EXPRENDER_VERTS; j++)
		{
			/* get circle verts */
			vfVector vert1 = polarToCartesian(exp->shockwaveDistance,
				angleStep * (j)     * 0.0174533f);
			vfVector vert2 = polarToCartesian(exp->shockwaveDistance,
				angleStep * (j + 1) * 0.0174533f);
			
			/* offset by explosion position */
			vert1 = vectorAdd(vert1, exp->position);
			vert2 = vectorAdd(vert2, exp->position);

			/* render line */
			vgLinef(vert1.x, vert1.y, vert2.x, vert2.y);
		}

		/* render INNER */
		for (int j = 0; j < VF_EXPRENDER_VERTS; j++)
		{
			/* get circle verts */
			vfVector vert1 = polarToCartesian(exp->shockwaveDistance
				- bhv.shockwaveThickness,
				angleStep * (j) * 0.0174533f);
			vfVector vert2 = polarToCartesian(exp->shockwaveDistance
				- bhv.shockwaveThickness,
				angleStep * (j + 1) * 0.0174533f);

			/* offset by explosion position */
			vert1 = vectorAdd(vert1, exp->position);
			vert2 = vectorAdd(vert2, exp->position);

			/* render line */
			vgLinef(vert1.x, vert1.y, vert2.x, vert2.y);
		}
	}

	vgLineSize(1.0f);
}

VFAPI void vfRenderPartitions(void)
{
	/* check for framskip */
	if (vgGetRenderSkipState()) return;

	/* get draw mutex */
	int wResult = WaitForSingleObject(_drawMutex, VF_RMUTEX_TIMEOUT);
	if (wResult != WAIT_OBJECT_0) return; /* on fail, don't render */

	/* set render layer*/
	vgRenderLayer(VF_PART_RENDERLAYER);

	/* render lines */
	vgLineSize(1.5f);
	/* render x and y axis */
	vgColor3(0xFF, 0x20, 0x20); vgLine(INT_MAX, 0, INT_MIN, 0);
	vgColor3(0x20, 0x20, 0xFF); vgLine(0, INT_MAX, 0, INT_MIN);
	/* render all partition boundaries */
	vgColor3(0x20, 0xFF, 0x20);
	for (int i = 0; i < _partitionCount; i++)
	{
		/* get partition */
		partition renderPart = _partBuff[i];

		/* if out of bounds, don't render */
		float partCenterX = (renderPart.x * _partitionSize)
			+ (_partitionSize / 2);
		float partCenterY = (renderPart.y * _partitionSize)
			+ (_partitionSize / 2);
		if (!vgCheckIfViewable(partCenterX, partCenterY, 0)) continue;

		/* get partition bounding box */
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

		/* if partition velsum is 0, skip rect */
		if (floorf(renderPart.velSum) == 0) continue;

		/* draw rect */
		vgColor4(min(renderPart.bqCount * 0x40, 0xFF), 0x80, 0x80, 0x40);
		vgRect(renderPart.x * _partitionSize,
			   renderPart.y * _partitionSize,
			_partitionSize,
			_partitionSize);
	}

	ReleaseMutex(_drawMutex);
}

static void renderProjectileTrailAlphaGroup(int aMin, int aMax)
{
	/* NEXT, RENDER ALL TRAILS */
	for (int i = 0; i < _projAllocated; i++)
	{
		/* on unused, skip */
		if (!_projBufferField[i]) continue;

		/* get object data */
		vfProjectile* proj = _projBuffer + i;
		vfProjectileBehavior* pb = _projBBuffer + proj->behaviorHandle;

		/* on no trail, skip */
		if (!pb->renderTrail) continue;

		/* check if should be rendered */
		if (pb->invisible) continue;
		if (proj->age < pb->renderAgeStart) continue;

		vgTextureFilterReset();
		vgUseTexture(pb->texture);

		/* render in decreasing alpha order */
		vfVector renderPos = proj->position;
		for (int i = 0; i < pb->renderTrail; i++)
		{
			/* get trail alpha */
			int renderAlpha = max(0,
				255 - (pb->renderTrailAlphaFalloff * i));

			/* check if out of view */
			if (!vgCheckIfViewable(proj->position.x, proj->position.y,
				VF_PROJECTILE_CULLEXTRA)) continue;

			/* check if should alpha skip*/
			if (renderAlpha == 0   ||
				renderAlpha > aMax ||
				renderAlpha < aMin) continue;

			/* set filter */
			vgTextureFilter(255, 255, 255,
				renderAlpha);

			/* decrement draw position */
			float lagX = proj->movement.x / pb->shapeScale;
			float lagY = proj->movement.y / pb->shapeScale;
			renderPos.x -= lagX * pb->renderTrailLag;
			renderPos.y -= lagY * pb->renderTrailLag;

			/* draw trail */
			float renderScale = max(0, pb->shapeScale -
				pb->renderTrailScaleFalloff * i);
			vgDrawShapeTextured(pb->shape, renderPos.x,
				renderPos.y, 0, renderScale);
		}
	}
}

VFAPI void vfRenderProjectiles(void)
{
	if (vgGetRenderSkipState()) return;

	/* get write mutex */
	int result = WaitForSingleObject(_writeMutex, VF_WMUTEX_TIMEOUT);
	if (result != WAIT_OBJECT_0)
		showMutexError("Write Mutex", "Could not draw projectiles!");

	/* FIRST, RENDER ALL PROJECTILES */
	for (int i = 0; i < _projAllocated; i++)
	{
		if (!_projBufferField[i]) continue; /* on unused, continue */
		
		/* get object data */
		vfProjectile* proj = _projBuffer + i;
		vfProjectileBehavior* pb = _projBBuffer + proj->behaviorHandle;

		/* check if should be rendered */
		if (pb->invisible) continue;
		if (proj->age < pb->renderAgeStart) continue;

		vgRenderLayer(pb->layer);
		vgTextureFilterReset();
		vgUseTexture(pb->texture);
		vgDrawShapeTextured(pb->shape, proj->position.x,
			proj->position.y, 0, pb->shapeScale);
	}

	/* RENDER TRAIL ALPHA GROUPS */
	renderProjectileTrailAlphaGroup(0x80, 0xFF);
	renderProjectileTrailAlphaGroup(0x20, 0x80);
	renderProjectileTrailAlphaGroup(0x00, 0x20);

	/* release write mutex */
	ReleaseMutex(_writeMutex);
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

VFAPI void vfSetPartitionMaxCount(int value)
{
	_partitionsMax = value;
}

VFAPI void vfGetPhysicsTickCount(vfTickCount* ticks)
{
	*ticks = _tickCount;
}

VFAPI int vfGetPhysicsUpdateTime(void)
{
	return _pUpdateTime;
}

VFAPI void vfGetPhysicsCollisionCounts(int* objCheckCount, 
	int* partCheckCount)
{
	*objCheckCount = _pCollisionCheckCount;
	*partCheckCount = _pPartitionCheckCount;
}

VFAPI void vfLogPhysicsPartitionData(FILE* file)
{
	fprintf(file, "========== TICKCOUNT: %08lld ==========\n", _tickCount);
	fprintf(file, "Part count: %d/%d\nPart size: %d\n",
		_partitionCount, _partitionsAllocated, _partitionSize);
	for (int i = 0; i < _partitionCount; i++)
	{
		/* print general data */
		partition* p = _partBuff + i;
		fprintf(file, "{%d} [%d, %d] size: %d used: %d vel: %f/%f\n",
			i, p->x, p->y, p->bqBuffSize, p->bqCount,
			p->velSum, floorf(p->velSum));

		/* print partition indexes */
		for (int j = 0; j < p->bqCount; j++)
		{
			/* newline every 20 elements*/
			if (j % 20 == 0 && j != 0) fprintf(file, "\n");

			/* print index */
			fprintf(file, "%02d ", p->bqIndexes[j]);
		}
		fprintf(file, "\n");
	}
	/* flush buffer */
	fflush(file);
}

VFAPI void vfLogPhysicsCollisionData(FILE* file)
{
	fprintf(file, "========== TICKCOUNT: %08lld ==========\n", _tickCount);
	fprintf(file, "Part count: %d/%d\nPart size: %d\n",
		_partitionCount, _partitionsAllocated, _partitionSize);
	fprintf(file, "Bound collisions checked: %d/%d\n", _pCollisionCheckCount,
		_bCount);
	fprintf(file, "Partitions traversed: %d/%d\n", _pPartitionCheckCount,
		_partitionCount);
	fprintf(file, "Time taken to calculate: %d ms\n",
		_pCollisionCheckTime);
	fflush(file);
}

VFAPI int vfCheckPointOverlap(vfVector point, vfBound* bound,
	float leniency)
{
	/* make bound origin point */
	point.x -= bound->body->position.x;
	point.y -= bound->body->position.y;

	/* rotate and scale point so that coordinate frame is relative */
	/* to bound */
	point = vertRotateScale(point, -bound->body->rotation,
		1.0f / bound->body->scale);

	/* check for overlap */
	int overlapCheck = 0;

	/* get min and max of all bound vertexes */
	float xMin = min(bound->position.x,
		bound->position.x + bound->dimensions.x);
	float xMax = max(bound->position.x,
		bound->position.x + bound->dimensions.x);
	float yMin = min(bound->position.y,
		bound->position.y + bound->dimensions.y);
	float yMax = max(bound->position.y,
		bound->position.y + bound->dimensions.y);

	/* test for overlap */
	if (point.x > xMin - leniency) overlapCheck++;
	if (point.x < xMax + leniency) overlapCheck++;
	if (point.y > yMin - leniency) overlapCheck++;
	if (point.y < yMax + leniency) overlapCheck++;

	return (overlapCheck == 4);
}

/* ========== PARTICLE RELATED FUNCTIONS ========== */

VFAPI void vfCreateParticle(vgShape shape, vgTexture texture,
	vfColor filter, vfLifeTime lifeTime, vfLayer layer,
	vfVector position, vfHandle behavior)
{
	/* find buffer spot */
	int pIndex = findBufferSpot(_pBuffer, _pBufferField, sizeof(vfParticle));

	/* get ref and set values */
	vfParticle* pRef = _pBuffer + pIndex;
	pRef->lifeTime = lifeTime;
	pRef->layer = layer;
	pRef->shape = shape;
	pRef->texture = texture;
	pRef->filter = filter;

	/* set transform values */
	pRef->transform.position = position;
	pRef->transform.scale    = 1;
	pRef->transform.rotation = 0;

	/* set behavior */
	if (behavior != VF_PB_ERROR)
		pRef->behavior = _pbBuffer[behavior];
	pRef->behavior.parent = pRef;

	/* increment particle count and mark field */
	_pBufferField[pIndex] = 1;
	_pCount++;
}

VFAPI void vfCreateParticleT(vgShape shape, vgTexture texture,
	vfColor filter, vfLifeTime lifeTime, vfLayer layer,
	vfTransform transform, vfHandle behavior)
{
	/* find buffer spot */
	int pIndex = findBufferSpot(_pBuffer, _pBufferField, sizeof(vfParticle));

	/* get ref and set values */
	vfParticle* pRef = _pBuffer + pIndex;
	pRef->lifeTime = lifeTime;
	pRef->layer = layer;
	pRef->shape = shape;
	pRef->texture = texture;
	pRef->filter = filter;

	/* set transform values */
	pRef->transform = transform;

	/* set behavior */
	if (behavior != VF_PB_ERROR)
		pRef->behavior = _pbBuffer[behavior];
	pRef->behavior.parent = pRef;

	/* increment particle count and mark field */
	_pBufferField[pIndex] = 1;
	_pCount++;
}

VFAPI vfHandle vfCreateParticleBehavior(PARTUPDCALLBACK behavior)
{
	/* check for full */
	if (_pbCount >= VF_PB_MAX) return VF_PB_ERROR;

	/* get particle behavior ref and set data */
	vfParticleBehavior* pbRef = _pbBuffer + _pbCount;
	ZeroMemory(pbRef, sizeof(vfParticleBehavior));
	pbRef->updateBehavior = behavior;

	_pbCount++; /* increment behavior count */

	return pbRef - _pbBuffer; /* return index */
}

VFAPI vfHandle vfCreateParticleBehaviorP(vfParticleBehavior reference)
{
	/* check for full */
	if (_pbCount >= VF_PB_MAX) return VF_PB_ERROR;

	/* get particle behavior ref and set data */
	vfParticleBehavior* pbRef = _pbBuffer + _pbCount;
	*pbRef = reference;
	pbRef->parent = NULL;

	_pbCount++; /* increment behavior count */

	return pbRef - _pbBuffer; /* return index */
}

/* ========== ATTRIBUTE RELATED FUNCTIONS ========== */

VFAPI vfHandle vfAttribTableRegister(vfAttributeTable toRegister)
{
	/* check max attribute tables registerd */
	if (_atCount >= VF_ATTRIBTABLES_MAX) return -1;

	_atBuffer[_atCount++] = toRegister;
	return _atCount - 1;
}

VFAPI void vfAttribTableAdd(vfAttributeTable* target,
	const char* attributeName, size_t memSize)
{
	/* get current attribute index */
	int attribIndex = target->attribCount;

	/* generate name and byte offset */
	target->attribNameHash[attribIndex] = generateStringHash(attributeName);
	target->attribByteOffset[attribIndex] = target->attribByteCount;
	
	/* increment bytecount and attribcount */
	target->attribByteCount += memSize;
	target->attribCount++;
}

VFAPI void vfSetEntityAttribHandle(vfEntity* entity, vfHandle handle)
{
	entity->attribHandle = handle;
	entity->attribBlock = HeapAlloc(_heap, HEAP_ZERO_MEMORY,
		_atBuffer[handle].attribByteCount);
}

/* ========== PROJECTILE RELATED FUNCTIONS ========== */
VFAPI vfHandle vfCreateProjectileBehavior(vgShape shape, vgTexture texture,
	uint8_t penetrationPower, float speed, float scale, vfLifeTime lifeTime)
{
	/* full check */
	if (_projBCount >= VF_PROJBEHAVIORS_MAX) return -1;

	/* get next projectile behavior and zero memory */
	vfProjectileBehavior* pbh = _projBBuffer + _projBCount;
	ZeroMemory(pbh, sizeof(vfProjectileBehavior));

	/* set all values */
	pbh->shape = shape;
	pbh->texture = texture;
	pbh->penetrationPower = penetrationPower;
	pbh->speed = speed;
	pbh->shapeScale = scale;
	pbh->boundScale = scale;
	pbh->maxAge = lifeTime;

	/* increment count and return */
	_projBCount++;
	return _projBCount - 1;
}

VFAPI vfHandle vfCreateProjectileBehaviorS(vfProjectileBehavior toCreate)
{
	/* full check */
	if (_projBCount >= VF_PROJBEHAVIORS_MAX) return -1;

	/* get next projectile behavior and zero memory */
	vfProjectileBehavior* pbh = _projBBuffer + _projBCount;
	ZeroMemory(pbh, sizeof(vfProjectileBehavior));

	/* copy over data */
	*pbh = toCreate;

	/* increment count and return */
	_projBCount++;
	return _projBCount - 1;
}

VFAPI vfProjectileBehavior vfGetProjectileBehavior(vfHandle pbHandle)
{
	return _projBBuffer[pbHandle];
}

VFAPI vfProjectileBehavior* vfGetProjectileBehaviorPTR(vfHandle pbHandle)
{
	return (_projBBuffer + pbHandle);
}

VFAPI vfHandle vfCreateProjectileV(vfEntity* source, vfHandle behavior,
	vfVector direction)
{
	vfVector polar = cartesianToPolar(direction);
	return vfCreateProjectileR(source, behavior, polar.y * 57.2958f);
}

static vfHandle createProjectileSingle(vfEntity* source,
	vfProjectileBehavior pb, vfHandle pbh,  float angle)
{
	/* make sure projectile buffer is big enough */
	ensureProjectileBufferSize();

	/* search for free projectile */
	int startIndex = _projCount / 2;
	for (int i = 0; i < _projAllocated; i++)
	{
		int indexActual = (startIndex + i) % _projAllocated;

		/* if taken, skip */
		if (_projBufferField[indexActual]) continue;

		/* on free buffer spot */
		_projBufferField[indexActual] = 1; /* mark used */
		vfProjectile* proj = _projBuffer + indexActual; /* get proj ptr */

		/* set values */
		proj->age = 0;
		proj->penetrations = 0;
		proj->lastPenetrated = NULL;

		proj->behaviorHandle = pbh;
		proj->source = source;
		proj->position = source->transform->position;

		/* randomize angle */
		angle += seededRandomFLOAT(indexActual, pb.spread);

		/* calculate speed */
		float speed = pb.speed + seededRandomFLOAT(indexActual,
			pb.speedVariation);

		proj->movement = polarToCartesian(speed,
			angle * 0.0174533);

		/* accumulate speed */
		proj->movement.x += source->physics.velocity.x;
		proj->movement.y += source->physics.velocity.y;

		/* increment projectile count */
		_projCount++;

		return indexActual;
	}
}

VFAPI vfHandle vfCreateProjectileR(vfEntity* source, vfHandle behavior,
	float direction)
{
	vfProjectileBehavior pb = _projBBuffer[behavior];
	vfHandle returnValue = 0;
	for (int i = 0; i < pb.pelletCount; i++)
	{
		int temp = createProjectileSingle(source, pb, behavior, direction);
		if (i == 0) returnValue = temp;
	}
	return returnValue;
}

VFAPI vfProjectile* vfGetProjectile(vfHandle handle)
{
	return _projBuffer + handle;
}

VFAPI void vfDestroyProjectile(vfProjectile* projectile)
{
	/* get buffer mutex */
	int result = WaitForSingleObject(_writeMutex, VF_WMUTEX_TIMEOUT);
	if (result != WAIT_OBJECT_0) showMutexError("Write Mutex",
		"Could not destroy projectile");

	/* clear flags and decrement count */
	int index = _projBuffer - projectile;
	_projBufferField[index] = 0;
	_projCount--;

	/* release buffer mutex */
	ReleaseMutex(_writeMutex);
}


/* EXPLOSION RELATED FUNCTIONS */
VFAPI vfHandle vfCreateExplosionBehavior(vfExplosionBehavior reference)
{
	/* bad size check */
	if (_expBCount >= VF_EXPBEHAVIORS_MAX) return -1;

	/* find free index */
	int addedIndex = _expBCount;
	_expBBuffer[addedIndex] = reference;

	/* increment and return */
	_expBCount++;
	return addedIndex;
}

VFAPI vfExplosionBehavior  vfGetExplosionBehavior(vfHandle handle)
{
	return _expBBuffer[handle];
}

VFAPI vfExplosionBehavior* vfGetExplosionBehaviorPTR(vfHandle handle)
{
	return _expBBuffer + handle;
}

VFAPI vfExplosion* vfCreateExplosionV(vfVector position, vfHandle behavior)
{
	/* find free buffer spot */
	for (int i = 0; i < VF_EXPLOSIONS_MAX; i++)
	{
		/* on free spot */
		if (!_expBufferField[i])
		{
			/* get explosion */
			vfExplosion* exp = _expBuffer + i;

			/* set members and return */
			exp->position = position;
			exp->age = 0;
			exp->behavior = behavior;

			exp->shockwaveDistance = 0;
			exp->previousShockwaveDistance = 0;

			exp->source = NULL;
			exp->boundCount = 0;
			exp->pushBounds = NULL;
			exp->spawnAge = _tickCount;

			/* set field and increment */
			_expBufferField[i] = 1;
			_expCount++;
			return exp;
		}
	}

	/* on exit, show error */
	MessageBoxA(NULL, "Explosion Buffer Full!\n",
		"FATAL ENGINE ERROR", MB_OK);
	exit(1);
	return -1;
}

VFAPI vfExplosion* vfCreateExplosionE(vfEntity* source, vfHandle behavior)
{
	vfExplosion* ent = vfCreateExplosionV(source->transform->position,
		behavior);
	ent->source = source;
	return ent;
}

VFAPI void vfDestroyExplosion(vfExplosion* toDestroy)
{
	/* get index and free */
	int index = toDestroy - _expBuffer;
	if (toDestroy->pushBounds)
		vFree(toDestroy->pushBounds);

	/* clear field and decrement count */
	_expBufferField[index] = 0;
	_expCount--;
	return;
}

/* ========= DATA RELATED FUNCTIONS ========== */

VFAPI void vfGetEntityPartitions(vfEntity* ent, int maxPartitions,
	int* xBuff, int* yBuff, int* xSize, int* ySize)
{
	/* get entity boundquad */
	vfBound* entBounds = ent->bounds;
	int bIndex = entBounds - _bBuffer;
	boundQuad* entBQ = _bqBuffer + bIndex;

	/* get partitions */
	partCheck(entBQ, maxPartitions, xSize, ySize, xBuff, yBuff);
}

/* DATA RELATED FUNCTIONS */
VFAPI void* vfGetBuffer(int type)
{
	/* return buffer point based on type */
	switch (type)
	{
	case VF_BUFF_TRANSFORM:
		return _tBuffer;

	case VF_BUFF_BOUND:
		return _bBuffer;

	case VF_BUFF_PARTICLE:
		return _pBuffer;

	case VF_BUFF_ENTITY:
		return _eBuffer;

	case VF_BUFF_PARTITION:
		return _partBuff;

	case VF_BUFF_PROJECTILE:
		return _projBuffer;
	
	/* fail condition */
	default:
		return NULL;
	}
}

VFAPI void* vfGetBufferField(int type)
{
	/* return field ptr (if exists) */
	switch (type)
	{
	case VF_BUFF_TRANSFORM:
		return _tBufferField;

	case VF_BUFF_BOUND:
		return _bBufferField;

	case VF_BUFF_PARTICLE:
		return _pBufferField;

	case VF_BUFF_ENTITY:
		return _eBufferField;

	case VF_BUFF_PROJECTILE:
		return _projBufferField;

	default:
		return NULL;
	}
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

	case VF_OBJ_PROJECTILE:
		count = _projCount;
		break;

	default:
		break;
	}

	return count;
}