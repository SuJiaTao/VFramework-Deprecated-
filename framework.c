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
static HANDLE _mutex; /* buffer use mutex */
static unsigned int _sleepTime;
static int _pEnabled; /* physics toggle */

/* internal buffers */

/* BIG AUXILLARY BUFFER */
static BYTE _mTank[VF_MEMTANK_SIZE + VF_MEMTANK_EXCESS];
static BYTE _mTField[VF_MEMTANK_SIZE / 8];

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

/* boundquad buffer, this buffer maps ever Bound object to a quad, which */
/* is the Bound object's dimensions and offset translated by it's transform */
static boundQuad* _bqBuffer;


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
		if ((field)[searchIndex] == 0) return i;
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
	/* GET COLLISION DATA */
	int outCheck = 0;
	for (int i = 0; i < VF_BUFFER_SIZE; i++)
	{
		if (outCheck >= _bCount) break;
		if (!_bBufferField[i]) continue;

		/* if bounds inactive, skip */
		if (!_bqBuffer[i].staticData.active) continue;

		int inCheck = 0;
		for (int j = 0; j < VF_BUFFER_SIZE; j++)
		{
			if (inCheck >= _bCount) break;
			if (!_bBufferField[j] || j == i) continue;
			collisionCheck(_bqBuffer + i, _bqBuffer + j);
			inCheck++;
		}

		inCheck++;
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
static DWORD WINAPI vfMain(void* params)
{
	ULONGLONG lastTime = 0;
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
		int mutStatus = WaitForSingleObject(_mutex, VF_MUTEX_TIMEOUT_INTERVAL);
		if (mutStatus != WAIT_OBJECT_0)
		{
			wchar_t errBuff[512];

			/* for every bound there is a boundQuad */
			int bMemSize = (sizeof(vfBound) + sizeof(boundQuad)) *
				VF_BUFFER_SIZE;
			/* multiply by 2 for tbuffer & tfinal */
			int tMemSize = sizeof(vfTransform) * VF_BUFFER_SIZE * 2;
			int eMemSize = sizeof(vfEntity) * VF_BUFFER_SIZE;

			/* err log */
			swprintf(errBuff, 512, L"Error Code: %d\nBounds Used/Allocated: %d/%d\n"
				L"Transforms Used/Allocated: %d/%d\nEntities Used/Allocated: %d/%d\n"
				L"Bytes allocated for Bounds: %d\n"
				L"Bytes allocated for Transforms: %d\n"
				L"Bytes allocated for Entities: %d\n", GetLastError(),
				_bCount, VF_BUFFER_SIZE, _tCount, VF_BUFFER_SIZE,
				_eCount, VF_BUFFER_SIZE, bMemSize,
				tMemSize, eMemSize);

			/* send messageboxes */
			MessageBox(NULL, L"[vfMain] Thread timewout in VFramework.dll",
				L"FATAL ERROR", MB_OK);
			MessageBox(NULL, errBuff, L"ERROR INFO", MB_OK);
			exit(1);
		}

		if (_pEnabled)
		{
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
		}

		/* RELEASE BUFFER OWNERSHIP */
		if (!ReleaseMutex(_mutex))
		{
			wchar_t errBuffer[255];
			swprintf(errBuffer, 255, L"[vfmain] Mutex relase failed! Err code: %d",
				GetLastError());
			MessageBox(NULL, errBuffer, L"FATAL ERROR", MB_OK);
			exit(1);
		}
	}
}

/* INIT AND TERMINATE FUNCTIONS */
VFAPI void vfInit(void)
{
	/* ger process heap */
	_heap = GetProcessHeap();

	/* init module internal data */
	_sleepTime = 0;
	_pEnabled = 0;
	_tCount = 0;
	_bCount = 0;
	_pCount = 0;
	_eCount = 0;

	/* init mutex */
	_mutex = CreateMutex(NULL, FALSE, NULL);
	if (_mutex == NULL)
	{
		MessageBox(NULL, L"Failed to create thread mutex object",
			L"FATAL ERR", MB_OK);
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

	return rParticle;
}

VFAPI vfParticle* vfCreateParticlea(vfTransform* transform, vgTexture texture,
	vgShape shape, unsigned char layer)
{
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
	/* get index and update buffer field */
	int index = particle - _pBuffer;
	_pBufferField[index] = 0;
	_pCount--;
	
	/* zero memory */
	if (zero)
	{
		ZeroMemory(particle, sizeof(vfParticle));
	}
}

VFAPI void vfDestroyEntity(vfEntity* entity, int zero)
{
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
	int rendered = 0;

	for (int i = 0; i < VF_BUFFER_SIZE; i++)
	{
		if (rendered >= _pCount) return;
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

		/* increment render count */
		rendered++;
	}
}

VFAPI void vfRenderEntities(void)
{
	int eRendered = 0;

	for (int i = 0; i < VF_BUFFER_SIZE; i++)
	{
		if (eRendered >= _eCount) return;
		if (_eBufferField[i] == 0) continue;

		vfEntity renderEnt = _eBuffer[i];
		if (!renderEnt.active) continue;

		/* grab the FINAL transform of the entity */
		vfTransform tFinal = *renderEnt.transform;

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
}

VFAPI void vfRenderBounds(void)
{
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

/* PHYSICS RELATED FUNCTIONS */
VFAPI void vfSetPhysicsState(int value)
{
	_pEnabled = value;
}

VFAPI void vfSetCollisionCallback(vfEntity* entity, ENTCOLCALLBACK callback)
{
	entity->collisionCallback = callback;
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
		pbuff = _tBuffer;
		break;

	case VF_BUFF_BOUND:
		pbuff = _bBuffer;
		break;

	case VF_BUFF_PARTICLE:
		pbuff = _pBuffer;
		break;

	case VF_BUFF_ENTITY:
		pbuff = _eBuffer;
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
	for (int i = 0; i < VF_MEMTANK_SIZE / 8; i++)
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
	if (size < 0 || size / 8 > VF_BUFFER_SIZE) return NULL;

	/* find lowest multiple of 8 which satisfies size */
	float fsize = (float)size;
	int sizeActual = (int)ceilf(fsize / 8);

	/* check for the impossible */
	if (sizeActual * 8 < size) sizeActual++;

	/* find index */
	int index = findFreeMTIndex(sizeActual);

	/* fail condition */
	if (index == -1) return NULL;

	/* fill field */
	for (int i = 0; i < sizeActual; i++)
		_mTField[index + i] = 1;

	/* check zero and return ptr */
	if (zero) ZeroMemory(_mTank + index, sizeActual * 8);
	return _mTank + index;
}

VFAPI int vfMTFree(void* ptr, int size, int zero)
{
	/* check for bad ptr */
	if (ptr == NULL || ptr < _mTank ||
		ptr > _mTank + (VF_MEMTANK_SIZE / 8)) return 0;

	/* find lowest multiple of 8 which satisfies size */
	float fsize = (float)size;
	int sizeActual = (int)ceilf(fsize / 8);

	/* check for the impossible */
	if (sizeActual * 8 < size) sizeActual++;

	/* free field */
	int startIndex = ((char*)ptr - _mTank) / 8;
	for (int i = 0; i < sizeActual; i++)
		_mTField[startIndex + i] = 0;

	/* zero */
	if (zero) ZeroMemory(ptr, sizeActual * 8);

	return 1;
}