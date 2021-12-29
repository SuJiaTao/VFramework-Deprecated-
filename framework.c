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

typedef unsigned char field;

/*========== INTERNAL RESOURCES ==========*/

static HANDLE _fThread; /* thread handle */
static HANDLE _heap; /* heap handle */
static HANDLE _mutex; /* buffer use mutex */
static unsigned int _sleepTime;
static int _pEnabled; /* physics toggle */

/* internal buffers */
static vfTransform* _tBuffer; static field* _tBufferField; static int _tBSize;
static vfTransform* _tFinalBuffer;
static vfParticle* _pBuffer; static field* _pBufferField; static int _pBSize;
static vfBound* _bBuffer; static field* _bBufferField; static int _bBSize;
static vfEntity* _eBuffer; static field* _eBufferField; static int _eBSize;

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

	/* empty buffers */
	for (int i = 0; i < VF_COLLISIONS_MAX; i++)
	{
		bQ.collisionData[i] = VECT(0, 0);
		bQ.collisionEdge[i] = VECT(0, 0);
		bQ.collisionTargetAverage[i] = VECT(0, 0);
	}
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
static inline int findBufferSpot(void** buffer, field** field, int* size, 
	size_t structSize)
{
	/* check for NULL */
	if (*buffer == NULL || *field == NULL) return -1;

	/* find empty spot within field */
	int i;
	int bSize = *size;
	for (i = 0; i < bSize; i++)
	{
		/* empty spot */
		if ((*field)[i] == 0) return i;
	}

	/* wait for REALLOC permission */
	int waitResult = WaitForSingleObject(_mutex,
		VF_MUTEX_DEADLOCK_INTERVAL);
	if (waitResult != WAIT_OBJECT_0)
	{
		wchar_t errBuff[255];
		swprintf(errBuff, 255, L"Error Code: %d", GetLastError());
		MessageBox(NULL, L"Thread deadlock in VFramework.dll",
			L"FATAL ERROR", MB_OK);
		MessageBox(NULL, errBuff, L"ERROR INFO", MB_OK);
		exit(1);
	}
	
	/* if loop exit, size too small */
	void* temp;
	bSize += VF_BUFFER_SIZE_INCREMENT;

	/* realloc buffer object */
	temp = HeapReAlloc(_heap, HEAP_ZERO_MEMORY,
		*buffer, bSize * structSize);
	if (temp == NULL) 
	{
		MessageBox(NULL, L"MEMORY FAILURE", L"FATAL ERROR", MB_OK);
		exit(1);
	}
	*buffer = temp;

	/* IF BUFFER OBJECT IS TRANSFORM BUFFER, REALLOC FINAL TBUFFER */
	if (buffer == &_tBuffer)
	{
		temp = HeapReAlloc(_heap, HEAP_ZERO_MEMORY, _tFinalBuffer,
			bSize * sizeof(vfTransform));
		if (temp == NULL)
		{
			MessageBox(NULL, L"MEMORY FAILURE", L"FATAL ERROR", MB_OK);
			exit(1);
		}
		_tFinalBuffer = temp;
	}

	/* IF BUFFER IS BOUND BUFFER, REALLOC BOUNDQUAD BUFFER */
	if (buffer = &_bBuffer)
	{
		temp = HeapReAlloc(_heap, HEAP_ZERO_MEMORY, _bqBuffer,
			bSize * sizeof(boundQuad));
		if (temp == NULL)
		{
			MessageBox(NULL, L"MEMORY FAILURE", L"FATAL ERROR", MB_OK);
			exit(1);
		}
		_bqBuffer = temp;
	}

	/* realloc buffer field */
	temp = HeapReAlloc(_heap, HEAP_ZERO_MEMORY,
		*field, bSize * sizeof(field));
	if (temp == NULL)
	{
		MessageBox(NULL, L"MEMORY FAILURE", L"FATAL ERROR", MB_OK);
		exit(1);
	}
	*field = temp;

	/* RELEASE MUTEX */
	if (!ReleaseMutex(_mutex))
	{
		wchar_t errBuffer[255];
		swprintf(errBuffer, 255, L"Mutex relase failed! Err code: %d",
			GetLastError());
		MessageBox(NULL, errBuffer, L"FATAL ERROR", MB_OK);
		exit(1);
	}

	/* update size and return */
	*size = bSize;
	return i;
}

/* ========== FINAL TRANSFORM HANDLING FUNCTION ========== */
static inline void updateFinalTransforms(void)
{
	/* ===== update FINAL transform objects ===== */
	for (int i = 0; i < _tBSize; i++)
	{
		/* check if empty */
		if (!_tBufferField[i]) continue;

		/* else, update final transform values */
		/* recall that there's an identical transform buffer */
		/* except all it's values correspond to their global value */
		/* based on their parent's values */
		vfTransform tValue = _tBuffer[i];
		vfHandle tParent = tValue.parent;
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
			r *= TFORM(tParent)->scale;
			theta += (TFORM(tParent)->rotation * degToRadians);

			/* convert back to cartesian */
			const float posX = r * cosf(theta);
			const float posY = r * sinf(theta);

			/* update tValue so that position member reflects GLOBAL */
			/* offset from PARENT POSITION rather than LOCAL */
			tValue.position = vfCreateVector(posX, posY);

			/* update all other members */

			/* update visual scale */
			tValue.position.x += TFORM(tParent)->position.x;
			tValue.position.y += TFORM(tParent)->position.y;
			tValue.scale *= TFORM(tParent)->scale;
			tValue.rotation += TFORM(tParent)->rotation;

			/* search for NEXT parent */
			tParent = TFORM(tParent)->parent;
		}

		/* set value */
		_tFinalBuffer[i] = tValue;
	}
}

/* ========= ENTITY VELOCITY DAMPENING FUNCTION ========== */
static inline void updateEntityVelocities(void)
{
	for (int i = 0; i < _eBSize; i++)
	{
		/* grab entity */
		if (!_eBufferField[i]) continue;

		vfEntity* ent = _eBuffer + i;

		/* if inactive, skip */
		if (!EBOUND(i)->active) continue;

		/* update transform members */
		vfPhysics* pObj = &(ent->physics);
		vfTransform* entityTransform = vfGetTransform(ent->transform);
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
	for (int i = 0; i < _bBSize; i++)
	{
		if (!_bBufferField[i]) continue;

		/* grab bound to convert */
		vfBound toConvert = _bBuffer[i];

		/* grab final transform */
		vfTransform tFinal = _tFinalBuffer[toConvert.body];

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
		for (int j = 0; j < VF_COLLISIONS_MAX; j++)
		{
			bQuad.collisionData[j] = VECT(0, 0);
		}

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
	if (!source->staticData.active) return 0; /* if not active, ignore */
	if (!target->staticData.active) return 0; /* if not active, ignore */

	/* grab the vector from target to source, this will be */
	/* handy later */
	vfVector avgVect = VECT(source->average.x - target->average.x,
		source->average.y - target->average.y);

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
		/* set collision physics value */
		target->collisionPhysics[target->collisions] =
			source->staticData.physics;

		/* check if target is static */
		if (!target->staticData.physics->moveable) return 1;

		/* check for NaN */
		if (isnan(smallestPVect.x)) smallestPVect.x = 0;
		if (isnan(smallestPVect.y)) smallestPVect.y = 0;

		/* get pushvector ratio */
		float massTotal = target->staticData.physics->mass +
			source->staticData.physics->mass;
		float massPercent = 1.0f - (target->staticData.physics->mass / massTotal);

		/* negative pushback vector */
		target->collisionData[target->collisions].x = -(smallestPVect.x * massPercent);
		target->collisionData[target->collisions].y = -(smallestPVect.y * massPercent);
		target->collisionEdge[target->collisions] = smallestEdgeVect;
		target->collisionTargetAverage[target->collisions] = source->average;
		target->collisions++;
	}

	return 1;
}

/* ======== COLLISION HANDLING FUNCTION ======== */
static inline void updateCollisions(void)
{
	/* GET COLLISION DATA */
	for (int i = 0; i < _bBSize; i++)
	{
		/* if not active, skip */
		if (!_bBufferField[i]) continue;
		if (!_bqBuffer[i].staticData.active) continue;

		for (int j = 0; j < _bBSize; j++)
		{
			if (!_bBufferField[j] || j == i) continue;
			collisionCheck(_bqBuffer + i, _bqBuffer + j);
		}
	}

	/* ACCUMULATE COLLISION DATA */
	for (int i = 0; i < _bBSize; i++)
	{
		/* if not active, skip */
		if (!_bBufferField[i]) continue;
		if (!_bqBuffer[i].staticData.active) continue;

		for (int k = 0; k < _bqBuffer[i].collisions; k++)
		{
			if (isnan(_bqBuffer[i].collisionData[k].x) ||
				isnan(_bqBuffer[i].collisionData[k].y)) continue;
			_bqBuffer[i].collisionAccumulator.x += _bqBuffer[i].collisionData[k].x;
			_bqBuffer[i].collisionAccumulator.y += _bqBuffer[i].collisionData[k].y;
		}
	}

	/* APPLY COLLISION DATA */
	for (int i = 0; i < _bBSize; i++)
	{
		/* if not used or not active, skip */
		if (!_bBufferField[i]) continue;
		if (!_bqBuffer[i].staticData.active) continue;

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
			TFORM(_bBuffer[i].body)->position.x += pushBack.x;
			TFORM(_bBuffer[i].body)->position.y += pushBack.y;
		}

	} /* COLLISION APPLICATION LOOP END */
}

/* ========== ENERGY TRANSFER SIMULATION FUNCTION ========== */
static inline void updateCollisionVelocities(void)
{
	/* loop through all objects with collisions */
	for (int i = 0; i < _bBSize; i++)
	{
		if (!_bBufferField[i]) continue; /* check active */
		if (!_bqBuffer[i].collisions) continue; /* check collided */
		if (!_bqBuffer[i].staticData.active) continue; /* inactive; avoid */

		/* loop through all collision data */
		for (int j = 0; j < _bqBuffer[i].collisions; j++)
		{
			/* get physics and quad data */
			boundQuad* currentQuad = _bqBuffer + i;
			vfPhysics* currentPhysics = currentQuad->staticData.physics;
			vfPhysics targetPhysics = *currentQuad->collisionPhysics[j];

			/* get mass ratio */
			float massTotal = currentPhysics->mass + targetPhysics.mass;
			float currentRatio = currentPhysics->mass / massTotal;
			float targetRatio = targetPhysics.mass / massTotal;

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
			if (vectorDotProduct(VECT(weightedVX, weightedVY), offsetVec) < 0)
			{
				currentPhysics->velocity = VECT(weightedVX, weightedVY);
			}

			/* === BOUNCE === */
			/* only bounce if current velocity is headed towards target */
			if (vectorDotProduct(currentPhysics->velocity, offsetVec) > 0)
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


			/* get inital and predicted velocity */
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
	} /* END COLLISION OBJECT LOOP */
}

/* ===== MODULE MAIN FUNCTION ===== */
static DWORD WINAPI vfMain(void* params)
{
	while (TRUE)
	{
		/* sleep (optional) */
		if (_sleepTime)
		{
			Sleep(_sleepTime);
		}

		/* WAIT FOR BUFFER OWNERSHIP */
		int mutStatus = WaitForSingleObject(_mutex, VF_MUTEX_DEADLOCK_INTERVAL);
		if (mutStatus != WAIT_OBJECT_0)
		{
			wchar_t errBuff[255];
			swprintf(errBuff, 255, L"Error Code: %d", GetLastError());
			MessageBox(NULL, L"[vfMain] Thread deadlock in VFramework.dll",
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
		sizeof(vfTransform) * VF_BUFFER_SIZE_INIT);
	_tBufferField = HeapAlloc(_heap, HEAP_ZERO_MEMORY,
		sizeof(field) * VF_BUFFER_SIZE_INIT);
	_tBSize = VF_BUFFER_SIZE_INIT;

	/* INIT FINAL TRANSFORM BUFFER */
	_tFinalBuffer = HeapAlloc(_heap, HEAP_ZERO_MEMORY,
		sizeof(vfTransform) * VF_BUFFER_SIZE_INIT);

	/* INIT PARTICLE BUFFER */
	_pBuffer = HeapAlloc(_heap, HEAP_ZERO_MEMORY,
		sizeof(vfParticle) * VF_BUFFER_SIZE_INIT);
	_pBufferField = HeapAlloc(_heap, HEAP_ZERO_MEMORY,
		sizeof(field) * VF_BUFFER_SIZE_INIT);
	_pBSize = VF_BUFFER_SIZE_INIT;

	/* INIT BOUND BUFFER */
	_bBuffer = HeapAlloc(_heap, HEAP_ZERO_MEMORY,
		sizeof(vfBound) * VF_BUFFER_SIZE_INIT);
	_bBufferField = HeapAlloc(_heap, HEAP_ZERO_MEMORY,
		sizeof(field) * VF_BUFFER_SIZE_INIT);
	_bBSize = VF_BUFFER_SIZE_INIT;

	/* INIT BOUNDQUAD BUFFER */
	_bqBuffer = HeapAlloc(_heap, HEAP_ZERO_MEMORY,
		sizeof(boundQuad) * VF_BUFFER_SIZE_INIT);

	/* INIT ENTITY BUFFER */
	_eBuffer = HeapAlloc(_heap, HEAP_ZERO_MEMORY,
		sizeof(vfEntity) * VF_BUFFER_SIZE_INIT);
	_eBufferField = HeapAlloc(_heap, HEAP_ZERO_MEMORY,
		sizeof(field) * VF_BUFFER_SIZE_INIT);
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

VFAPI vfHandle vfCreateTransformv(vfVector vector)
{
	/* find transform buffer spot */
	int tIndex = findBufferSpot(&_tBuffer, &_tBufferField, &_tBSize,
		sizeof(vfTransform));
	_tBufferField[tIndex] = 1;

	/* set values */
	vfTransform* rTransform = _tBuffer + tIndex;
	rTransform->position = vector;
	rTransform->parent = VF_NOPARENT;

	return tIndex;
}

VFAPI vfHandle vfCreateTransforma(vfVector vector, float rotation,
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
	rTransform->parent = VF_NOPARENT;

	return tIndex;
}

VFAPI vfHandle vfCreateTransformp(vfHandle parent)
{
	/* find transform buffer spot */
	int tIndex = findBufferSpot(&_tBuffer, &_tBufferField, &_tBSize,
		sizeof(vfTransform));
	_tBufferField[tIndex] = 1;

	/* set values */
	vfTransform* rTransform = _tBuffer + tIndex;
	rTransform->parent = parent;

	return tIndex;
}

VFAPI vfHandle vfCreateBoundt(vfHandle body)
{
	/* find bound buffer spot */
	int bIndex = findBufferSpot(&_bBuffer, &_bBufferField, &_bBSize,
		sizeof(vfBound));
	_bBufferField[bIndex] = 1;

	/* set value */
	vfBound* rBound = _bBuffer + bIndex;
	rBound->body = body;
	rBound->active = TRUE;
	rBound->physics = NULL;

	return bIndex;
}

VFAPI vfHandle vfCreateBounda(vfHandle body, vfVector position,
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
	rBound->active = TRUE;
	rBound->physics = NULL;

	return bIndex;
}

VFAPI vfHandle vfCreateParticlet(vfHandle transform)
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

	return pIndex;
}

VFAPI vfHandle vfCreateParticlea(vfHandle transform, vgTexture texture,
	vgShape shape, unsigned char layer)
{
	/* get free spot */
	int pIndex = findBufferSpot(&_pBuffer, &_pBufferField, &_pBSize,
		sizeof(vfParticle));
	_pBufferField[pIndex] = 1;

	/* set values */
	vfParticle* rParticle = _pBuffer + pIndex;
	rParticle->layer = layer;
	rParticle->filter = vfCreateColor(255, 255, 255, 255);
	rParticle->texture = texture;
	rParticle->shape = shape;
	rParticle->transform = transform;

	return pIndex;
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
	
	return rPhys;
}

VFAPI vfHandle vfCreateEntity(unsigned char layer, vgShape shape,
	vgTexture texture, vfPhysics physics, vfVector boundPosition,
	vfVector boundDimensions)
{
	/* find free spot */
	int eIndex = findBufferSpot(&_eBuffer, &_eBufferField, &_eBSize,
		sizeof(vfEntity));
	_eBufferField[eIndex] = 1;
	
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
	vfBound* entBounds = vfGetBound(rEnt->bounds);
	entBounds->physics = &(rEnt->physics);
	
	return eIndex;
}

/* STRUCT DESTRUCTION FUNCTIONS */

VFAPI void vfDestroyTransform(vfHandle transform)
{
	_tBufferField[transform] = 0;
}

VFAPI void vfDestroyBound(vfHandle bound)
{
	_bBufferField[bound] = 0;
}

VFAPI void vfDestroyParticle(vfHandle particle)
{
	_pBufferField[particle] = 0;
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

VFAPI vfTransform* vfGetTransformEnt(vfHandle hndl)
{
	vfEntity ent = _eBuffer[hndl];
	return vfGetTransform(ent.transform);
}

VFAPI vfBound* vfGetBound(vfHandle hndl)
{
	return _bBuffer + hndl;
}

VFAPI vfParticle* vfGetParticle(vfHandle hndl)
{
	return _pBuffer + hndl;
}

VFAPI vfEntity* vfGetEntity(vfHandle hndl)
{
	return _eBuffer + hndl;
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
		vfTransform* rTransform = TFORM(render.transform);
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
		if (_eBufferField[i] == 0) continue;

		vfEntity renderEnt = _eBuffer[i];
		if (!renderEnt.active) continue;

		/* grab the FINAL transform of the entity */
		vfTransform tFinal = _tFinalBuffer[renderEnt.transform];

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
	/* wait for all bounds to be properly updated */
	int wResult = WaitForSingleObject(_mutex, VF_MUTEX_RENDERBOUNDS_INTERVAL);
	if (wResult != WAIT_OBJECT_0) /* not finished updating */
	{
		return;
	}
	else
	{
		ReleaseMutex(_mutex); /* release captured mutex */
	}

	/* render bound */
	vgLineSize(2.5f);
	for (int i = 0; i < _bBSize; i++)
	{

		if (!_bBufferField[i]) continue;

		boundQuad bQ = _bqBuffer[i];
		
		if (bQ.collisions > 0)
		{
			vgColor3(255, 128, 64);
		}
		else
		{
			vgColor3(64, 200, 64);
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
	}
	vgLineSize(1.0f);
}

/* PHYSICS RELATED FUNCTIONS */
VFAPI void vfSetPhysicsState(int value)
{
	_pEnabled = value;
}
