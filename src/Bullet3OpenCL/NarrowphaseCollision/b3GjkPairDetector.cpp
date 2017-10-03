/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "b3GjkPairDetector.h"
#include "Bullet3Common/b3Transform.h"
#include "b3VoronoiSimplexSolver.h"
#include "Bullet3Collision/NarrowPhaseCollision/shared/b3ConvexPolyhedronData.h"
#include "b3VectorFloat4.h"
#include "b3GjkEpa.h"
#include "b3SupportMappings.h"

b3GjkPairDetector::b3GjkPairDetector(b3VoronoiSimplexSolver* simplexSolver,b3GjkEpaSolver2*	penetrationDepthSolver)
:m_cachedSeparatingAxis(b3MakeVector3(b3Scalar(0.),b3Scalar(-1.),b3Scalar(0.))),
m_penetrationDepthSolver(penetrationDepthSolver),
m_simplexSolver(simplexSolver),
m_ignoreMargin(false),
m_lastUsedMethod(-1),
m_catchDegeneracies(1),
m_fixContactNormalDirection(1)
{
}

bool getClosestPoints(b3GjkPairDetector* gjkDetector, const b3Transform&	transA, const b3Transform&	transB,
	const b3ConvexPolyhedronData& hullA, const b3ConvexPolyhedronData& hullB, 
	const b3AlignedObjectArray<b3Vector3>& verticesA,
	const b3AlignedObjectArray<b3Vector3>& verticesB,
	b3Scalar maximumDistanceSquared,
	b3Vector3& resultSepNormal,
	float& resultSepDistance,
	b3Vector3& resultPointOnB)
{
	b3Vector3	normalInB= b3MakeVector3(b3Scalar(0.),b3Scalar(0.),b3Scalar(0.));

	b3Transform localTransA = transA;
	b3Transform localTransB = transB;
	
	b3Vector3 positionOffset = b3MakeVector3(0,0,0);// = (localTransA.getOrigin() + localTransB.getOrigin()) * b3Scalar(0.5);
	localTransA.getOrigin() -= positionOffset;
	localTransB.getOrigin() -= positionOffset;

	bool checkSimplex = false;

		for ( ; ; )
		{

			b3Vector3 seperatingAxisInA;
			b3Vector3 seperatingAxisInB;

			b3Vector3 pInA = localGetSupportVertexWithoutMargin(seperatingAxisInA,&hullA,verticesA);
			b3Vector3 qInB = localGetSupportVertexWithoutMargin(seperatingAxisInB,&hullB,verticesB);

			b3Vector3  pWorld = localTransA(pInA);	
			b3Vector3  qWorld = localTransB(qInB);




			b3Vector3 w;

			// potential exit, they don't overlap

			//add current vertex to simplex
			gjkDetector->m_simplexSolver->addVertex(w, pWorld, qWorld);
			b3Vector3 newCachedSeparatingAxis;

			//calculate the closest point to the origin (update vector v)
			if (!gjkDetector->m_simplexSolver->closest(newCachedSeparatingAxis))
			{
				checkSimplex = true;
				break;
			}

			gjkDetector->m_cachedSeparatingAxis = newCachedSeparatingAxis;
		}

		if (checkSimplex)
		{
			normalInB = gjkDetector->m_cachedSeparatingAxis;
			b3Scalar lenSqr =gjkDetector->m_cachedSeparatingAxis.length2();
			
			{
				b3Scalar rlen = b3Scalar(1.) / b3Sqrt(lenSqr );
				normalInB *= rlen; //normalize
			}
		}

		resultSepNormal = normalInB;
		return true;
}





