#include "CollisionDetection.h"
#include "CollisionVolume.h"
#include "AABBVolume.h"
#include "OBBVolume.h"
#include "SphereVolume.h"
#include "Window.h"
#include "Maths.h"
#include "Debug.h"
#include <algorithm>
#include <limits>

using namespace NCL;
using namespace NCL::Maths;

bool CollisionDetection::RayPlaneIntersection(const Ray&r, const Plane&p, RayCollision& collisions) {
	float ln = Vector::Dot(p.GetNormal(), r.GetDirection());

	if (ln == 0.0f) {
		return false; //direction vectors are perpendicular! 1
	}
	
	Vector3 planePoint = p.GetPointOnPlane();

	Vector3 pointDir = planePoint - r.GetPosition();

	float d = Vector::Dot(pointDir, p.GetNormal()) / ln;

	collisions.collidedAt = r.GetPosition() + (r.GetDirection() * d);

	return true;
}

bool CollisionDetection::RayIntersection(const Ray& r,GameObject& object, RayCollision& collision) {
	bool hasCollided = false;

	const Transform& worldTransform = object.GetTransform();
	const CollisionVolume* volume	= object.GetBoundingVolume();

	if (!volume) {
		return false;
	}

	switch (volume->type) {
		case VolumeType::AABB:		hasCollided = RayAABBIntersection(r, worldTransform, (const AABBVolume&)*volume	, collision); break;
		case VolumeType::OBB:		hasCollided = RayOBBIntersection(r, worldTransform, (const OBBVolume&)*volume	, collision); break;
		case VolumeType::Sphere:	hasCollided = RaySphereIntersection(r, worldTransform, (const SphereVolume&)*volume	, collision); break;

		case VolumeType::Capsule:	hasCollided = RayCapsuleIntersection(r, worldTransform, (const CapsuleVolume&)*volume, collision); break;
	}

	return hasCollided;
}

bool CollisionDetection::RayBoxIntersection(const Ray&r, const Vector3& boxPos, const Vector3& boxSize, RayCollision& collision) {
	// Slab method for axis aligned box (boxSize = half extents)1
	Vector3 rayDir = r.GetDirection();
	Vector3 rayPos = r.GetPosition();

	auto SafeInv = [](float v) {
		const float eps = 1e-8f;
		if (fabs(v) < eps) {
			return std::numeric_limits<float>::infinity();
		}
		return 1.0f / v;
	};

	Vector3 invDir = Vector3(SafeInv(rayDir.x), SafeInv(rayDir.y), SafeInv(rayDir.z));

	Vector3 tMin = (boxPos - boxSize - rayPos) * invDir;
	Vector3 tMax = (boxPos + boxSize - rayPos) * invDir;

	// Handle negative directions
	if (tMin.x > tMax.x) std::swap(tMin.x, tMax.x);
	if (tMin.y > tMax.y) std::swap(tMin.y, tMax.y);
	if (tMin.z > tMax.z) std::swap(tMin.z, tMax.z);

	float tNear = std::max(tMin.x, std::max(tMin.y, tMin.z));
	float tFar  = std::min(tMax.x, std::min(tMax.y, tMax.z));

	if (tNear > tFar || tFar < 0.0f) {
		return false;
	}

	float tHit = tNear >= 0.0f ? tNear : tFar;
	collision.rayDistance = tHit;
	collision.collidedAt  = rayPos + rayDir * tHit;
	return true;
}

bool CollisionDetection::RayAABBIntersection(const Ray&r, const Transform& worldTransform, const AABBVolume& volume, RayCollision& collision) {
	Vector3 boxPos  = worldTransform.GetPosition();
	Vector3 halfDim = volume.GetHalfDimensions();
	Vector3 worldScale = worldTransform.GetScale();
	halfDim = Vector3(halfDim.x * worldScale.x, halfDim.y * worldScale.y, halfDim.z * worldScale.z);
	return RayBoxIntersection(r, boxPos, halfDim, collision);
}

bool CollisionDetection::RayOBBIntersection(const Ray&r, const Transform& worldTransform, const OBBVolume& volume, RayCollision& collision) {
	Vector3 boxSize = volume.GetHalfDimensions();

	Quaternion q = worldTransform.GetOrientation();
	Matrix3 invOrientation = Quaternion::RotationMatrix<Matrix3>(q.Conjugate());
	Matrix3 orientation     = Quaternion::RotationMatrix<Matrix3>(q);

	Vector3 localRayPos = invOrientation * (r.GetPosition() - worldTransform.GetPosition());
	Vector3 localRayDir = invOrientation * r.GetDirection();

	Ray localRay(localRayPos, localRayDir);
	RayCollision localCollision;

	bool collided = RayBoxIntersection(localRay, Vector3(), boxSize, localCollision);
	if (collided) {
		collision.rayDistance = localCollision.rayDistance;
		collision.collidedAt  = worldTransform.GetPosition() + orientation * localCollision.collidedAt;
	}
	return collided;
}

bool CollisionDetection::RaySphereIntersection(const Ray&r, const Transform& worldTransform, const SphereVolume& volume, RayCollision& collision) {
	Vector3 spherePos   = worldTransform.GetPosition();
	Vector3 scale       = worldTransform.GetScale();
	float   sphereRadius = volume.GetRadius() * std::max(scale.x, std::max(scale.y, scale.z));

	Vector3 dir = r.GetDirection();
	Vector3 diff = spherePos - r.GetPosition();

	float t = Vector::Dot(diff, dir);
	if (t < 0.0f) {
		return false; // sphere is behind ray
	}

	Vector3 closestPoint = r.GetPosition() + dir * t;
	float distSq = Vector::LengthSquared(spherePos - closestPoint);
	float radiusSq = sphereRadius * sphereRadius;

	if (distSq > radiusSq) {
		return false;
	}

	float thc = sqrt(radiusSq - distSq);
	float tHit = t - thc;
	if (tHit < 0.0f) {
		tHit = t + thc; // inside sphere case
	}

	collision.rayDistance = tHit;
	collision.collidedAt  = r.GetPosition() + dir * tHit;
	return true;
}

bool CollisionDetection::RayCapsuleIntersection(const Ray& r, const Transform& worldTransform, const CapsuleVolume& volume, RayCollision& collision) {
	// Treat capsule as segment with spheres on ends
	Quaternion q = worldTransform.GetOrientation();
	Matrix3 orientation = Quaternion::RotationMatrix<Matrix3>(q);

	Vector3 up = orientation * Vector3(0, 1, 0);
	Vector3 p0 = worldTransform.GetPosition() - up * volume.GetHalfHeight();
	Vector3 p1 = worldTransform.GetPosition() + up * volume.GetHalfHeight();

	Vector3 d = p1 - p0;           // segment direction
	Vector3 m = r.GetPosition() - p0;
	Vector3 n = r.GetDirection();

	float md = Vector::Dot(m, d);
	float nd = Vector::Dot(n, d);
	float dd = Vector::Dot(d, d);

	float t = 0.0f;
	float s = 0.0f;

	const float kEpsilon = 1e-6f;
	if (dd > kEpsilon) {
		s = md / dd;
	}

	// Clamp s to segment
	s = std::min(1.0f, std::max(0.0f, s));

	// Project ray to closest point on segment
	Vector3 closestOnSeg = p0 + d * s;
	Vector3 segToRayOrigin = r.GetPosition() - closestOnSeg;
	float proj = Vector::Dot(segToRayOrigin, n);
	t = -proj;

	if (t < 0.0f) {
		return false;
	}

	Vector3 closestPoint = r.GetPosition() + n * t;
	float distSq = Vector::LengthSquared(closestPoint - closestOnSeg);
	float radiusSq = volume.GetRadius() * volume.GetRadius();

	if (distSq > radiusSq) {
		return false;
	}

	collision.rayDistance = t;
	collision.collidedAt  = closestPoint;
	return true;
}

bool CollisionDetection::ObjectIntersection(GameObject* a, GameObject* b, CollisionInfo& collisionInfo) {
	const CollisionVolume* volA = a->GetBoundingVolume();
	const CollisionVolume* volB = b->GetBoundingVolume();

	if (!volA || !volB) {
		return false;
	}

	collisionInfo.a = a;
	collisionInfo.b = b;

	Transform& transformA = a->GetTransform();
	Transform& transformB = b->GetTransform();

	VolumeType pairType = (VolumeType)((int)volA->type | (int)volB->type);

	//Two AABBs
	if (pairType == VolumeType::AABB) {
		return AABBIntersection((AABBVolume&)*volA, transformA, (AABBVolume&)*volB, transformB, collisionInfo);
	}
	//Two Spheres
	if (pairType == VolumeType::Sphere) {
		return SphereIntersection((SphereVolume&)*volA, transformA, (SphereVolume&)*volB, transformB, collisionInfo);
	}
	//Two OBBs
	if (pairType == VolumeType::OBB) {
		return OBBIntersection((OBBVolume&)*volA, transformA, (OBBVolume&)*volB, transformB, collisionInfo);
	}
	//Two Capsules

	//AABB vs Sphere pairs
	if (volA->type == VolumeType::AABB && volB->type == VolumeType::Sphere) {
		return AABBSphereIntersection((AABBVolume&)*volA, transformA, (SphereVolume&)*volB, transformB, collisionInfo);
	}
	if (volA->type == VolumeType::Sphere && volB->type == VolumeType::AABB) {
		collisionInfo.a = b;
		collisionInfo.b = a;
		return AABBSphereIntersection((AABBVolume&)*volB, transformB, (SphereVolume&)*volA, transformA, collisionInfo);
	}

	//OBB vs sphere pairs
	if (volA->type == VolumeType::OBB && volB->type == VolumeType::Sphere) {
		return OBBSphereIntersection((OBBVolume&)*volA, transformA, (SphereVolume&)*volB, transformB, collisionInfo);
	}
	if (volA->type == VolumeType::Sphere && volB->type == VolumeType::OBB) {
		collisionInfo.a = b;
		collisionInfo.b = a;
		return OBBSphereIntersection((OBBVolume&)*volB, transformB, (SphereVolume&)*volA, transformA, collisionInfo);
	}

	//Capsule vs other interactions
	if (volA->type == VolumeType::Capsule && volB->type == VolumeType::Sphere) {
		return SphereCapsuleIntersection((CapsuleVolume&)*volA, transformA, (SphereVolume&)*volB, transformB, collisionInfo);
	}
	if (volA->type == VolumeType::Sphere && volB->type == VolumeType::Capsule) {
		collisionInfo.a = b;
		collisionInfo.b = a;
		return SphereCapsuleIntersection((CapsuleVolume&)*volB, transformB, (SphereVolume&)*volA, transformA, collisionInfo);
	}

	if (volA->type == VolumeType::Capsule && volB->type == VolumeType::AABB) {
		return AABBCapsuleIntersection((CapsuleVolume&)*volA, transformA, (AABBVolume&)*volB, transformB, collisionInfo);
	}
	if (volB->type == VolumeType::Capsule && volA->type == VolumeType::AABB) {
		collisionInfo.a = b;
		collisionInfo.b = a;
		return AABBCapsuleIntersection((CapsuleVolume&)*volB, transformB, (AABBVolume&)*volA, transformA, collisionInfo);
	}

	return false;
}

bool CollisionDetection::AABBTest(const Vector3& posA, const Vector3& posB, const Vector3& halfSizeA, const Vector3& halfSizeB) {
	Vector3 delta = posB - posA;
	Vector3 totalSize = halfSizeA + halfSizeB;

	if (abs(delta.x) < totalSize.x &&
		abs(delta.y) < totalSize.y &&
		abs(delta.z) < totalSize.z) {
		return true;
	}
	return false;
}

//AABB/AABB Collisions
bool CollisionDetection::AABBIntersection(const AABBVolume& volumeA, const Transform& worldTransformA,
	const AABBVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {
	Vector3 boxAPos = worldTransformA.GetPosition();
	Vector3 boxBPos = worldTransformB.GetPosition();

	Vector3 halfA = volumeA.GetHalfDimensions();
	Vector3 halfB = volumeB.GetHalfDimensions();

	// apply world scale
	Vector3 scaleA = worldTransformA.GetScale();
	Vector3 scaleB = worldTransformB.GetScale();
	halfA = Vector3(halfA.x * scaleA.x, halfA.y * scaleA.y, halfA.z * scaleA.z);
	halfB = Vector3(halfB.x * scaleB.x, halfB.y * scaleB.y, halfB.z * scaleB.z);

	if (!AABBTest(boxAPos, boxBPos, halfA, halfB)) {
		return false;
	}

	Vector3 delta = boxBPos - boxAPos;
	Vector3 total = halfA + halfB;

	// penetration on each axis
	float px = total.x - std::abs(delta.x);
	float py = total.y - std::abs(delta.y);
	float pz = total.z - std::abs(delta.z);

	// choose smallest penetration axis for normal
	Vector3 normal;
	float penetration = px;
	normal = Vector3((delta.x < 0) ? -1.0f : 1.0f, 0, 0);

	if (py < penetration) {
		penetration = py;
		normal = Vector3(0, (delta.y < 0) ? -1.0f : 1.0f, 0);
	}
	if (pz < penetration) {
		penetration = pz;
		normal = Vector3(0, 0, (delta.z < 0) ? -1.0f : 1.0f);
	}

	Vector3 contactPoint = boxAPos + normal * (halfA);
	collisionInfo.AddContactPoint(contactPoint - boxAPos, contactPoint - boxBPos, normal, penetration);
	collisionInfo.framesLeft = 1;
	return true;
}

//Sphere / Sphere Collision
bool CollisionDetection::SphereIntersection(const SphereVolume& volumeA, const Transform& worldTransformA,
	const SphereVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {
	Vector3 delta = worldTransformB.GetPosition() - worldTransformA.GetPosition();

	Vector3 scaleA = worldTransformA.GetScale();
	Vector3 scaleB = worldTransformB.GetScale();

	float radiusA = volumeA.GetRadius() * std::max(scaleA.x, std::max(scaleA.y, scaleA.z));
	float radiusB = volumeB.GetRadius() * std::max(scaleB.x, std::max(scaleB.y, scaleB.z));

	float distSq = Vector::LengthSquared(delta);
	float radiusSum = radiusA + radiusB;

	if (distSq > radiusSum * radiusSum) {
		return false;
	}

	float dist = sqrt(std::max(distSq, 0.0001f));
	Vector3 normal = delta / dist;
	float penetration = radiusSum - dist;

	Vector3 localA = normal * radiusA;
	Vector3 localB = -normal * radiusB;
	collisionInfo.AddContactPoint(localA, localB, normal, penetration);
	collisionInfo.framesLeft = 1;
	return true;
}

//AABB - Sphere Collision
bool CollisionDetection::AABBSphereIntersection(const AABBVolume& volumeA, const Transform& worldTransformA,
	const SphereVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {
	Vector3 boxPos = worldTransformA.GetPosition();
	Vector3 halfSize = volumeA.GetHalfDimensions();
	Vector3 boxScale = worldTransformA.GetScale();
	halfSize = Vector3(halfSize.x * boxScale.x, halfSize.y * boxScale.y, halfSize.z * boxScale.z);

	Vector3 spherePos = worldTransformB.GetPosition();
	Vector3 sphereScale = worldTransformB.GetScale();
	float sphereRadius = volumeB.GetRadius() * std::max(sphereScale.x, std::max(sphereScale.y, sphereScale.z));

	Vector3 boxMin = boxPos - halfSize;
	Vector3 boxMax = boxPos + halfSize;

	Vector3 closestPoint = Vector3(
		std::max(boxMin.x, std::min(spherePos.x, boxMax.x)),
		std::max(boxMin.y, std::min(spherePos.y, boxMax.y)),
		std::max(boxMin.z, std::min(spherePos.z, boxMax.z))
	);

	Vector3 delta = spherePos - closestPoint;
	float distSq = Vector::LengthSquared(delta);
	if (distSq > sphereRadius * sphereRadius) {
		return false;
	}

	// Handle case where sphere center is inside the box (delta ~ 0)
	Vector3 normal;
	float penetration = 0.0f;
	if (distSq < 1e-6f) {
		// compute smallest distance to each face to push sphere out
		float dxMin = (spherePos.x - boxMin.x);
		float dxMax = (boxMax.x - spherePos.x);
		float dyMin = (spherePos.y - boxMin.y);
		float dyMax = (boxMax.y - spherePos.y);
		float dzMin = (spherePos.z - boxMin.z);
		float dzMax = (boxMax.z - spherePos.z);

		float minDist = dxMin;
		normal = Vector3(-1, 0, 0);

		if (dxMax < minDist) { minDist = dxMax; normal = Vector3(1, 0, 0); }
		if (dyMin < minDist) { minDist = dyMin; normal = Vector3(0, -1, 0); }
		if (dyMax < minDist) { minDist = dyMax; normal = Vector3(0, 1, 0); }
		if (dzMin < minDist) { minDist = dzMin; normal = Vector3(0, 0, -1); }
		if (dzMax < minDist) { minDist = dzMax; normal = Vector3(0, 0, 1); }

		penetration = sphereRadius - minDist;
		closestPoint = spherePos + normal * (sphereRadius - penetration);
	}
	else {
		float dist = sqrt(distSq);
		normal = delta / dist;
		penetration = sphereRadius - dist;
	}

	collisionInfo.AddContactPoint(closestPoint - boxPos, closestPoint - spherePos, normal, penetration);
	collisionInfo.framesLeft = 1;
	return true;
}

bool  CollisionDetection::OBBSphereIntersection(const OBBVolume& volumeA, const Transform& worldTransformA,
	const SphereVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {
	Vector3 boxPos = worldTransformA.GetPosition();
	Vector3 halfSize = volumeA.GetHalfDimensions();
	Vector3 boxScale = worldTransformA.GetScale();
	halfSize = Vector3(halfSize.x * boxScale.x, halfSize.y * boxScale.y, halfSize.z * boxScale.z);

	Quaternion q = worldTransformA.GetOrientation();
	Matrix3 invOrientation = Quaternion::RotationMatrix<Matrix3>(q.Conjugate());
	Matrix3 orientation = Quaternion::RotationMatrix<Matrix3>(q);

	Vector3 spherePosWorld = worldTransformB.GetPosition();
	Vector3 spherePosLocal = invOrientation * (spherePosWorld - boxPos);

	Vector3 sphereScale = worldTransformB.GetScale();
	float sphereRadius = volumeB.GetRadius() * std::max(sphereScale.x, std::max(sphereScale.y, sphereScale.z));

	Vector3 localClosest = Vector3(
		std::max(-halfSize.x, std::min(spherePosLocal.x, halfSize.x)),
		std::max(-halfSize.y, std::min(spherePosLocal.y, halfSize.y)),
		std::max(-halfSize.z, std::min(spherePosLocal.z, halfSize.z))
	);

	Vector3 localDelta = spherePosLocal - localClosest;
	float distSq = Vector::LengthSquared(localDelta);
	if (distSq > sphereRadius * sphereRadius) {
		return false;
	}

	float dist = sqrt(std::max(distSq, 0.0001f));
	Vector3 normalLocal = localDelta / dist;
	Vector3 normalWorld = orientation * normalLocal;

	Vector3 worldClosest = boxPos + orientation * localClosest;
	float penetration = sphereRadius - dist;

	collisionInfo.AddContactPoint(worldClosest - boxPos, worldClosest - spherePosWorld, normalWorld, penetration);
	collisionInfo.framesLeft = 1;
	return true;
}

bool CollisionDetection::AABBCapsuleIntersection(
	const CapsuleVolume& volumeA, const Transform& worldTransformA,
	const AABBVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {
	// Treat capsule as sphere along its axis, find closest point on segment to AABB
	Vector3 boxPos = worldTransformB.GetPosition();
	Vector3 halfSize = volumeB.GetHalfDimensions();
	Vector3 boxScale = worldTransformB.GetScale();
	halfSize = Vector3(halfSize.x * boxScale.x, halfSize.y * boxScale.y, halfSize.z * boxScale.z);

	Quaternion q = worldTransformA.GetOrientation();
	Matrix3 orientation = Quaternion::RotationMatrix<Matrix3>(q);
	Vector3 up = orientation * Vector3(0, 1, 0);

	Vector3 capsuleCenter = worldTransformA.GetPosition();
	Vector3 p0 = capsuleCenter - up * volumeA.GetHalfHeight();
	Vector3 p1 = capsuleCenter + up * volumeA.GetHalfHeight();

	Vector3 segment = p1 - p0;
	Vector3 boxMin = boxPos - halfSize;
	Vector3 boxMax = boxPos + halfSize;

	// closest point on segment to box center
	Vector3 boxCenter = boxPos;
	float t = Vector::Dot(boxCenter - p0, segment) / Vector::LengthSquared(segment);
	t = std::max(0.0f, std::min(1.0f, t));
	Vector3 closestOnSegment = p0 + segment * t;

	// closest point on box to that segment point
	Vector3 closestOnBox = Vector3(
		std::max(boxMin.x, std::min(closestOnSegment.x, boxMax.x)),
		std::max(boxMin.y, std::min(closestOnSegment.y, boxMax.y)),
		std::max(boxMin.z, std::min(closestOnSegment.z, boxMax.z))
	);

	Vector3 delta = closestOnSegment - closestOnBox;
	float distSq = Vector::LengthSquared(delta);
	float radiusSq = volumeA.GetRadius() * volumeA.GetRadius();

	if (distSq > radiusSq) {
		return false;
	}

	float dist = sqrt(std::max(distSq, 0.0001f));
	Vector3 normal = delta / dist;
	float penetration = volumeA.GetRadius() - dist;
	collisionInfo.AddContactPoint(closestOnBox - p0, closestOnBox - boxPos, normal, penetration);
	collisionInfo.framesLeft = 1;
	return true;
}

bool CollisionDetection::SphereCapsuleIntersection(
	const CapsuleVolume& volumeA, const Transform& worldTransformA,
	const SphereVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {
	Quaternion q = worldTransformA.GetOrientation();
	Matrix3 orientation = Quaternion::RotationMatrix<Matrix3>(q);
	Vector3 up = orientation * Vector3(0, 1, 0);

	Vector3 capsuleCenter = worldTransformA.GetPosition();
	Vector3 p0 = capsuleCenter - up * volumeA.GetHalfHeight();
	Vector3 p1 = capsuleCenter + up * volumeA.GetHalfHeight();

	Vector3 segment = p1 - p0;
	Vector3 spherePos = worldTransformB.GetPosition();

	float t = Vector::Dot(spherePos - p0, segment) / Vector::LengthSquared(segment);
	t = std::max(0.0f, std::min(1.0f, t));
	Vector3 closestOnSegment = p0 + segment * t;

	Vector3 delta = spherePos - closestOnSegment;
	float distSq = Vector::LengthSquared(delta);

	Vector3 sphereScale = worldTransformB.GetScale();
	float sphereRadius = volumeB.GetRadius() * std::max(sphereScale.x, std::max(sphereScale.y, sphereScale.z));
	float combinedRadius = sphereRadius + volumeA.GetRadius();

	if (distSq > combinedRadius * combinedRadius) {
		return false;
	}

	float dist = sqrt(std::max(distSq, 0.0001f));
	Vector3 normal = delta / dist;
	float penetration = combinedRadius - dist;

	collisionInfo.AddContactPoint(closestOnSegment - capsuleCenter, spherePos - spherePos, normal, penetration);
	collisionInfo.framesLeft = 1;
	return true;
}

bool CollisionDetection::OBBIntersection(const OBBVolume& volumeA, const Transform& worldTransformA,
	const OBBVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {
	// Approximate by converting each OBB to a world-aligned AABB using orientation absolute values
	Vector3 posA = worldTransformA.GetPosition();
	Vector3 posB = worldTransformB.GetPosition();

	Vector3 halfA = volumeA.GetHalfDimensions();
	Vector3 halfB = volumeB.GetHalfDimensions();

	Vector3 scaleA = worldTransformA.GetScale();
	Vector3 scaleB = worldTransformB.GetScale();

	halfA = Vector3(halfA.x * scaleA.x, halfA.y * scaleA.y, halfA.z * scaleA.z);
	halfB = Vector3(halfB.x * scaleB.x, halfB.y * scaleB.y, halfB.z * scaleB.z);

	Matrix3 orientA = Quaternion::RotationMatrix<Matrix3>(worldTransformA.GetOrientation());
	Matrix3 orientB = Quaternion::RotationMatrix<Matrix3>(worldTransformB.GetOrientation());

	Matrix3 absA = Matrix::Absolute(orientA);
	Matrix3 absB = Matrix::Absolute(orientB);

	Vector3 aabbHalfA = Vector3(
		absA.array[0][0] * halfA.x + absA.array[1][0] * halfA.y + absA.array[2][0] * halfA.z,
		absA.array[0][1] * halfA.x + absA.array[1][1] * halfA.y + absA.array[2][1] * halfA.z,
		absA.array[0][2] * halfA.x + absA.array[1][2] * halfA.y + absA.array[2][2] * halfA.z
	);

	Vector3 aabbHalfB = Vector3(
		absB.array[0][0] * halfB.x + absB.array[1][0] * halfB.y + absB.array[2][0] * halfB.z,
		absB.array[0][1] * halfB.x + absB.array[1][1] * halfB.y + absB.array[2][1] * halfB.z,
		absB.array[0][2] * halfB.x + absB.array[1][2] * halfB.y + absB.array[2][2] * halfB.z
	);

	if (!AABBTest(posA, posB, aabbHalfA, aabbHalfB)) {
		return false;
	}

	// Use center delta to define a simple normal (not fully SAT accurate)
	Vector3 delta = posB - posA;
	Vector3 normal = Vector::Normalise(delta);
	if (Vector::LengthSquared(delta) < 1e-6f) {
		normal = Vector3(0, 1, 0);
	}

	// Estimate penetration as min overlap on axes
	Vector3 total = aabbHalfA + aabbHalfB;
	float px = total.x - std::abs(delta.x);
	float py = total.y - std::abs(delta.y);
	float pz = total.z - std::abs(delta.z);
	float penetration = std::min(px, std::min(py, pz));

	collisionInfo.AddContactPoint(Vector3(), Vector3(), normal, penetration);
	collisionInfo.framesLeft = 1;
	return true;
}

Matrix4 GenerateInverseView(const Camera &c) {
	float pitch = c.GetPitch();
	float yaw	= c.GetYaw();
	Vector3 position = c.GetPosition();

	Matrix4 iview =
		Matrix::Translation(position) *
		Matrix::Rotation(-yaw, Vector3(0, -1, 0)) *
		Matrix::Rotation(-pitch, Vector3(-1, 0, 0));

	return iview;
}

Matrix4 GenerateInverseProjection(float aspect, float fov, float nearPlane, float farPlane) {
	float negDepth = nearPlane - farPlane;

	float invNegDepth = negDepth / (2 * (farPlane * nearPlane));

	Matrix4 m;

	float h = 1.0f / tan(fov*PI_OVER_360);

	m.array[0][0] = aspect / h;
	m.array[1][1] = tan(fov * PI_OVER_360);
	m.array[2][2] = 0.0f;

	m.array[2][3] = invNegDepth;//// +PI_OVER_360;
	m.array[3][2] = -1.0f;
	m.array[3][3] = (0.5f / nearPlane) + (0.5f / farPlane);

	return m;
}

Vector3 CollisionDetection::Unproject(const Vector3& screenPos, const PerspectiveCamera& cam) {
	Vector2i screenSize = Window::GetWindow()->GetScreenSize();

	float aspect = Window::GetWindow()->GetScreenAspect();
	float fov		= cam.GetFieldOfVision();
	float nearPlane = cam.GetNearPlane();
	float farPlane  = cam.GetFarPlane();

	//Create our inverted matrix! Note how that to get a correct inverse matrix,
	//the order of matrices used to form it are inverted, too.
	Matrix4 invVP = GenerateInverseView(cam) * GenerateInverseProjection(aspect, fov, nearPlane, farPlane);

	Matrix4 proj  = cam.BuildProjectionMatrix(aspect);

	//Our mouse position x and y values are in 0 to screen dimensions range,
	//so we need to turn them into the -1 to 1 axis range of clip space.
	//We can do that by dividing the mouse values by the width and height of the
	//screen (giving us a range of 0.0 to 1.0), multiplying by 2 (0.0 to 2.0)
	//and then subtracting 1 (-1.0 to 1.0).
	Vector4 clipSpace = Vector4(
		(screenPos.x / (float)screenSize.x) * 2.0f - 1.0f,
		(screenPos.y / (float)screenSize.y) * 2.0f - 1.0f,
		(screenPos.z),
		1.0f
	);

	//Then, we multiply our clipspace coordinate by our inverted matrix
	Vector4 transformed = invVP * clipSpace;

	//our transformed w coordinate is now the 'inverse' perspective divide, so
	//we can reconstruct the final world space by dividing x,y,and z by w.
	return Vector3(transformed.x / transformed.w, transformed.y / transformed.w, transformed.z / transformed.w);
}

Ray CollisionDetection::BuildRayFromMouse(const PerspectiveCamera& cam) {
	Vector2 screenMouse = Window::GetMouse()->GetAbsolutePosition();
	Vector2i screenSize	= Window::GetWindow()->GetScreenSize();

	//We remove the y axis mouse position from height as OpenGL is 'upside down',
	//and thinks the bottom left is the origin, instead of the top left!
	Vector3 nearPos = Vector3(screenMouse.x,
		screenSize.y - screenMouse.y,
		-0.99999f
	);

	//We also don't use exactly 1.0 (the normalised 'end' of the far plane) as this
	//causes the unproject function to go a bit weird. 
	Vector3 farPos = Vector3(screenMouse.x,
		screenSize.y - screenMouse.y,
		0.99999f
	);

	Vector3 a = Unproject(nearPos, cam);
	Vector3 b = Unproject(farPos, cam);
	Vector3 c = b - a;

	c = Vector::Normalise(c);

	return Ray(cam.GetPosition(), c);
}

//http://bookofhook.com/mousepick.pdf
Matrix4 CollisionDetection::GenerateInverseProjection(float aspect, float fov, float nearPlane, float farPlane) {
	Matrix4 m;

	float t = tan(fov*PI_OVER_360);

	float neg_depth = nearPlane - farPlane;

	const float h = 1.0f / t;

	float c = (farPlane + nearPlane) / neg_depth;
	float e = -1.0f;
	float d = 2.0f*(nearPlane*farPlane) / neg_depth;

	m.array[0][0] = aspect / h;
	m.array[1][1] = tan(fov * PI_OVER_360);
	m.array[2][2] = 0.0f;

	m.array[2][3] = 1.0f / d;

	m.array[3][2] = 1.0f / e;
	m.array[3][3] = -c / (d * e);

	return m;
}

/*
And here's how we generate an inverse view matrix. It's pretty much
an exact inversion of the BuildViewMatrix function of the Camera class!
*/
Matrix4 CollisionDetection::GenerateInverseView(const Camera &c) {
	float pitch = c.GetPitch();
	float yaw	= c.GetYaw();
	Vector3 position = c.GetPosition();

	Matrix4 iview =
		Matrix::Translation(position) *
		Matrix::Rotation(yaw, Vector3(0, 1, 0)) *
		Matrix::Rotation(pitch, Vector3(1, 0, 0));

	return iview;
}


/*
If you've read through the Deferred Rendering tutorial you should have a pretty
good idea what this function does. It takes a 2D position, such as the mouse
position, and 'unprojects' it, to generate a 3D world space position for it.

Just as we turn a world space position into a clip space position by multiplying
it by the model, view, and projection matrices, we can turn a clip space
position back to a 3D position by multiply it by the INVERSE of the
view projection matrix (the model matrix has already been assumed to have
'transformed' the 2D point). As has been mentioned a few times, inverting a
matrix is not a nice operation, either to understand or code. But! We can cheat
the inversion process again, just like we do when we create a view matrix using
the camera.

So, to form the inverted matrix, we need the aspect and fov used to create the
projection matrix of our scene, and the camera used to form the view matrix.

*/
Vector3	CollisionDetection::UnprojectScreenPosition(Vector3 position, float aspect, float fov, const PerspectiveCamera& c) {
	//Create our inverted matrix! Note how that to get a correct inverse matrix,
	//the order of matrices used to form it are inverted, too.
	Matrix4 invVP = GenerateInverseView(c) * GenerateInverseProjection(aspect, fov, c.GetNearPlane(), c.GetFarPlane());


	Vector2i screenSize = Window::GetWindow()->GetScreenSize();

	//Our mouse position x and y values are in 0 to screen dimensions range,
	//so we need to turn them into the -1 to 1 axis range of clip space.
	//We can do that by dividing the mouse values by the width and height of the
	//screen (giving us a range of 0.0 to 1.0), multiplying by 2 (0.0 to 2.0)
	//and then subtracting 1 (-1.0 to 1.0).
	Vector4 clipSpace = Vector4(
		(position.x / (float)screenSize.x) * 2.0f - 1.0f,
		(position.y / (float)screenSize.y) * 2.0f - 1.0f,
		(position.z) - 1.0f,
		1.0f
	);

	//Then, we multiply our clipspace coordinate by our inverted matrix
	Vector4 transformed = invVP * clipSpace;

	//our transformed w coordinate is now the 'inverse' perspective divide, so
	//we can reconstruct the final world space by dividing x,y,and z by w.
	return Vector3(transformed.x / transformed.w, transformed.y / transformed.w, transformed.z / transformed.w);
}

