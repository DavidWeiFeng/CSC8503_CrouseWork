#include "PositionConstraint.h"
#include "GameObject.h"
#include "PhysicsObject.h"

using namespace NCL;
using namespace Maths;
using namespace CSC8503;

PositionConstraint::PositionConstraint(GameObject* a, GameObject* b, float d)
{
	objectA		= a;
	objectB		= b;
	distance	= d;
}

//a simple constraint that stops objects from being more than <distance> away
//from each other...this would be all we need to simulate a rope, or a ragdoll
void PositionConstraint::UpdateConstraint(float dt)	
{
	if (!objectA || !objectB) {
		return;
	}
	PhysicsObject* physA = objectA->GetPhysicsObject();
	PhysicsObject* physB = objectB->GetPhysicsObject();
	if (!physA && !physB) {
		return;
	}

	Vector3 posA = objectA->GetTransform().GetPosition();
	Vector3 posB = objectB->GetTransform().GetPosition();
	Vector3 ab = posB - posA;
	float currentLen = Vector::Length(ab);
	if (currentLen < 1e-5f || currentLen <= distance) {
		return;
	}

	Vector3 dir = ab / currentLen;
	float error = currentLen - distance;
	if (error < 0.05f) {
		return; // skip tiny errors to avoid over-damping
	}

	float invMassA = physA ? physA->GetInverseMass() : 0.0f;
	float invMassB = physB ? physB->GetInverseMass() : 0.0f;
	float totalInvMass = invMassA + invMassB;
	if (totalInvMass <= 0.0f) {
		return;
	}

	Vector3 correction = dir * (error / totalInvMass);
	const float correctionFactor = 0.20f; // softer pull to allow dragging
	const float velDamp = 1.0f; // keep most velocity, avoid freezing

	// Apply positional correction
	if (physA && invMassA > 0.0f) {
		Vector3 newPosA = posA + correction * invMassA * correctionFactor;
		objectA->GetTransform().SetPosition(newPosA);
		physA->SetLinearVelocity(physA->GetLinearVelocity() * velDamp);
	}
	if (physB && invMassB > 0.0f) {
		Vector3 newPosB = posB - correction * invMassB * correctionFactor;
		objectB->GetTransform().SetPosition(newPosB);
		physB->SetLinearVelocity(physB->GetLinearVelocity() * velDamp);
	}
}
