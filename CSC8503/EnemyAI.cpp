#include "EnemyAI.h"
#include "Debug.h"
#include "PhysicsObject.h"

using namespace NCL;
using namespace CSC8503;
using namespace NCL::Maths;

EnemyAI::EnemyAI(NavigationMesh& mesh, Params p) : navMesh(mesh), params(p) {
	InitStates();
}

void EnemyAI::SetOwner(GameObject* obj) {
	owner = obj;
	if (owner) {
		lastPos = owner->GetTransform().GetPosition();
	}
}

void EnemyAI::SetTarget(GameObject* t) {
	target = t;
	if (target) {
		lastTargetPos = target->GetTransform().GetPosition();
	}
}

void EnemyAI::Reset() {
	requestRecover = false;
	recoverTimer = 0.0f;
	stuckTimer = 0.0f;
	pathTimer = 0.0f;
	ResetPath();
}

void EnemyAI::InitStates() {
	idleState    = new State([this](float dt) { UpdateIdle(dt); });
	chaseState   = new State([this](float dt) { UpdateChase(dt); });
	recoverState = new State([this](float dt) { UpdateRecover(dt); });

	stateMachine.AddState(idleState);
	stateMachine.AddState(chaseState);
	stateMachine.AddState(recoverState);

	stateMachine.AddTransition(new StateTransition(idleState, chaseState, [this]() {
		if (!owner || !target) return false;
		float dist = Vector::Length(target->GetTransform().GetPosition() - owner->GetTransform().GetPosition());
		return dist < params.chaseDistance;
	}));
	stateMachine.AddTransition(new StateTransition(chaseState, idleState, [this]() {
		if (!owner || !target) return true;
		float dist = Vector::Length(target->GetTransform().GetPosition() - owner->GetTransform().GetPosition());
		return dist > params.loseDistance;
	}));
	stateMachine.AddTransition(new StateTransition(chaseState, recoverState, [this]() {
		return requestRecover;
	}));
	stateMachine.AddTransition(new StateTransition(recoverState, chaseState, [this]() {
		if (!owner || !target) return false;
		float dist = Vector::Length(target->GetTransform().GetPosition() - owner->GetTransform().GetPosition());
		return recoverTimer >= params.recoverDuration && dist < params.loseDistance;
	}));
	stateMachine.AddTransition(new StateTransition(recoverState, idleState, [this]() {
		if (!owner || !target) return recoverTimer >= params.recoverDuration;
		float dist = Vector::Length(target->GetTransform().GetPosition() - owner->GetTransform().GetPosition());
		return recoverTimer >= params.recoverDuration && dist >= params.loseDistance;
	}));
}

void EnemyAI::OnStateChanged(State* newState) {
	if (newState == recoverState || newState == idleState) {
		recoverTimer = 0.0f;
		requestRecover = false;
		ResetPath();
	}
	if (newState == chaseState) {
		recoverTimer = 0.0f;
		requestRecover = false;
		stuckTimer = 0.0f;
		if (!hasPath) {
			BuildPath();
		}
	}
}

void EnemyAI::ResetPath() {
	path.clear();
	pathIndex = 0;
	hasPath = false;
	pathTimer = 0.0f;
}

bool EnemyAI::BuildPath() {
	if (!owner || !target) {
		return false;
	}
	NavigationPath navPath;
	Vector3 from = owner->GetTransform().GetPosition();
	Vector3 to = target->GetTransform().GetPosition();
	if (!navMesh.FindPath(from, to, navPath)) {
		hasPath = false;
		return false;
	}
	path.clear();
	Vector3 wp;
	while (navPath.PopWaypoint(wp)) {
		path.emplace_back(wp);
	}
	if (path.size() > 1) {
		pathIndex = 1; // skip start
	} else {
		pathIndex = 0;
	}
	hasPath = !path.empty();
	pathTimer = 0.0f;
	lastTargetPos = target->GetTransform().GetPosition();
	return hasPath;
}

void EnemyAI::Update(float dt) {
	if (!owner || !target) {
		return;
	}
	State* before = stateMachine.GetActiveState();
	stateMachine.Update(dt);
	State* after = stateMachine.GetActiveState();
	if (after != before) {
		OnStateChanged(after);
	}
}

void EnemyAI::UpdateIdle(float dt) {
	(void)dt;
	requestRecover = false;
	ResetPath();
}

void EnemyAI::UpdateChase(float dt) {
	if (!owner || !target) {
		return;
	}
	pathTimer += dt;
	Vector3 targetPos = target->GetTransform().GetPosition();
	if (!hasPath || pathTimer >= params.pathRefreshTime ||
		Vector::Length(targetPos - lastTargetPos) > params.repathPlayerDelta) {
		if (!BuildPath()) {
			requestRecover = true;
			return;
		}
	}
	PhysicsObject* phys = owner->GetPhysicsObject();
	if (!phys) {
		return;
	}
	Vector3 pos = owner->GetTransform().GetPosition();
	Vector3 seek = targetPos;
	if (pathIndex < path.size()) {
		seek = path[pathIndex];
	}
	Vector3 toTarget = seek - pos;
	toTarget.y = 0.0f;
	float dist = Vector::Length(toTarget);
	if (dist < params.waypointTolerance && pathIndex + 1 < path.size()) {
		pathIndex++;
		seek = path[pathIndex];
		toTarget = seek - pos;
		toTarget.y = 0.0f;
		dist = Vector::Length(toTarget);
	}
	if (Vector::LengthSquared(toTarget) > 1e-4f) {
		Vector3 dir = Vector::Normalise(toTarget);
		float step = params.moveSpeed * dt;
		Vector3 newPos = pos + dir * step;
		if (Vector::Length(newPos - pos) > dist) {
			newPos = seek;
		}
		owner->GetTransform().SetPosition(newPos);
		phys->SetLinearVelocity(Vector3());
	}
	float moved = Vector::Length(pos - lastPos);
	if (moved < params.stuckMoveEpsilon) {
		stuckTimer += dt;
		if (stuckTimer > params.stuckTime) {
			requestRecover = true;
		}
	} else {
		stuckTimer = 0.0f;
		lastPos = pos;
	}
	// Debug draw path
	if (pathIndex < path.size()) {
		Vector3 prev = pos;
		for (size_t i = pathIndex; i < path.size(); ++i) {
			Vector3 wp = path[i];
			Debug::DrawLine(prev, wp, Vector4(1, 0, 0, 1));
			Debug::DrawLine(wp, wp + Vector3(0, 0.5f, 0), Vector4(0, 1, 0, 1));
			prev = wp;
		}
		Debug::Print("Enemy path len: " + std::to_string(path.size() - pathIndex), Vector2(5, 70), Debug::RED);
	}
}

void EnemyAI::UpdateRecover(float dt) {
	if (!owner) {
		return;
	}
	recoverTimer += dt;
	requestRecover = false;
	PhysicsObject* phys = owner->GetPhysicsObject();
	if (phys) {
		Vector3 vel = phys->GetLinearVelocity();
		vel *= 0.9f;
		phys->SetLinearVelocity(vel);
	}
	if (!hasPath && recoverTimer > params.recoverDuration * 0.5f) {
		BuildPath();
	}
}
