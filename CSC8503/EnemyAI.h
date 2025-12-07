#pragma once
#include <vector>
#include "NavigationMesh.h"
#include "NavigationPath.h"
#include "GameObject.h"
#include "StateMachine.h"
#include "StateTransition.h"
#include "State.h"

namespace NCL::CSC8503 {
	class EnemyAI {
	public:
		struct Params {
			float moveSpeed         = 8.0f;
			float waypointTolerance = 0.5f;
			float chaseDistance     = 80.0f;
			float loseDistance      = 120.0f;
			float repathPlayerDelta = 3.0f;
			float stuckTime         = 1.25f;
			float stuckMoveEpsilon  = 0.05f;
			float recoverDuration   = 0.6f;
			float pathRefreshTime   = 1.0f;
		};

		EnemyAI(NavigationMesh& navMesh, Params params = Params());
		~EnemyAI() = default;

		void SetOwner(GameObject* obj);
		void SetTarget(GameObject* target);
		void Update(float dt);
		void Reset();

	private:
		void InitStates();
		void OnStateChanged(State* newState);
		void UpdateIdle(float dt);
		void UpdateChase(float dt);
		void UpdateRecover(float dt);
		bool BuildPath();
		void ResetPath();

		NavigationMesh& navMesh;
		Params params;

		GameObject* owner  = nullptr;
		GameObject* target = nullptr;

		StateMachine stateMachine;
		State* idleState    = nullptr;
		State* chaseState   = nullptr;
		State* recoverState = nullptr;

		std::vector<NCL::Maths::Vector3> path;
		size_t pathIndex = 0;
		float pathTimer = 0.0f;
		float stuckTimer = 0.0f;
		float recoverTimer = 0.0f;
		NCL::Maths::Vector3 lastPos = NCL::Maths::Vector3();
		NCL::Maths::Vector3 lastTargetPos = NCL::Maths::Vector3();
		bool requestRecover = false;
		bool hasPath = false;
	};
}
