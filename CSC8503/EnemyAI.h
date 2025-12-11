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
			float waypointTolerance = 1.0f;
			float chaseDistance     = 10.0f;
			float loseDistance      = 15.0f;
			float catchDistance     = 1.0f;
			float repathPlayerDelta = 3.0f;
			float stuckTime         = 1.25f;
			float stuckMoveEpsilon  = 0.05f;
			float recoverDuration   = 0.6f;
			float pathRefreshTime   = 1.0f;
			bool  useFunnel         = true;
			NCL::Maths::Vector3 floorMin = NCL::Maths::Vector3(-1000, -1000, -1000);
			NCL::Maths::Vector3 floorMax = NCL::Maths::Vector3(1000, 1000, 1000);
		};

		EnemyAI(NavigationMesh& navMesh, Params params = Params());
		~EnemyAI() = default;

		void SetOwner(GameObject* obj);
		void SetTarget(GameObject* target);
		void SetPatrolPoints(const std::vector<NCL::Maths::Vector3>& points);
		void SetOnCatch(const std::function<void()>& fn) { onCatch = fn; }
		void Update(float dt);
		void Reset();

	private:
		void InitStates();
		void OnStateChanged(State* newState);
		void UpdateIdle(float dt);
		void UpdatePatrol(float dt);
		void UpdateChase(float dt);
		void UpdateRecover(float dt);
		NCL::Maths::Vector3 ChoosePatrolTarget();
		bool BuildPath();
		bool BuildPathTo(const NCL::Maths::Vector3& dest);
		void ResetPath();
		bool IsTargetOnFloor() const;

		NavigationMesh& navMesh;
		Params params;

		GameObject* owner  = nullptr;
		GameObject* target = nullptr;
		std::function<void()> onCatch;

		StateMachine stateMachine;
		State* idleState    = nullptr; // used as patrol
		State* chaseState   = nullptr;
		State* recoverState = nullptr;

		std::vector<NCL::Maths::Vector3> path;
		size_t pathIndex = 0;
		float pathTimer = 0.0f;
		float stuckTimer = 0.0f;
		float recoverTimer = 0.0f;
		NCL::Maths::Vector3 lastPos = NCL::Maths::Vector3();
		NCL::Maths::Vector3 lastTargetPos = NCL::Maths::Vector3();
		NCL::Maths::Vector3 currentPatrolTarget = NCL::Maths::Vector3();
		float patrolTargetTimer = 0.0f;
		bool requestRecover = false;
		bool hasPath = false;
		std::vector<NCL::Maths::Vector3> patrolPoints;
		size_t patrolIndex = 0;
	};
}
