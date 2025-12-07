#pragma once
#include <vector>
#include "RenderObject.h"
#include "PositionConstraint.h"
#include "NavigationMesh.h"
#include "NavigationPath.h"
#include "StateMachine.h"
#include "StateTransition.h"
#include "State.h"
namespace NCL {
	class Controller;

	namespace Rendering {
		class Mesh;
		class Texture;
		class Shader;
	}
	namespace CSC8503 {
		class GameTechRendererInterface;
		class PhysicsSystem;
		class GameWorld;
		class GameObject;
		class PositionConstraint;

		class TutorialGame {
		public:
			TutorialGame(GameWorld& gameWorld, GameTechRendererInterface& renderer, PhysicsSystem& physics);
			~TutorialGame();

			virtual void UpdateGame(float dt);
			void LateUpdate(float dt);

		protected:
			void InitCamera();

			void InitWorld();

			/*
			These are some of the world/object creation functions I created when testing the functionality
			in the module. Feel free to mess around with them to see different objects being created in different
			test scenarios (constraints, collision types, and so on).
			*/
			void InitGameExamples();

			void CreateSphereGrid(int numRows, int numCols, float rowSpacing, float colSpacing, float radius);
			void CreatedMixedGrid(int numRows, int numCols, float rowSpacing, float colSpacing);
			void CreateAABBGrid(int numRows, int numCols, float rowSpacing, float colSpacing, const NCL::Maths::Vector3& cubeDims);
			void BuildSlopeScene();
			void UpdateGateAndPlate(float dt);

			bool SelectObject();
			void MoveSelectedObject();
			void DebugObjectMovement();
			void LockedObjectMovement();
			void UpdateThirdPersonCamera(float dt);
			void HandlePlayerMovement(float dt);
			void HandleGrab();
			bool IsPlayerGrounded() const;
			void InitEnemyAgent(const NCL::Maths::Vector3& pos);
			void UpdateEnemyAI(float dt);
			void UpdateEnemyIdle(float dt);
			void UpdateEnemyChase(float dt);
			void UpdateEnemyRecover(float dt);
			bool BuildEnemyPathToPlayer();
			void ResetEnemyPath();
			void OnEnemyStateChanged(NCL::CSC8503::State* newState);

			GameObject* AddFloorToWorld(const NCL::Maths::Vector3& position);
			GameObject* AddSphereToWorld(const NCL::Maths::Vector3& position, float radius, float inverseMass = 10.0f);
			GameObject* AddCubeToWorld(const NCL::Maths::Vector3& position, NCL::Maths::Vector3 dimensions, float inverseMass = 10.0f);
			GameObject* AddOBBCubeToWorld(const NCL::Maths::Vector3& position, NCL::Maths::Vector3 dimensions, const NCL::Maths::Quaternion& orientation, float inverseMass = 10.0f);

			GameObject* AddPlayerToWorld(const NCL::Maths::Vector3& position);
			GameObject* AddEnemyToWorld(const NCL::Maths::Vector3& position);
			GameObject* AddBonusToWorld(const NCL::Maths::Vector3& position);

			GameObject* playerObject = nullptr;
			GameWorld& world;
			GameTechRendererInterface& renderer;
			PhysicsSystem& physics;
			Controller* controller;

			bool useGravity;
			bool inSelectionMode;

			float		forceMagnitude;
			float		playerMoveForce = 30.0f;
			float		playerMaxSpeed  = 10.0f;
			float		playerJumpImpulse = 11.0f;
			float		groundFriction = 0.93f; // treated as per-second damping base
			float		airFriction    = 0.99f; // treated as per-second damping base
			float		playerRadius   = 1.0f;
			float		cameraFollowDistance = 12.0f;
			float		cameraFollowHeight   = 3.0f;
			float		cameraLookSensitivity = 0.35f;
			float		grabMaxDistance = 10.0f;
			float		grabDamping     = 5.0f;
			float		grabSpring      = 20.0f;   // stiffness
			float		grabMaxForce    = 500.0f;  // clamp to avoid explosion
			float		grabMaxLinearSpeed  = 20.0f;
			float		grabMaxAngularSpeed = 5.0f;
			
			// Puzzle objects
			GameObject* pushableCube = nullptr;
			GameObject* pressurePlate = nullptr;
			GameObject* gateLeftObject  = nullptr;
			GameObject* gateRightObject = nullptr;
			bool gateOpen = false;
			float gateAnimT = 0.0f;
			float gateSlideDistance = 6.0f;
			NCL::Maths::Vector3 gateLeftClosedPos  = NCL::Maths::Vector3();
			NCL::Maths::Vector3 gateRightClosedPos = NCL::Maths::Vector3();
			NCL::Maths::Vector3 gateLeftOpenPos    = NCL::Maths::Vector3();
			NCL::Maths::Vector3 gateRightOpenPos   = NCL::Maths::Vector3();
			NCL::Maths::Vector3 pushCubeHalfSize = NCL::Maths::Vector3(0.5f, 0.5f, 0.5f);
			NCL::Maths::Vector3 plateHalfSize    = NCL::Maths::Vector3(2.0f, 0.2f, 2.0f);
			NCL::Maths::Vector3 gateHalfSize     = NCL::Maths::Vector3(2.0f, 4.0f, 0.5f);
			
			struct EnemyAgent {
				GameObject* object = nullptr;
				StateMachine stateMachine;
				State* idleState = nullptr;
				State* chaseState = nullptr;
				State* recoverState = nullptr;
				std::vector<NCL::Maths::Vector3> path;
				size_t pathIndex = 0;
				float pathTimer = 0.0f;
				float pathRefreshTime = 1.0f;
				float stuckTimer = 0.0f;
				float recoverTimer = 0.0f;
				NCL::Maths::Vector3 lastPos = NCL::Maths::Vector3();
				NCL::Maths::Vector3 lastPlayerPos = NCL::Maths::Vector3();
				bool requestRecover = false;
				bool hasPath = false;
			};
			EnemyAgent enemyAgent;
			NavigationMesh* navMesh = nullptr;
			float enemyMoveSpeed = 8.0f; // units/sec
			float enemyWaypointTolerance = 0.5f;
			float enemyChaseDistance = 80.0f;
			float enemyLoseDistance = 120.0f;
			float enemyRepathPlayerDelta = 3.0f;
			float enemyStuckTime = 1.25f;
			float enemyStuckMoveEpsilon = 0.05f;
			float enemyRecoverDuration = 0.6f;

			GameObject* selectionObject = nullptr;

			Rendering::Mesh* capsuleMesh	= nullptr;
			Rendering::Mesh* cubeMesh		= nullptr;
			Rendering::Mesh* sphereMesh		= nullptr;

			Rendering::Texture* defaultTex  = nullptr;
			Rendering::Texture* checkerTex	= nullptr;
			Rendering::Texture* glassTex	= nullptr;

			//Coursework Meshes
			Rendering::Mesh* catMesh	= nullptr;
			Rendering::Mesh* kittenMesh = nullptr;
			Rendering::Mesh* enemyMesh	= nullptr;
			Rendering::Mesh* bonusMesh	= nullptr;

			GameTechMaterial checkerMaterial;
			GameTechMaterial glassMaterial;
			GameTechMaterial notexMaterial;

			//Coursework Additional functionality	
			GameObject* lockedObject = nullptr;
			NCL::Maths::Vector3 lockedOffset = NCL::Maths::Vector3(0, 30, -15);
			void LockCameraToObject(GameObject* o) {
				lockedObject = o;
			}

			GameObject* objClosest = nullptr;
			GameObject* forwardHitObject = nullptr;
			GameObject* grabbedObject = nullptr;
			PositionConstraint* grabConstraint = nullptr;
			NCL::Maths::Vector3 grabLocalOffset = NCL::Maths::Vector3();
			float grabDistance = 0.0f;
		};
	}
}
