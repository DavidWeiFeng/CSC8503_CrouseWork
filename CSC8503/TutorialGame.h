#pragma once
#include <vector>
#include <string>
#include <unordered_set>
#include <array>
#include "RenderObject.h"
#include "PositionConstraint.h"
#include "NavigationMesh.h"
#include "NavigationPath.h"
#include "EnemyAI.h"
#include "PlayerObject.h"
#include <unordered_map>
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
		class NetworkedGame;

		class TutorialGame {
		public:
			friend class PlayerObject;
			TutorialGame(GameWorld& gameWorld, GameTechRendererInterface& renderer, PhysicsSystem& physics);
			~TutorialGame();

			virtual void UpdateGame(float dt);
			void LateUpdate(float dt);

		protected:
			void InitCamera();

			void InitWorld();
			bool HandleStartMenu(float dt);

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
			void HandleBouncePad(float dt);
			void SpawnBouncePad();
			void UpdateSweeper(float dt);
			void HandleGoalPlate(float dt);
			void RenderGameOverUI(float dt);

			bool SelectObject();
			void MoveSelectedObject();
			void DebugObjectMovement();
			void LockedObjectMovement();
			void UpdateThirdPersonCamera(float dt);
			void HandlePlayerMovement(float dt);
			void HandleGrab();
			bool IsPlayerGrounded() const;
			void MovePlayer(GameObject* player, float dt, bool keyW, bool keyA, bool keyS, bool keyD, bool keySpace, float cameraYaw);
			void InitEnemyAgent(const NCL::Maths::Vector3& pos);
			void InitMazeEnemyAgent(const NCL::Maths::Vector3& pos);
			void UpdateEnemyAI(float dt);

			GameObject* AddFloorToWorld(const NCL::Maths::Vector3& position);
			GameObject* AddSphereToWorld(const NCL::Maths::Vector3& position, float radius, float inverseMass = 10.0f, Rendering::Mesh* mesh = nullptr, const GameTechMaterial* material = nullptr, const std::string& name = "");
			GameObject* AddCubeToWorld(const NCL::Maths::Vector3& position, NCL::Maths::Vector3 dimensions, float inverseMass = 10.0f, Rendering::Mesh* mesh = nullptr, const GameTechMaterial* material = nullptr);
			GameObject* AddOBBCubeToWorld(const NCL::Maths::Vector3& position, NCL::Maths::Vector3 dimensions, const NCL::Maths::Quaternion& orientation, float inverseMass = 10.0f);

			GameObject* AddPlayerToWorld(const NCL::Maths::Vector3& position);
			GameObject* AddEnemyToWorld(const NCL::Maths::Vector3& position);
			GameObject* AddBonusToWorld(const NCL::Maths::Vector3& position);
			GameObject* AddCoinToWorld(const NCL::Maths::Vector3& position);
			GameObject* BuildSphereObject(GameObject* obj, const NCL::Maths::Vector3& position, float radius, float inverseMass, Rendering::Mesh* mesh = nullptr, const GameTechMaterial* material = nullptr);
			void OnPlayerCollectCoin(GameObject* coin);
			void OnPlayerCaught();
			void UpdatePendingRemovals(float dt);
			void UpdateCoinPickups();
			void LoadMazeFromTxt(const std::string& path, const NCL::Maths::Vector3& offset);

			GameObject* playerObject = nullptr;
			GameWorld& world;
			GameTechRendererInterface& renderer;
			PhysicsSystem& physics;
			Controller* controller;
			int playerScore = 0;
			struct PendingRemoval {
				GameObject* obj = nullptr;
				int framesLeft = 0;
			};
			std::vector<PendingRemoval> pendingRemoval;

			bool useGravity;
			bool inSelectionMode;
			bool inStartMenu = true;
			int  menuSelection = 0; // 0 = single, 1 = multiplayer
			enum class GameMode { Single, Multi };
			GameMode currentMode = GameMode::Single;
			bool isFullscreen = false;
			std::string ipInput = "127.0.0.1";
			bool editingIP = false;

			float		forceMagnitude;
			float		playerMoveForce = 30.0f;
			float		playerMaxSpeed  = 10.0f;
			float		playerJumpImpulse = 14.0f;
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
			GameObject* bouncePad = nullptr;
			bool bouncePadSpawned = false;
			bool playerOnBouncePad = false;
			GameObject* sweeperStick = nullptr;
			float sweeperAngle = 0.0f;
			float sweeperAngularSpeed = 1.5f;
			float sweeperLen = 30.0f;
			float sweeperThickness = 1.0f;
			float sweeperImpulseScale = 25.0f;
			float sweeperUpImpulse = 40.0f;
			bool sweeperHitActive = false;
			GameObject* goalPlate = nullptr;
			bool goalTriggered = false;
			NCL::Maths::Vector3 goalPlateHalf = NCL::Maths::Vector3(2.0f, 0.1f, 2.0f);
			// 协作重物
			GameObject* heavyObject = nullptr;
			NCL::Maths::Vector3 heavyHalfSize = NCL::Maths::Vector3(2.0f, 2.0f, 2.0f);
			float heavyGrabRange = 6.0f;
			bool localHeavyGrabbing = false;
			int currentHeavyGrabbers = 0;
			std::unordered_map<GameObject*, PositionConstraint*> heavyGrabConstraints;
			// 结算界面
			bool showGameOver = false;
			bool highScoreEligible = false;
			bool nameSubmitted = false;
			std::string nameInput = "";
			NCL::Maths::Vector3 floorCenter = NCL::Maths::Vector3();
			NCL::Maths::Vector3 floorHalfSize = NCL::Maths::Vector3();
			NCL::Maths::Vector3 mazeMin = NCL::Maths::Vector3();
			NCL::Maths::Vector3 mazeMax = NCL::Maths::Vector3();
			NCL::Maths::Vector3 mazeCenter = NCL::Maths::Vector3();
			NCL::Maths::Vector3 mazeHalfSize = NCL::Maths::Vector3();
			NCL::Maths::Vector3 playerSpawnPos = NCL::Maths::Vector3();
			float gateSlideDistance = 6.0f;
			NCL::Maths::Vector3 gateLeftClosedPos  = NCL::Maths::Vector3();
			NCL::Maths::Vector3 gateRightClosedPos = NCL::Maths::Vector3();
			NCL::Maths::Vector3 gateLeftOpenPos    = NCL::Maths::Vector3();
			NCL::Maths::Vector3 gateRightOpenPos   = NCL::Maths::Vector3();
			NCL::Maths::Vector3 pushCubeHalfSize = NCL::Maths::Vector3(0.7f, 0.7f, 0.7f);
			NCL::Maths::Vector3 plateHalfSize    = NCL::Maths::Vector3(3.0f, 0.2f, 4.0f);
			NCL::Maths::Vector3 gateHalfSize     = NCL::Maths::Vector3(2.0f, 4.0f, 0.5f);
			
			GameObject* enemyObject = nullptr;
			EnemyAI* enemyAI = nullptr;
			GameObject* mazeEnemyObject = nullptr;
			EnemyAI* mazeEnemyAI = nullptr;
			NavigationMesh* navMesh = nullptr;

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
			Rendering::Mesh* coinMesh = nullptr;

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
			std::vector<GameObject*> coins;
			std::unordered_set<GameObject*> floorCoins;
			float coinRadius = 0.2f;
			std::string mazeStatus = "not loaded";
		};
	}
}
