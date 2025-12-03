#pragma once
#include "RenderObject.h"
#include "PositionConstraint.h"
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

			bool SelectObject();
			void MoveSelectedObject();
			void DebugObjectMovement();
			void LockedObjectMovement();
			void UpdateThirdPersonCamera(float dt);
			void HandlePlayerMovement(float dt);
			void HandleGrab();
			bool IsPlayerGrounded() const;

			GameObject* AddFloorToWorld(const NCL::Maths::Vector3& position);
			GameObject* AddSphereToWorld(const NCL::Maths::Vector3& position, float radius, float inverseMass = 10.0f);
			GameObject* AddCubeToWorld(const NCL::Maths::Vector3& position, NCL::Maths::Vector3 dimensions, float inverseMass = 10.0f);

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
