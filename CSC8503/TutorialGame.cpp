#include "TutorialGame.h"
#include "GameWorld.h"
#include "PhysicsSystem.h"
#include "PhysicsObject.h"
#include "RenderObject.h"
#include "TextureLoader.h"
#include "PositionConstraint.h"
#include "OrientationConstraint.h"
#include "StateGameObject.h"
#include "Window.h"
#include "Texture.h"
#include "Shader.h"
#include "Mesh.h"
#include "Debug.h"
#include "KeyboardMouseController.h"
#include "GameTechRendererInterface.h"
#include "Ray.h"
#include "OBBVolume.h"
#include <algorithm>
#include <cmath>

using namespace NCL;
using namespace CSC8503;
using namespace NCL::Maths;

/**
 * @brief 构造一个新的教程游戏。
 * @param inWorld 游戏世界。
 * @param inRenderer 游戏渲染器。
 * @param inPhysics 物理系统。
 */
TutorialGame::TutorialGame(GameWorld& inWorld, GameTechRendererInterface& inRenderer, PhysicsSystem& inPhysics)
	:	world(inWorld),
		renderer(inRenderer),
	physics(inPhysics)
{

	forceMagnitude	= 10.0f;
	useGravity		= true;
	inSelectionMode = false;

	controller = new KeyboardMouseController(*Window::GetWindow()->GetKeyboard(), *Window::GetWindow()->GetMouse());

	world.GetMainCamera().SetController(*controller);

	world.SetSunPosition({ -200.0f, 60.0f, -200.0f });
	world.SetSunColour({ 0.8f, 0.8f, 0.5f });

	controller->MapAxis(0, "Sidestep");
	controller->MapAxis(1, "UpDown");
	controller->MapAxis(2, "Forward");

	controller->MapAxis(3, "XLook");
	controller->MapAxis(4, "YLook");

	cubeMesh	= renderer.LoadMesh("cube.msh");
	sphereMesh	= renderer.LoadMesh("sphere.msh");
	catMesh		= renderer.LoadMesh("ORIGAMI_Chat.msh");
	kittenMesh	= renderer.LoadMesh("Kitten.msh");
	coinMesh = renderer.LoadMesh("Coin.msh");
	enemyMesh	= renderer.LoadMesh("Keeper.msh");
	bonusMesh	= renderer.LoadMesh("19463_Kitten_Head_v1.msh");
	capsuleMesh = renderer.LoadMesh("capsule.msh");
	defaultTex	= renderer.LoadTexture("Default.png");
	checkerTex	= renderer.LoadTexture("checkerboard.png");
	glassTex	= renderer.LoadTexture("stainedglass.tga");

	checkerMaterial.type		= MaterialType::Opaque;
	checkerMaterial.diffuseTex	= checkerTex;

	glassMaterial.type			= MaterialType::Transparent;
	glassMaterial.diffuseTex	= glassTex;
	notexMaterial.type          = MaterialType::Opaque;
	notexMaterial.diffuseTex    = defaultTex ? defaultTex : checkerTex;

	if (!navMesh) {
		navMesh = new NavigationMesh("test.navmesh");
	}

	InitCamera();
	InitWorld();
}

bool TutorialGame::IsPlayerGrounded() const {
	if (!playerObject) {
		return false;
	}
	RayCollision collision;
	Vector3 origin = playerObject->GetTransform().GetPosition();
	origin.y += 0.1f; // slight offset above feet
	Ray downRay(origin, Vector3(0, -1, 0));

	if (world.Raycast(downRay, collision, true, playerObject)) {
		if (collision.rayDistance <= (playerRadius + 0.2f)) {
			return true;
		}
	}
	return false;
}

/**
 * @brief 教程游戏的析构函数。
 */
TutorialGame::~TutorialGame()	{
	delete navMesh;
	navMesh = nullptr;
}

/**
 * @brief 每帧更新游戏逻辑。
 * @param dt 帧时间增量。
 */
void TutorialGame::UpdateGame(float dt) {
	float fps = dt > 1e-6f ? 1.0f / dt : 0.0f;

	if (Window::GetKeyboard()->KeyPressed(KeyCodes::F1)) {
		InitWorld(); //We can reset the simulation at any time with F1
		selectionObject = nullptr;
	}

	if (Window::GetKeyboard()->KeyPressed(KeyCodes::F2)) {
		InitCamera(); //F2 will reset the camera to a specific default place
	}
	// 随机化约束顺序 - 减少计算偏差
	if (Window::GetKeyboard()->KeyPressed(KeyCodes::F9)) {
		world.ShuffleConstraints(true);
	}
	if (Window::GetKeyboard()->KeyPressed(KeyCodes::F10)) {
		world.ShuffleConstraints(false);
	}
	// 随机化对象顺序
	if (Window::GetKeyboard()->KeyPressed(KeyCodes::F7)) {
		world.ShuffleObjects(true);
	}
	if (Window::GetKeyboard()->KeyPressed(KeyCodes::F8)) {
		world.ShuffleObjects(false);
	}

	if (lockedObject) {
		LockedObjectMovement(); // 使用箭头键控制锁定对象
	}
	else {
		DebugObjectMovement();  // 调试模式的对象操作
	}
	//This year we can draw debug textures as well!
	//Debug::DrawTex(*defaultTex, Vector2(10, 10), Vector2(5, 5), Debug::WHITE);
	//Debug::DrawLine(Vector3(), Vector3(0, 100, 0), Vector4(1, 0, 0, 1));
	Debug::Print("FPS: " + std::to_string(fps), Vector2(5, 80), Debug::YELLOW);
	if (playerObject) {
		Vector3 pp = playerObject->GetTransform().GetPosition();
		Debug::Print("Player: " + std::to_string(pp.x) + "," + std::to_string(pp.y) + "," + std::to_string(pp.z), Vector2(5, 75), Debug::WHITE);
	}
	HandleGrab();
	HandlePlayerMovement(dt); // 处理玩家输入移动	
	UpdateGateAndPlate(dt);   // 更新压力板与大门
	UpdateEnemyAI(dt);        // 更新敌人AI
	// 右键从玩家正前方发射射线，高亮命中的物体
	world.OperateOnContents(
		[dt](GameObject* o) {
			o->Update(dt);
		}
	);
}

void TutorialGame::UpdateThirdPersonCamera(float dt) {
	if (!playerObject) {
		return;
	}
	(void)dt;

	Camera& camera = world.GetMainCamera();

	float yaw = camera.GetYaw();
	float pitch = camera.GetPitch();

	if (controller) {
		yaw   -= controller->GetNamedAxis("XLook") * cameraLookSensitivity;
		pitch -= controller->GetNamedAxis("YLook") * cameraLookSensitivity;
	}

	pitch = std::clamp(pitch, -80.0f, 80.0f);

	Matrix3 yawRotation = Matrix::RotationMatrix3x3(yaw, Vector3(0, 1, 0));
	Matrix3 pitchRotation = Matrix::RotationMatrix3x3(pitch, Vector3(1, 0, 0));
	Matrix3 orientation = yawRotation * pitchRotation;

	Vector3 forward = orientation * Vector3(0, 0, -1);
	forward = Vector::Normalise(forward);

	Vector3 targetPos = playerObject->GetTransform().GetPosition();
	Vector3 camPos = targetPos + Vector3(0, cameraFollowHeight, 0) - forward * cameraFollowDistance;

	camera.SetPosition(camPos);
	camera.SetYaw(yaw);
	camera.SetPitch(pitch);
}

void TutorialGame::LateUpdate(float dt) {
	(void)dt;
	UpdateThirdPersonCamera(0.0f);
}

/**
 * @brief 初始化摄像机设置。
 */
void TutorialGame::InitCamera() {
	world.GetMainCamera().SetNearPlane(0.1f);
	world.GetMainCamera().SetFarPlane(500.0f);
	world.GetMainCamera().SetPitch(-15.0f);
	world.GetMainCamera().SetYaw(315.0f);
	world.GetMainCamera().SetPosition(Vector3(-60, 40, 60));
	lockedObject = nullptr;
}

/**
 * @brief 初始化游戏世界，清除旧对象并创建新场景。
 */
void TutorialGame::InitWorld() {
	world.ClearAndErase();
	physics.Clear();
	playerObject = nullptr;
	selectionObject = nullptr;
	pushableCube = nullptr;
	pressurePlate = nullptr;
	gateLeftObject = nullptr;
	gateRightObject = nullptr;
	gateOpen = false;
	gateAnimT = 0.0f;
	enemyAgent = EnemyAgent();
	physics.UseGravity(true);
	BuildSlopeScene();
}
/**
 * @brief 构建一个使用球体网格进行图形显示，并使用边界球体进行刚体表示的游戏对象。
 * @details 这和立方体函数将让你构建很多‘简单’的物理世界。你可能还需要另一个函数来创建OBB立方体。
 * @param position 球体的位置。
 * @param radius 球体的半径。
 * @param inverseMass 球体的逆质量。
 * @return 创建的球体游戏对象。
 */
GameObject* TutorialGame::AddSphereToWorld(const Vector3& position,
	float radius,
	float inverseMass,
	Rendering::Mesh* mesh,
	const GameTechMaterial* material) {
	GameObject* sphere = new GameObject();

	Vector3 sphereSize = Vector3(radius, radius, radius);
	SphereVolume* volume = new SphereVolume(radius);
	sphere->SetBoundingVolume(volume);
	sphere->GetTransform().SetScale(sphereSize).SetPosition(position);
	// ---------- 默认资源处理 ----------
	if (!mesh) {
		mesh = sphereMesh;   // class 成员变量
	}
	const GameTechMaterial& usedMaterial =
		material ? *material : checkerMaterial;  // class 成员变量
	sphere->SetRenderObject(new RenderObject(sphere->GetTransform(),mesh,usedMaterial));
	sphere->SetPhysicsObject(new PhysicsObject(sphere->GetTransform(), sphere->GetBoundingVolume()));
	sphere->GetPhysicsObject()->SetInverseMass(inverseMass);
	sphere->GetPhysicsObject()->InitSphereInertia();
	world.AddGameObject(sphere);
	return sphere;
}


/**
 * @brief 构建一个使用AABB体积的立方体游戏对象。
 * @param position 立方体的位置。
 * @param dimensions 立方体的尺寸。
 * @param inverseMass 立方体的逆质量。
 * @return 创建的立方体游戏对象。
 */
GameObject* TutorialGame::AddCubeToWorld(const Vector3& position, Vector3 dimensions, float inverseMass, 
	Rendering::Mesh* mesh,
	const GameTechMaterial* material) {

	GameObject* cube = new GameObject();
	AABBVolume* volume = new AABBVolume(dimensions);
	cube->SetBoundingVolume(volume);
	cube->GetTransform().SetPosition(position).SetScale(dimensions * 2.0f);
	if (!mesh) {
		mesh = cubeMesh;  
	}
	const GameTechMaterial& usedMaterial =
		material ? *material : checkerMaterial;  // class 成员变量
	cube->SetRenderObject(new RenderObject(cube->GetTransform(), mesh, usedMaterial));
	cube->SetPhysicsObject(new PhysicsObject(cube->GetTransform(), cube->GetBoundingVolume()));

	cube->GetPhysicsObject()->SetInverseMass(inverseMass);
	cube->GetPhysicsObject()->InitCubeInertia();

	world.AddGameObject(cube);

	return cube;
}

GameObject* TutorialGame::AddOBBCubeToWorld(const Vector3& position, Vector3 dimensions, const Quaternion& orientation, float inverseMass) {
	GameObject* cube = new GameObject();

	OBBVolume* volume = new OBBVolume(dimensions);
	cube->SetBoundingVolume(volume);

	cube->GetTransform()
		.SetPosition(position)
		.SetOrientation(orientation)
		.SetScale(dimensions * 2.0f);

	cube->SetRenderObject(new RenderObject(cube->GetTransform(), cubeMesh, checkerMaterial));
	cube->SetPhysicsObject(new PhysicsObject(cube->GetTransform(), cube->GetBoundingVolume()));

	cube->GetPhysicsObject()->SetInverseMass(inverseMass);
	cube->GetPhysicsObject()->InitCubeInertia();

	world.AddGameObject(cube);

	return cube;
}
/**
 * @brief 初始化游戏中的一些示例对象，如玩家、敌人和奖励品。
 */
//void TutorialGame::InitGameExamples() {
//	playerObject = AddPlayerToWorld(Vector3(0, 5, 0));
//}



void TutorialGame::UpdateGateAndPlate(float dt) {
	(void)dt;

	bool platePressed = false;
	if (pushableCube && pressurePlate) {
		Vector3 platePos = pressurePlate->GetTransform().GetPosition();
		Vector3 cubePos = pushableCube->GetTransform().GetPosition();
		Vector3 delta = cubePos - platePos;

		bool overlapX = std::abs(delta.x) <= (pushCubeHalfSize.x + plateHalfSize.x);
		bool overlapZ = std::abs(delta.z) <= (pushCubeHalfSize.z + plateHalfSize.z);
		bool overlapY = (cubePos.y - pushCubeHalfSize.y) <= (platePos.y + plateHalfSize.y + 0.4f);

		if (overlapX && overlapZ && overlapY) {
			platePressed = true;
		}
	}

	if (platePressed) {
		gateOpen = true; // latch open once activated
	}

	if (pressurePlate && pressurePlate->GetRenderObject()) {
		Vector4 color = platePressed ? Vector4(0.2f, 1.0f, 0.2f, 1.0f) : Vector4(0.2f, 0.8f, 0.2f, 1.0f);
		pressurePlate->GetRenderObject()->SetColour(color);
	}

	if (gateLeftObject && gateRightObject) {
		float target = gateOpen ? 1.0f : 0.0f;
		float speed = 2.0f;
		if (gateAnimT < target) {
			gateAnimT = std::min(target, gateAnimT + speed * dt);
		}
		else if (gateAnimT > target) {
			gateAnimT = std::max(target, gateAnimT - speed * dt);
		}

		Vector3 leftPos = gateLeftClosedPos * (1.0f - gateAnimT) + gateLeftOpenPos * gateAnimT;
		Vector3 rightPos = gateRightClosedPos * (1.0f - gateAnimT) + gateRightOpenPos * gateAnimT;
		gateLeftObject->GetTransform().SetPosition(leftPos);
		gateRightObject->GetTransform().SetPosition(rightPos);
		if (gateLeftObject->GetRenderObject()) {
			gateLeftObject->GetRenderObject()->SetColour(gateOpen ? Vector4(0.2f, 0.5f, 1.0f, 1.0f) : Vector4(0.2f, 0.2f, 0.8f, 1.0f));
		}
		if (gateRightObject->GetRenderObject()) {
			gateRightObject->GetRenderObject()->SetColour(gateOpen ? Vector4(0.2f, 0.5f, 1.0f, 1.0f) : Vector4(0.2f, 0.2f, 0.8f, 1.0f));
		}
	}
}
/**
 * @brief 处理对象的选择。
 * @details 每一帧，这段代码都会让你执行一次光线投射，看看光标下是否有物体，如果有，就将其“选择”到一个指针中，以便稍后进行操作。按 Q 键可以在此行为和移动相机之间切换。
 * @return 如果选择了新对象，则返回true。
 */
bool TutorialGame::SelectObject() {
	if (Window::GetKeyboard()->KeyPressed(KeyCodes::Q)) {
		inSelectionMode = !inSelectionMode;
		if (inSelectionMode) {
			Window::GetWindow()->ShowOSPointer(true);
			Window::GetWindow()->LockMouseToWindow(false);
		}
		else {
			Window::GetWindow()->ShowOSPointer(false);
			Window::GetWindow()->LockMouseToWindow(true);
		}
	}
	if (inSelectionMode) {
		Debug::Print("Press Q to change to camera mode!", Vector2(5, 85));

		if (Window::GetMouse()->ButtonDown(NCL::MouseButtons::Left)) {
			if (selectionObject) {	//set colour to deselected;
				selectionObject->GetRenderObject()->SetColour(Vector4(1, 1, 1, 1));
				selectionObject = nullptr;
			}

			Ray ray = CollisionDetection::BuildRayFromMouse(world.GetMainCamera());

			RayCollision closestCollision;
			if (world.Raycast(ray, closestCollision, true)) {
				selectionObject = (GameObject*)closestCollision.node;

				selectionObject->GetRenderObject()->SetColour(Vector4(0, 1, 0, 1));
				return true;
			}
			else {
				return false;
			}
		}
		if (Window::GetKeyboard()->KeyPressed(NCL::KeyCodes::L)) {
			if (selectionObject) {
				if (lockedObject == selectionObject) {
					lockedObject = nullptr;
				}
				else {
					lockedObject = selectionObject;
				}
			}
		}
	}
	else {
		Debug::Print("Press Q to change to select mode!", Vector2(5, 85));
	}
	return false;
}
/**
 * @brief 处理锁定对象的移动逻辑。
 */
void TutorialGame::LockedObjectMovement() {
	Matrix4 view = world.GetMainCamera().BuildViewMatrix();
	Matrix4 camWorld = Matrix::Inverse(view);

	Vector3 rightAxis = Vector3(camWorld.GetColumn(0)); //view is inverse of model!

	//forward is more tricky -  camera forward is 'into' the screen...
	//so we can take a guess, and use the cross of straight up, and
	//the right axis, to hopefully get a vector that's good enough! 1

	Vector3 fwdAxis = Vector::Cross(Vector3(0, 1, 0), rightAxis);
	fwdAxis.y = 0.0f;
	fwdAxis = Vector::Normalise(fwdAxis);

	if (Window::GetKeyboard()->KeyDown(KeyCodes::UP)) {
		selectionObject->GetPhysicsObject()->AddForce(fwdAxis);
	}

	if (Window::GetKeyboard()->KeyDown(KeyCodes::DOWN)) {
		selectionObject->GetPhysicsObject()->AddForce(-fwdAxis);
	}

	if (Window::GetKeyboard()->KeyDown(KeyCodes::NEXT)) {
		selectionObject->GetPhysicsObject()->AddForce(Vector3(0, -10, 0));
	}
}

/**
 * @brief 处理用于调试的对象移动，允许通过键盘直接对选定对象施加力和扭矩。
 */
void TutorialGame::DebugObjectMovement() {
	//If we've selected an object, we can manipulate it with some key presses
	if (inSelectionMode && selectionObject) {
		//Twist the selected object!
		if (Window::GetKeyboard()->KeyDown(KeyCodes::LEFT)) {
			selectionObject->GetPhysicsObject()->AddTorque(Vector3(-10, 0, 0));
		}

		if (Window::GetKeyboard()->KeyDown(KeyCodes::RIGHT)) {
			selectionObject->GetPhysicsObject()->AddTorque(Vector3(10, 0, 0));
		}

		if (Window::GetKeyboard()->KeyDown(KeyCodes::NUM7)) {
			selectionObject->GetPhysicsObject()->AddTorque(Vector3(0, 10, 0));
		}

		if (Window::GetKeyboard()->KeyDown(KeyCodes::NUM8)) {
			selectionObject->GetPhysicsObject()->AddTorque(Vector3(0, -10, 0));
		}

		if (Window::GetKeyboard()->KeyDown(KeyCodes::RIGHT)) {
			selectionObject->GetPhysicsObject()->AddTorque(Vector3(10, 0, 0));
		}

		if (Window::GetKeyboard()->KeyDown(KeyCodes::UP)) {
			selectionObject->GetPhysicsObject()->AddForce(Vector3(0, 0, -10));
		}

		if (Window::GetKeyboard()->KeyDown(KeyCodes::DOWN)) {
			selectionObject->GetPhysicsObject()->AddForce(Vector3(0, 0, 10));
		}

		if (Window::GetKeyboard()->KeyDown(KeyCodes::NUM5)) {
			selectionObject->GetPhysicsObject()->AddForce(Vector3(0, -10, 0));
		}
	}
}

/**
 * @brief 处理玩家的移动输入和物理。
 * @param dt 帧时间增量。
 */
void TutorialGame::HandlePlayerMovement(float dt) {
	if (!playerObject) {
		return;
	}
	PhysicsObject* phys = playerObject->GetPhysicsObject();
	if (!phys) {
		return;
	}
	// Ground check via short raycast downward
	bool grounded = IsPlayerGrounded();
	Matrix4 view = world.GetMainCamera().BuildViewMatrix();
	Matrix4 camWorld = Matrix::Inverse(view);
	Vector3 rightAxis = Vector3(camWorld.GetColumn(0));
	Vector3 fwdAxis = Vector::Cross(Vector3(0, 1, 0), rightAxis);
	rightAxis.y = 0.0f;
	fwdAxis.y = 0.0f;
	rightAxis = Vector::Normalise(rightAxis);
	fwdAxis = Vector::Normalise(fwdAxis);
	Vector3 moveDir;
	auto* keyboard = Window::GetKeyboard();
	if (keyboard->KeyDown(KeyCodes::W)) {
		moveDir += fwdAxis;
	}
	if (keyboard->KeyDown(KeyCodes::S)) {
		moveDir -= fwdAxis;
	}
	if (keyboard->KeyDown(KeyCodes::A)) {
		moveDir -= rightAxis;
	}
	if (keyboard->KeyDown(KeyCodes::D)) {
		moveDir += rightAxis;
	}
	if (Vector::LengthSquared(moveDir) > 0.0f) {
		moveDir = Vector::Normalise(moveDir);
		phys->AddForce(moveDir * playerMoveForce);
	}
	// Jump only when grounded
	if (grounded && keyboard->KeyPressed(KeyCodes::SPACE)) {
		phys->ApplyLinearImpulse(Vector3(0, playerJumpImpulse, 0));
	}
	Vector3 velocity = phys->GetLinearVelocity();
	// Apply planar friction
	Vector3 horizVel = Vector3(velocity.x, 0.0f, velocity.z);
	if (Vector::LengthSquared(horizVel) > 0.0f) {
		float friction = grounded ? groundFriction : airFriction;
		// Treat friction as per-second damping; raise to dt to make it framerate-independent
		float frictionFactor = std::pow(friction, dt);
		horizVel *= frictionFactor;
		velocity.x = horizVel.x;
		velocity.z = horizVel.z;
	}
	float speed = Vector::Length(velocity);
	if (speed > playerMaxSpeed && speed > 0.0f) {
		phys->SetLinearVelocity(Vector::Normalise(velocity) * playerMaxSpeed);
	}
	else {
		phys->SetLinearVelocity(velocity);
	}
}
void TutorialGame::HandleGrab() {
	auto releaseGrab = [&]() {
		if (grabbedObject) {
			if (grabbedObject->GetRenderObject()) {
				grabbedObject->GetRenderObject()->SetColour(Vector4(1, 1, 1, 1));
			}
		}
		grabbedObject = nullptr;
		grabLocalOffset = Vector3();
		grabDistance = 0.0f;
		};

	bool rightDown = Window::GetMouse()->ButtonDown(NCL::MouseButtons::Right);

	if (!inSelectionMode && !selectionObject) {
		if (!grabbedObject && Window::GetMouse()->ButtonPressed(NCL::MouseButtons::Right) && playerObject) {
			Vector3 origin = playerObject->GetTransform().GetPosition();
			Vector3 forward = playerObject->GetTransform().GetOrientation() * Vector3(0, 0, 1);
			forward = Vector::Normalise(forward);
			Ray ray(origin, forward);
			RayCollision hit;
			Debug::DrawLine(origin, origin + forward * 100.0f, Vector4(1, 0, 0, 1));
			if (world.Raycast(ray, hit, true, playerObject)) {
				if (hit.rayDistance > grabMaxDistance) {
					return; // 超出距离，不让抓
				}
				GameObject* hitObj = (GameObject*)hit.node;
				PhysicsObject* phys = hitObj ? hitObj->GetPhysicsObject() : nullptr;
				if (phys && phys->GetInverseMass() > 0.0f) {
					grabbedObject = hitObj;
					Quaternion invOrient = grabbedObject->GetTransform().GetOrientation().Conjugate();
					Vector3 local = hit.collidedAt - grabbedObject->GetTransform().GetPosition();
					grabLocalOffset = invOrient * local;
					float desiredGrabDistance = 1.0f;
					grabDistance = desiredGrabDistance;
					//grabDistance = std::min(hit.rayDistance, grabMaxDistance);
					if (grabbedObject->GetRenderObject()) {
						grabbedObject->GetRenderObject()->SetColour(Vector4(1, 0, 0, 1));
					}
				}
			}
		}
	}

	if (grabbedObject) {
		if (!rightDown) {
			releaseGrab();
		}
		else {
			Vector3 origin = playerObject->GetTransform().GetPosition();
			Vector3 forward = playerObject->GetTransform().GetOrientation() * Vector3(0, 0, 1);
			forward = Vector::Normalise(forward);

			Vector3 targetPos = origin + forward * grabDistance;
			Vector3 objPos = grabbedObject->GetTransform().GetPosition();
			Vector3 worldAnchor = objPos + grabbedObject->GetTransform().GetOrientation() * grabLocalOffset;

			Vector3 error = targetPos - worldAnchor;
			PhysicsObject* phys = grabbedObject->GetPhysicsObject();
			if (phys) {
				Vector3 force = error * grabSpring - phys->GetLinearVelocity() * grabDamping;
				phys->AddForceAtPosition(force, worldAnchor);
			}

			Debug::DrawLine(origin, targetPos, Vector4(1, 0, 0, 1));
			Debug::DrawLine(worldAnchor, targetPos, Vector4(0, 1, 0, 1));
		}
	}
}

void TutorialGame::InitEnemyAgent(const Vector3& pos) {
	if (enemyAgent.object) {
		return;
	}
	enemyAgent.object = AddEnemyToWorld(pos);
	enemyAgent.lastPos = pos;
	enemyAgent.lastPlayerPos = pos;
	enemyAgent.idleState = new State([this](float dt) { UpdateEnemyIdle(dt); });
	enemyAgent.chaseState = new State([this](float dt) { UpdateEnemyChase(dt); });
	enemyAgent.recoverState = new State([this](float dt) { UpdateEnemyRecover(dt); });
	enemyAgent.stateMachine.AddState(enemyAgent.idleState);
	enemyAgent.stateMachine.AddState(enemyAgent.chaseState);
	enemyAgent.stateMachine.AddState(enemyAgent.recoverState);
	enemyAgent.stateMachine.AddTransition(new StateTransition(enemyAgent.idleState, enemyAgent.chaseState, [this]() {
		if (!playerObject || !enemyAgent.object) {
			return false;
		}
		float dist = Vector::Length(playerObject->GetTransform().GetPosition() - enemyAgent.object->GetTransform().GetPosition());
		return dist < enemyChaseDistance;
	}));
	enemyAgent.stateMachine.AddTransition(new StateTransition(enemyAgent.chaseState, enemyAgent.idleState, [this]() {
		if (!playerObject || !enemyAgent.object) {
			return true;
		}
		float dist = Vector::Length(playerObject->GetTransform().GetPosition() - enemyAgent.object->GetTransform().GetPosition());
		return dist > enemyLoseDistance;
	}));
	enemyAgent.stateMachine.AddTransition(new StateTransition(enemyAgent.chaseState, enemyAgent.recoverState, [this]() {
		return enemyAgent.requestRecover;
	}));
	enemyAgent.stateMachine.AddTransition(new StateTransition(enemyAgent.recoverState, enemyAgent.chaseState, [this]() {
		if (!playerObject || !enemyAgent.object) {
			return false;
		}
		float dist = Vector::Length(playerObject->GetTransform().GetPosition() - enemyAgent.object->GetTransform().GetPosition());
		return enemyAgent.recoverTimer >= enemyRecoverDuration && dist < enemyLoseDistance;
	}));
	enemyAgent.stateMachine.AddTransition(new StateTransition(enemyAgent.recoverState, enemyAgent.idleState, [this]() {
		if (!playerObject || !enemyAgent.object) {
			return enemyAgent.recoverTimer >= enemyRecoverDuration;
		}
		float dist = Vector::Length(playerObject->GetTransform().GetPosition() - enemyAgent.object->GetTransform().GetPosition());
		return enemyAgent.recoverTimer >= enemyRecoverDuration && dist >= enemyLoseDistance;
	}));
	OnEnemyStateChanged(enemyAgent.stateMachine.GetActiveState());
}

void TutorialGame::ResetEnemyPath() {
	enemyAgent.path.clear();
	enemyAgent.pathIndex = 0;
	enemyAgent.hasPath = false;
	enemyAgent.pathTimer = 0.0f;
}

bool TutorialGame::BuildEnemyPathToPlayer() {
	if (!navMesh || !enemyAgent.object || !playerObject) {
		return false;
	}
	NavigationPath newPath;
	Vector3 from = enemyAgent.object->GetTransform().GetPosition();
	Vector3 to = playerObject->GetTransform().GetPosition();
	if (!navMesh->FindPath(from, to, newPath)) {
		enemyAgent.hasPath = false;
		return false;
	}
	enemyAgent.path.clear();
	Vector3 waypoint;
	while (newPath.PopWaypoint(waypoint)) {
		enemyAgent.path.emplace_back(waypoint);
	}
	if (enemyAgent.path.size() > 1) {
		enemyAgent.pathIndex = 1; // skip start node (at enemy position), go to next
	} else {
		enemyAgent.pathIndex = 0;
	}
	enemyAgent.hasPath = !enemyAgent.path.empty();
	enemyAgent.pathTimer = 0.0f;
	enemyAgent.lastPlayerPos = playerObject->GetTransform().GetPosition();
	return enemyAgent.hasPath;
}

void TutorialGame::OnEnemyStateChanged(State* newState) {
	if (newState == enemyAgent.recoverState) {
		enemyAgent.recoverTimer = 0.0f;
		enemyAgent.requestRecover = false;
		ResetEnemyPath();
	}
	if (newState == enemyAgent.idleState) {
		enemyAgent.recoverTimer = 0.0f;
		enemyAgent.requestRecover = false;
		ResetEnemyPath();
	}
	if (newState == enemyAgent.chaseState) {
		enemyAgent.recoverTimer = 0.0f;
		enemyAgent.requestRecover = false;
		enemyAgent.stuckTimer = 0.0f;
		if (!enemyAgent.hasPath) {
			BuildEnemyPathToPlayer();
		}
	}
}

void TutorialGame::UpdateEnemyIdle(float dt) {
	(void)dt;
	if (!enemyAgent.object) {
		return;
	}
	enemyAgent.requestRecover = false;
	ResetEnemyPath();
}

void TutorialGame::UpdateEnemyChase(float dt) {
	if (!enemyAgent.object || !playerObject) {
		return;
	}
	enemyAgent.pathTimer += dt;
	Vector3 playerPos = playerObject->GetTransform().GetPosition();
	if (!enemyAgent.hasPath || enemyAgent.pathTimer >= enemyAgent.pathRefreshTime ||
		Vector::Length(playerPos - enemyAgent.lastPlayerPos) > enemyRepathPlayerDelta) {
		if (!BuildEnemyPathToPlayer()) {
			enemyAgent.requestRecover = true;
			return;
		}
	}
	PhysicsObject* phys = enemyAgent.object->GetPhysicsObject();
	if (!phys) {
		return;
	}
	Vector3 pos = enemyAgent.object->GetTransform().GetPosition();
	Vector3 target = playerPos;
	if (enemyAgent.pathIndex < enemyAgent.path.size()) {
		target = enemyAgent.path[enemyAgent.pathIndex];
	}
	Vector3 toTarget = target - pos;
	toTarget.y = 0.0f; // navmesh is flat on floor; prevent flying toward elevated targets
	float dist = Vector::Length(toTarget);
	if (dist < enemyWaypointTolerance && enemyAgent.pathIndex + 1 < enemyAgent.path.size()) {
		enemyAgent.pathIndex++;
		target = enemyAgent.path[enemyAgent.pathIndex];
		toTarget = target - pos;
		toTarget.y = 0.0f;
	}
	if (Vector::LengthSquared(toTarget) > 1e-4f) {
		Vector3 dir = Vector::Normalise(toTarget);
		float step = enemyMoveSpeed * dt;
		Vector3 newPos = pos + dir * step;
		// Clamp to target if overshoot
		if (Vector::Length(newPos - pos) > dist) {
			newPos = target;
		}
		enemyAgent.object->GetTransform().SetPosition(newPos);
		phys->SetLinearVelocity(Vector3());
	}
	float moved = Vector::Length(pos - enemyAgent.lastPos);
	if (moved < enemyStuckMoveEpsilon) {
		enemyAgent.stuckTimer += dt;
		if (enemyAgent.stuckTimer > enemyStuckTime) {
			enemyAgent.requestRecover = true;
		}
	}
	else {
		enemyAgent.stuckTimer = 0.0f;
		enemyAgent.lastPos = pos;
	}
	// Debug draw path
	if (enemyAgent.pathIndex < enemyAgent.path.size()) {
		Vector3 prev = pos;
		for (size_t i = enemyAgent.pathIndex; i < enemyAgent.path.size(); ++i) {
			Vector3 wp = enemyAgent.path[i];
			Debug::DrawLine(prev, wp, Vector4(1, 0, 0, 1));
			Debug::DrawLine(wp, wp + Vector3(0, 0.5f, 0), Vector4(0, 1, 0, 1)); // small marker
			prev = wp;
		}
		Debug::Print("Enemy path len: " + std::to_string(enemyAgent.path.size() - enemyAgent.pathIndex), Vector2(5, 70), Debug::RED);
	}
}

void TutorialGame::UpdateEnemyRecover(float dt) {
	if (!enemyAgent.object) {
		return;
	}
	enemyAgent.recoverTimer += dt;
	enemyAgent.requestRecover = false;
	PhysicsObject* phys = enemyAgent.object->GetPhysicsObject();
	if (phys) {
		Vector3 vel = phys->GetLinearVelocity();
		vel *= 0.9f;
		phys->SetLinearVelocity(vel);
	}
	if (!enemyAgent.hasPath && enemyAgent.recoverTimer > enemyRecoverDuration * 0.5f) {
		BuildEnemyPathToPlayer();
	}
}

void TutorialGame::UpdateEnemyAI(float dt) {
	if (!enemyAgent.object || !playerObject || !navMesh) {
		return;
	}
	State* before = enemyAgent.stateMachine.GetActiveState();
	enemyAgent.stateMachine.Update(dt);
	State* after = enemyAgent.stateMachine.GetActiveState();
	if (after != before) {
		OnEnemyStateChanged(after);
	}
}
GameObject* TutorialGame::AddFloorToWorld(const Vector3& position) {
	Vector3 floorSize = Vector3(30, 0.5f, 30);
	return AddCubeToWorld(position, floorSize, 0.0f);
}

GameObject* TutorialGame::AddPlayerToWorld(const Vector3& position) {
	return AddSphereToWorld(position,1.0f,0.5f,catMesh,&notexMaterial);
}
GameObject* TutorialGame::AddEnemyToWorld(const Vector3& position) {
	float meshSize = 2.0f;
	Vector3 halfDims = Vector3(1,1,1) * meshSize;
	return AddCubeToWorld(position,halfDims,0.5f,enemyMesh,&notexMaterial);
}
GameObject* TutorialGame::AddCoinToWorld(const Vector3& position) {
	float radius = 0.20f; // sphere half height = radius
	Vector3 spawn = position;
	// Ensure it rests on default floor (half-height 1.0) instead of embedding
	float floorTop = 1.2f;
	if (spawn.y < floorTop + radius) {
		spawn.y = floorTop + radius + 0.05f;
	}
	return AddSphereToWorld(spawn, radius, 0.0f, coinMesh, &notexMaterial); // static coin, no gravity
}

void TutorialGame::BuildSlopeScene() {
	AddFloorToWorld(Vector3(0, 0, 50)); //地板
	// 放在地板上方，半径 0.25，高度余量 0.05
	AddCoinToWorld(Vector3(0, 1.3f, 40)); 
	// High platform for spawn
	Vector3 platformHalfSize = Vector3(12.0f, 2.0f, 12.0f);
	Vector3 platformPos = Vector3(0.0f, 12.0f, -30.0f);
	AddCubeToWorld(platformPos, platformHalfSize, 0.0f);

	// Slope connecting platform to ground
	Vector3 slopeHalfSize = Vector3(8.0f, 1.0f, 15.0f);
	Vector3 slopePos = Vector3(0.0f, 7.0f, -5.0f);
	Quaternion slopeRot = Quaternion::AxisAngleToQuaterion(Vector3(1, 0, 0), 25.0f);
	AddOBBCubeToWorld(slopePos, slopeHalfSize, slopeRot, 0.0f); //斜坡

	// Pushable cube on the platform center
	pushableCube = AddSphereToWorld(platformPos + Vector3(0.0f, platformHalfSize.y + pushCubeHalfSize.y, 0.0f),1.0f , 1.0f);

	// Pressure plate at ramp bottom
	Vector3 platePos = Vector3(-1.0f, 1.0f, 12.0f);
	pressurePlate = AddOBBCubeToWorld(platePos, plateHalfSize, Quaternion::AxisAngleToQuaterion(Vector3(0, 1, 0), 0.0f), 0.0f);
	if (pressurePlate && pressurePlate->GetRenderObject()) {
		pressurePlate->GetRenderObject()->SetColour(Vector4(0.2f, 0.8f, 0.2f, 1.0f));
	}

	// Gate mechanism in front of plate	
	Vector3 gateCenter = Vector3(0.0f, gateHalfSize.y, 20.0f);
	float gateGap = 0.5f;
	float gateOffset = gateHalfSize.x + gateGap * 0.5f;
	gateLeftClosedPos = gateCenter + Vector3(-gateOffset, 0.0f, 0.0f);
	gateRightClosedPos = gateCenter + Vector3(gateOffset, 0.0f, 0.0f);
	gateLeftOpenPos = gateLeftClosedPos + Vector3(-gateSlideDistance, 0.0f, 0.0f);
	gateRightOpenPos = gateRightClosedPos + Vector3(gateSlideDistance, 0.0f, 0.0f);

	gateLeftObject = AddCubeToWorld(gateLeftClosedPos, gateHalfSize, 0.0f);
	gateRightObject = AddCubeToWorld(gateRightClosedPos, gateHalfSize, 0.0f);
	if (gateLeftObject && gateLeftObject->GetRenderObject()) {
		gateLeftObject->GetRenderObject()->SetColour(Vector4(0.2f, 0.2f, 0.8f, 1.0f));
	}
	if (gateRightObject && gateRightObject->GetRenderObject()) {
		gateRightObject->GetRenderObject()->SetColour(Vector4(0.2f, 0.2f, 0.8f, 1.0f));
	}

	// Spawn player on the platform
	float spawnOffsetY = platformHalfSize.y + playerRadius + 4.5f;
	playerObject = AddPlayerToWorld(platformPos + Vector3(-10.0f, spawnOffsetY, 0.0f));
	InitEnemyAgent(Vector3(40.0f, 5.5f, 5.0f));
}
