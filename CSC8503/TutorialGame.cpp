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
#include <algorithm>

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
	useGravity		= false;
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

	if (Window::GetKeyboard()->KeyPressed(KeyCodes::G)) {
		useGravity = !useGravity; //Toggle gravity!
		physics.UseGravity(useGravity);
	}
	//Running certain physics updates in a consistent order might cause some
	//bias in the calculations - the same objects might keep 'winning' the constraint
	//allowing the other one to stretch too much etc. Shuffling the order so that it
	//is random every frame can help reduce such bias.


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

	RayCollision closestCollision;
	if (Window::GetKeyboard()->KeyPressed(KeyCodes::K) && selectionObject) {
		Vector3 rayPos;
		Vector3 rayDir;
		// 从选定对象的位置和朝向发射射线
		rayDir = selectionObject->GetTransform().GetOrientation() * Vector3(0, 0, -1);

		rayPos = selectionObject->GetTransform().GetPosition();

		Ray r = Ray(rayPos, rayDir);

		if (world.Raycast(r, closestCollision, true, selectionObject)) {
			// 恢复上一个高亮对象的颜色
			if (objClosest) {
				objClosest->GetRenderObject()->SetColour(Vector4(1, 1, 1, 1));
			}
			// 高亮新的被射线击中的对象
			objClosest = (GameObject*)closestCollision.node;

			objClosest->GetRenderObject()->SetColour(Vector4(1, 0, 1, 1));
		}
	}

	//This year we can draw debug textures as well!
	//Debug::DrawTex(*defaultTex, Vector2(10, 10), Vector2(5, 5), Debug::WHITE);
	//Debug::DrawLine(Vector3(), Vector3(0, 100, 0), Vector4(1, 0, 0, 1));
	Debug::Print("FPS: " + std::to_string(fps), Vector2(5, 80), Debug::YELLOW);
	if (useGravity) {
		Debug::Print("(G)ravity on", Vector2(5, 95), Debug::RED);
	}
	else {
		Debug::Print("(G)ravity off", Vector2(5, 95), Debug::RED);
	}

	SelectObject(); // 处理对象选择
	MoveSelectedObject();	 // 移动选定的对象
	HandlePlayerMovement(dt); // 处理玩家输入移动
	
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

	CreatedMixedGrid(15, 15, 3.5f, 3.5f);

	InitGameExamples();

	AddFloorToWorld(Vector3(0, -20, 0));
}

/**
 * @brief 向世界添加一个大型不可移动的立方体作为地面。
 * @param position 地面的位置。
 * @return 创建的地面游戏对象。
 */
GameObject* TutorialGame::AddFloorToWorld(const Vector3& position) {
	GameObject* floor = new GameObject();

	Vector3 floorSize = Vector3(200, 2, 200);
	AABBVolume* volume = new AABBVolume(floorSize);
	floor->SetBoundingVolume(volume);
	floor->GetTransform()
		.SetScale(floorSize * 2.0f)
		.SetPosition(position);

	floor->SetRenderObject(new RenderObject(floor->GetTransform(), cubeMesh, checkerMaterial));
	floor->SetPhysicsObject(new PhysicsObject(floor->GetTransform(), floor->GetBoundingVolume()));

	floor->GetPhysicsObject()->SetInverseMass(0);
	floor->GetPhysicsObject()->InitCubeInertia();

	world.AddGameObject(floor);

	return floor;
}

/**
 * @brief 构建一个使用球体网格进行图形显示，并使用边界球体进行刚体表示的游戏对象。
 * @details 这和立方体函数将让你构建很多‘简单’的物理世界。你可能还需要另一个函数来创建OBB立方体。
 * @param position 球体的位置。
 * @param radius 球体的半径。
 * @param inverseMass 球体的逆质量。
 * @return 创建的球体游戏对象。
 */
GameObject* TutorialGame::AddSphereToWorld(const Vector3& position, float radius, float inverseMass) {
	GameObject* sphere = new GameObject();

	Vector3 sphereSize = Vector3(radius, radius, radius);
	SphereVolume* volume = new SphereVolume(radius);
	sphere->SetBoundingVolume(volume);

	sphere->GetTransform()
		.SetScale(sphereSize)
		.SetPosition(position);

	sphere->SetRenderObject(new RenderObject(sphere->GetTransform(), sphereMesh, checkerMaterial));
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
GameObject* TutorialGame::AddCubeToWorld(const Vector3& position, Vector3 dimensions, float inverseMass) {
	GameObject* cube = new GameObject();

	AABBVolume* volume = new AABBVolume(dimensions);
	cube->SetBoundingVolume(volume);

	cube->GetTransform()
		.SetPosition(position)
		.SetScale(dimensions * 2.0f);

	cube->SetRenderObject(new RenderObject(cube->GetTransform(), cubeMesh, checkerMaterial));
	cube->SetPhysicsObject(new PhysicsObject(cube->GetTransform(), cube->GetBoundingVolume()));

	cube->GetPhysicsObject()->SetInverseMass(inverseMass);
	cube->GetPhysicsObject()->InitCubeInertia();

	world.AddGameObject(cube);

	return cube;
}

/**
 * @brief 向世界添加一个玩家角色。
 * @param position 玩家的位置。
 * @return 创建的玩家游戏对象。
 */
GameObject* TutorialGame::AddPlayerToWorld(const Vector3& position) {
	float meshSize		= 1.0f;
	float inverseMass	= 0.5f;

	GameObject* character = new GameObject();
	SphereVolume* volume  = new SphereVolume(1.0f);

	character->SetBoundingVolume(volume);

	character->GetTransform()
		.SetScale(Vector3(meshSize, meshSize, meshSize))
		.SetPosition(position);

	character->SetRenderObject(new RenderObject(character->GetTransform(), catMesh, notexMaterial));
	character->SetPhysicsObject(new PhysicsObject(character->GetTransform(), character->GetBoundingVolume()));

	character->GetPhysicsObject()->SetInverseMass(inverseMass);
	character->GetPhysicsObject()->InitSphereInertia();

	world.AddGameObject(character);

	return character;
}

/**
 * @brief 向世界添加一个敌人角色。
 * @param position 敌人的位置。
 * @return 创建的敌人游戏对象。
 */
GameObject* TutorialGame::AddEnemyToWorld(const Vector3& position) {
	float meshSize		= 3.0f;
	float inverseMass	= 0.5f;

	GameObject* character = new GameObject();

	AABBVolume* volume = new AABBVolume(Vector3(0.3f, 0.9f, 0.3f) * meshSize);
	character->SetBoundingVolume(volume);

	character->GetTransform()
		.SetScale(Vector3(meshSize, meshSize, meshSize))
		.SetPosition(position);

	character->SetRenderObject(new RenderObject(character->GetTransform(), enemyMesh, notexMaterial));
	character->SetPhysicsObject(new PhysicsObject(character->GetTransform(), character->GetBoundingVolume()));

	character->GetPhysicsObject()->SetInverseMass(inverseMass);
	character->GetPhysicsObject()->InitSphereInertia();

	world.AddGameObject(character);

	return character;
}

/**
 * @brief 向世界添加一个奖励物品。
 * @param position 奖励物品的位置。
 * @return 创建的奖励物品游戏对象。
 */
GameObject* TutorialGame::AddBonusToWorld(const Vector3& position) {
	GameObject* apple = new GameObject();

	SphereVolume* volume = new SphereVolume(0.5f);
	apple->SetBoundingVolume(volume);
	apple->GetTransform()
		.SetScale(Vector3(2, 2, 2))
		.SetPosition(position);

	apple->SetRenderObject(new RenderObject(apple->GetTransform(), bonusMesh, glassMaterial));
	apple->SetPhysicsObject(new PhysicsObject(apple->GetTransform(), apple->GetBoundingVolume()));

	apple->GetPhysicsObject()->SetInverseMass(1.0f);
	apple->GetPhysicsObject()->InitSphereInertia();

	world.AddGameObject(apple);

	return apple;
}

/**
 * @brief 初始化游戏中的一些示例对象，如玩家、敌人和奖励品。
 */
void TutorialGame::InitGameExamples() {
	playerObject = AddPlayerToWorld(Vector3(0, 5, 0));
	AddEnemyToWorld(Vector3(5, 5, 0));
	AddBonusToWorld(Vector3(10, 5, 0));
}

/**
 * @brief 创建一个由球体组成的网格。
 * @param numRows 网格的行数。
 * @param numCols 网格的列数。
 * @param rowSpacing 行间距。
 * @param colSpacing 列间距。
 * @param radius 球体的半径。
 */
void TutorialGame::CreateSphereGrid(int numRows, int numCols, float rowSpacing, float colSpacing, float radius) {
	for (int x = 0; x < numCols; ++x) {
		for (int z = 0; z < numRows; ++z) {
			Vector3 position = Vector3(x * colSpacing, 10.0f, z * rowSpacing);
			AddSphereToWorld(position, radius, 1.0f);
		}
	}
	AddFloorToWorld(Vector3(0, 0, 0));
}

/**
 * @brief 创建一个由立方体和球体混合组成的网格。
 * @param numRows 网格的行数。
 * @param numCols 网格的列数。
 * @param rowSpacing 行间距。
 * @param colSpacing 列间距。
 */
void TutorialGame::CreatedMixedGrid(int numRows, int numCols, float rowSpacing, float colSpacing) {
	float sphereRadius = 1.0f;
	Vector3 cubeDims = Vector3(1, 1, 1);
	AddCubeToWorld(Vector3(20,20,20), cubeDims);
	AddSphereToWorld(Vector3(9,9,9), sphereRadius);
	//for (int x = 0; x < numCols; ++x) {
	//	for (int z = 0; z < numRows; ++z) {
	//		Vector3 position = Vector3(x * colSpacing, 10.0f, z * rowSpacing);

	//		if (rand() % 2) {
	//			AddCubeToWorld(position, cubeDims);
	//		}
	//		else {
	//			AddSphereToWorld(position, sphereRadius);
	//		}
	//	}
	//}
}

/**
 * @brief 创建一个由AABB立方体组成的网格。
 * @param numRows 网格的行数。
 * @param numCols 网格的列数。
 * @param rowSpacing 行间距。
 * @param colSpacing 列间距。
 * @param cubeDims 立方体的尺寸。
 */
void TutorialGame::CreateAABBGrid(int numRows, int numCols, float rowSpacing, float colSpacing, const Vector3& cubeDims) {
	for (int x = 1; x < numCols+1; ++x) {
		for (int z = 1; z < numRows+1; ++z) {
			Vector3 position = Vector3(x * colSpacing, 10.0f, z * rowSpacing);
			AddCubeToWorld(position, cubeDims, 1.0f);
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
 * @brief 移动当前选定的对象。
 * @details 如果一个对象被点击，可以用鼠标右键推动它，推动的量由滚轮决定。在第一个教程中，这不会做任何事情，因为我们还没有将线性运动添加到我们的物理系统中。在第二个教程之后，物体将以直线移动 - 在第三个教程之后，它们还能够在扭矩下扭转。
 */
void TutorialGame::MoveSelectedObject() {
	Debug::Print("Click Force:" + std::to_string(forceMagnitude), Vector2(5, 90));
	forceMagnitude += Window::GetMouse()->GetWheelMovement() * 100.0f;

	if (!selectionObject) {
		return;//we haven't selected anything!
	}
	//Push the selected object!
	if (Window::GetMouse()->ButtonPressed(NCL::MouseButtons::Right)) {
		Ray ray = CollisionDetection::BuildRayFromMouse(world.GetMainCamera());

		RayCollision closestCollision;
		if (world.Raycast(ray, closestCollision, true)) {
			if (closestCollision.node == selectionObject) {
				selectionObject->GetPhysicsObject()->AddForceAtPosition(ray.GetDirection() * forceMagnitude, closestCollision.collidedAt);
			}
		}
	}
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
