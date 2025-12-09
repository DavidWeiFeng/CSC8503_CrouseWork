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
#include <random>
#include <chrono>

using namespace NCL;
using namespace CSC8503;
using namespace NCL::Maths;

/**
 * @brief 閺嬪嫰鐘辩存稉閺傛壆娈戦弫娆戔柤濞撳憡鍨欓妴
 * @param inWorld 濞撳憡鍨欐稉鏍鏅閵
 * @param inRenderer 濞撳憡鍨欏〒鍙夌厠閸ｃ劊
 * @param inPhysics 閻椻晝鎮婄化鑽ょ埠閵
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
 * @brief 閺佹瑧鈻煎〒鍛婂灆閻ㄥ嫭鐎介弸鍕鍤遍弫鑸
 */
TutorialGame::~TutorialGame()	{
	delete enemyAI;
	enemyAI = nullptr;
	delete navMesh;
	navMesh = nullptr;
}

/**
 * @brief 濮ｅ繐鎶氶弴瀛樻煀濞撳憡鍨欓柅鏄忕帆閵
 * @param dt 鐢褎妞傞梻鏉戠偤鍣洪妴
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
	// 闂呭繑婧閸栨牜瀹抽弶鐔笺庢惔 - 閸戝繐鐨鐠侊紕鐣婚崑蹇撴▕
	if (Window::GetKeyboard()->KeyPressed(KeyCodes::F9)) {
		world.ShuffleConstraints(true);
	}
	if (Window::GetKeyboard()->KeyPressed(KeyCodes::F10)) {
		world.ShuffleConstraints(false);
	}
	// 闂呭繑婧閸栨牕纭呰杽妞ゅ搫绨
	if (Window::GetKeyboard()->KeyPressed(KeyCodes::F7)) {
		world.ShuffleObjects(true);
	}
	if (Window::GetKeyboard()->KeyPressed(KeyCodes::F8)) {
		world.ShuffleObjects(false);
	}

	if (lockedObject) {
		LockedObjectMovement(); // 娴ｈ法鏁ょ粻婢舵挳鏁閹貉冨煑闁夸礁鐣剧电呰杽
	}
	else {
		DebugObjectMovement();  // 鐠嬪啳鐦濡鈥崇础閻ㄥ嫬纭呰杽閹垮秳缍
	}
	//This year we can draw debug textures as well!
	//Debug::DrawTex(*defaultTex, Vector2(10, 10), Vector2(5, 5), Debug::WHITE);
	//Debug::DrawLine(Vector3(), Vector3(0, 100, 0), Vector4(1, 0, 0, 1));
	Debug::Print("FPS: " + std::to_string(fps), Vector2(5, 80), Debug::YELLOW);
	if (playerObject) {
		Vector3 pp = playerObject->GetTransform().GetPosition();
		Debug::Print("Player: " + std::to_string(pp.x) + "," + std::to_string(pp.y) + "," + std::to_string(pp.z), Vector2(5, 75), Debug::WHITE);
	}
	Debug::Print("Score: " + std::to_string(playerScore), Vector2(80, 95), Debug::WHITE);
	HandleGrab();
	HandlePlayerMovement(dt); // 婢跺嫮鎮婇悳鈺佹儼绶閸忋儳些閸	
	UpdateGateAndPlate(dt);   // 閺囧瓨鏌婇崢瀣濮忛弶澶哥瑢婢堆囨，
	UpdateEnemyAI(dt);        // 閺囧瓨鏌婇弫灞兼眽AI
	UpdateCoinPickups();      // 閹靛濮╁Λ濞村鍣剧敮浣瑰瑎閸欐牭绱濋柆鍨鍘ら崣宥呰剨
	// 閸欐娊鏁娴犲海甯虹硅埖锝呭犻弬鐟板絺鐏忓嫬鐨犵痪鍖＄礉妤傛ü瀵掗崨鎴掕厬閻ㄥ嫮澧挎担
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
	UpdatePendingRemovals(dt);
	UpdateThirdPersonCamera(0.0f);
}

/**
 * @brief 閸掓繂瀣瀵查幗鍕鍎氶張楦垮墽鐤嗛妴
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
 * @brief 閸掓繂瀣瀵插〒鍛婂灆娑撴牜鏅閿涘本绔婚梽銈嗘＋鐎电呰杽楠炶泛鍨卞ょ儤鏌婇崷鐑樻珯閵
 */
void TutorialGame::InitWorld() {
	// force delete any pending objects before clearing
	for (auto& p : pendingRemoval) {
		delete p.obj;
	}
	pendingRemoval.clear();
	coins.clear();
	world.ClearAndErase();
	physics.Clear();
	playerObject = nullptr;
	playerScore = 0;
	selectionObject = nullptr;
	pushableCube = nullptr;
	pressurePlate = nullptr;
	gateLeftObject = nullptr;
	gateRightObject = nullptr;
	gateOpen = false;
	gateAnimT = 0.0f;
	if (enemyAI) {
		delete enemyAI;
		enemyAI = nullptr;
	}
	enemyObject = nullptr;
	physics.UseGravity(true);
	BuildSlopeScene();
}
/**
 * @brief 閺嬪嫬缂撴稉娑撴担璺ㄦ暏閻炲啩缍嬬純鎴炵壐鏉╂稖灞芥禈瑜般垺妯夌粈鐚寸礉楠炴湹濞囬悽銊ㄧ珶閻ｅ瞼鎮嗘担鎾圭箻鐞涘苯鍨版担鎾广冪粈铏规畱濞撳憡鍨欑电呰杽閵
 * @details 鏉╂瑥鎷扮粩瀣鏌熸担鎾冲毐閺佹澘鐨㈢拋鈺缍橀弸鍕缂撳板牆姘ｆ肩暆閸楁洍娆戞畱閻椻晝鎮婃稉鏍鏅閵嗗倷缍橀崣閼冲熺箷闂囩憰浣稿綗娑撴稉閸戣姤鏆熼弶銉ュ灡瀵ょ瘺BB缁斿鏌熸担鎾
 * @param position 閻炲啩缍嬮惃鍕缍呯純閵
 * @param radius 閻炲啩缍嬮惃鍕宕愬板嫨
 * @param inverseMass 閻炲啩缍嬮惃鍕鍡氬窛闁插繈
 * @return 閸掓稑缂撻惃鍕鎮嗘担鎾寸埗閹村繐纭呰杽閵
 */
GameObject* TutorialGame::BuildSphereObject(GameObject* obj, const Vector3& position, float radius, float inverseMass,
	Rendering::Mesh* mesh,
	const GameTechMaterial* material) {
	if (!obj) {
		obj = new GameObject();
	}
	Vector3 sphereSize = Vector3(radius, radius, radius);
	SphereVolume* volume = new SphereVolume(radius);
	obj->SetBoundingVolume(volume);
	obj->GetTransform().SetScale(sphereSize).SetPosition(position);
	if (!mesh) {
		mesh = sphereMesh;
	}
	const GameTechMaterial& usedMaterial =
		material ? *material : checkerMaterial;
	obj->SetRenderObject(new RenderObject(obj->GetTransform(), mesh, usedMaterial));
	obj->SetPhysicsObject(new PhysicsObject(obj->GetTransform(), obj->GetBoundingVolume()));
	obj->GetPhysicsObject()->SetInverseMass(inverseMass);
	obj->GetPhysicsObject()->InitSphereInertia();
	world.AddGameObject(obj);
	return obj;
}

GameObject* TutorialGame::AddSphereToWorld(const Vector3& position,
	float radius,
	float inverseMass,
	Rendering::Mesh* mesh,
	const GameTechMaterial* material,
	const std::string& name) {
	return BuildSphereObject(new GameObject(name), position, radius, inverseMass, mesh, material);
}

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
		material ? *material : checkerMaterial;  // class 閹存劕鎲抽崣姗鍣
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
					return; //
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
	if (enemyObject) {
		return;
	}
	enemyObject = AddEnemyToWorld(pos);
	if (enemyAI) {
		delete enemyAI;
		enemyAI = nullptr;
	}
	if (navMesh) {
		EnemyAI::Params params;
		params.moveSpeed = 8.0f;
		params.waypointTolerance = 0.5f;
		params.chaseDistance = 80.0f;
		params.loseDistance = 120.0f;
		params.repathPlayerDelta = 3.0f;
		params.stuckTime = 1.25f;
		params.stuckMoveEpsilon = 0.05f;
		params.recoverDuration = 0.6f;
		params.pathRefreshTime = 1.0f;
		params.catchDistance = 3.0f;
		// Define floor bounds (a little above ground to include player feet)
		params.floorMin = floorCenter - floorHalfSize - Vector3(0, -1.0f, 0);
		params.floorMax = floorCenter + floorHalfSize + Vector3(0, 2.0f, 0);
		enemyAI = new EnemyAI(*navMesh, params);
		enemyAI->SetOwner(enemyObject);
		enemyAI->SetTarget(playerObject);
		enemyAI->SetOnCatch([this]() { OnPlayerCaught(); });
	}
}

void TutorialGame::UpdateEnemyAI(float dt) {
	if (!enemyAI || !navMesh || !enemyObject || !playerObject) {
		return;
	}
	enemyAI->SetTarget(playerObject);
	enemyAI->Update(dt);
}
GameObject* TutorialGame::AddFloorToWorld(const Vector3& position) {
	Vector3 floorSize = Vector3(30, 0.5f, 30);
	floorCenter = position;
	floorHalfSize = floorSize;
	return AddCubeToWorld(position, floorSize, 0.0f);
}

GameObject* TutorialGame::AddPlayerToWorld(const Vector3& position) {
	PlayerObject* player = new PlayerObject(*this, "Player");
	return BuildSphereObject(player, position, 1.0f, 0.5f, catMesh, &notexMaterial);
}

GameObject* TutorialGame::AddEnemyToWorld(const Vector3& position) {
	float meshSize = 2.0f;
	Vector3 halfDims = Vector3(1, 1, 1) * meshSize;
	return AddCubeToWorld(position, halfDims, 0.5f, enemyMesh, &notexMaterial);
}

GameObject* TutorialGame::AddCoinToWorld(const Vector3& position) {
	float radius = coinRadius;
	Vector3 spawn = position;
	// Ensure it rests on default floor (half-height 1.0) instead of embedding
	float floorTop = 1.2f;
	if (spawn.y < floorTop + radius) {
		spawn.y = floorTop + radius + 0.05f;
	}
	GameObject* coin = new GameObject("Coin");
	Vector3 sphereSize = Vector3(radius, radius, radius);
	SphereVolume* vol = new SphereVolume(radius);
	coin->SetBoundingVolume(vol);
	coin->GetTransform().SetScale(sphereSize).SetPosition(spawn);
	coin->SetRenderObject(new RenderObject(coin->GetTransform(), coinMesh ? coinMesh : sphereMesh, notexMaterial));
	if (coin->GetRenderObject()) {
		coin->GetRenderObject()->SetColour(Vector4(1.0f, 0.9f, 0.2f, 1.0f)); // bright yellow for visibility
	}
	// No PhysicsObject to avoid physics bounce; pickup via distance check
	world.AddGameObject(coin);
	coins.push_back(coin);
	return coin;
}

void TutorialGame::BuildSlopeScene() {
	Vector3 floorPos = Vector3(0, 0, 50);

	AddFloorToWorld(floorPos); //地板



	// 随机生成 10 个金币，位置在地板范围内，高度 floorCenter.y + 1.0f

	std::mt19937 rng((uint32_t)std::chrono::system_clock::now().time_since_epoch().count());

	Vector3 margin = floorHalfSize * 0.2f;
	std::uniform_real_distribution<float> distX(floorCenter.x - floorHalfSize.x + margin.x, floorCenter.x + floorHalfSize.x - margin.x);
	std::uniform_real_distribution<float> distZ(floorCenter.z - floorHalfSize.z + margin.z, floorCenter.z + floorHalfSize.z - margin.z);
	for (int i = 0; i < 10; ++i) {
		Vector3 coinPos(distX(rng), floorCenter.y + 1.0f, distZ(rng));
		AddCoinToWorld(coinPos);
	}


	// High platform for spawn
	Vector3 platformHalfSize = Vector3(12.0f, 2.0f, 12.0f);
	Vector3 platformPos = Vector3(0.0f, 12.0f, -30.0f);
	AddCubeToWorld(platformPos, platformHalfSize, 0.0f);

	// Slope connecting platform to ground
	Vector3 slopeHalfSize = Vector3(8.0f, 1.0f, 15.0f);
	Vector3 slopePos = Vector3(0.0f, 7.0f, -5.0f);
	Quaternion slopeRot = Quaternion::AxisAngleToQuaterion(Vector3(1, 0, 0), 25.0f);
	AddOBBCubeToWorld(slopePos, slopeHalfSize, slopeRot, 0.0f);

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
	playerSpawnPos = platformPos + Vector3(-10.0f, spawnOffsetY, 0.0f);
	playerObject = AddPlayerToWorld(playerSpawnPos);

	// Enemy spawn on floor area
	Vector3 enemySpawn = floorCenter;
	enemySpawn.y = floorCenter.y + floorHalfSize.y + 2.5f;
	enemySpawn.x += -floorHalfSize.x * 0.6f;
	InitEnemyAgent(enemySpawn);

	// Patrol points on floor corners
	if (enemyAI) {
		std::vector<Vector3> patrol = {
			floorCenter + Vector3( floorHalfSize.x * 0.8f, 0.0f,  floorHalfSize.z * 0.8f),
			floorCenter + Vector3(-floorHalfSize.x * 0.8f, 0.0f,  floorHalfSize.z * 0.8f),
			floorCenter + Vector3(-floorHalfSize.x * 0.8f, 0.0f, -floorHalfSize.z * 0.8f),
			floorCenter + Vector3( floorHalfSize.x * 0.8f, 0.0f, -floorHalfSize.z * 0.8f),
		};
		enemyAI->SetPatrolPoints(patrol);
	}
}

void TutorialGame::OnPlayerCollectCoin(GameObject* coin) {
	if (!coin) return;
	playerScore++;
	// defer removal: first take it out of the world so it stops updating/colliding
	world.RemoveGameObject(coin, false);
	if (coin->GetRenderObject()) {
		coin->GetRenderObject()->SetColour(Vector4(1, 1, 0, 0.4f));
	}
	bool alreadyQueued = false;
	for (auto& p : pendingRemoval) {
		if (p.obj == coin) {
			alreadyQueued = true;
			break;
		}
	}
	if (!alreadyQueued) {
		// wait a few physics frames to let existing collisions age out
		pendingRemoval.push_back({ coin, 8 });
	}
	coins.erase(std::remove(coins.begin(), coins.end(), coin), coins.end());
}

void TutorialGame::OnPlayerCaught() {
	playerScore = 0;
	if (playerObject) {
		playerObject->GetTransform().SetPosition(playerSpawnPos);
		if (auto* phys = playerObject->GetPhysicsObject()) {
			phys->SetLinearVelocity(Vector3());
			phys->SetAngularVelocity(Vector3());
		}
	}
	// reset enemy pathing after catch
	if (enemyAI) {
		enemyAI->Reset();
	}
}

void TutorialGame::UpdatePendingRemovals(float dt) {
	(void)dt;
	for (auto it = pendingRemoval.begin(); it != pendingRemoval.end(); ) {
		it->framesLeft -= 1;
		if (it->framesLeft <= 0) {
			delete it->obj;
			it = pendingRemoval.erase(it);
		}
		else {
			++it;
		}
	}
}

void TutorialGame::UpdateCoinPickups() {
	if (!playerObject) {
		return;
	}
	Vector3 playerPos = playerObject->GetTransform().GetPosition();
	std::vector<GameObject*> toCollect;
	for (GameObject* coin : coins) {
		if (!coin) continue;
		Vector3 diff = playerPos - coin->GetTransform().GetPosition();
		float dist = Vector::Length(diff);
		if (dist <= (playerRadius + coinRadius)) {
			toCollect.push_back(coin);
		}
	}
	for (GameObject* coin : toCollect) {
		OnPlayerCollectCoin(coin);
	}
}
