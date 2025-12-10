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
#include "Assets.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <random>
#include <chrono>
#include <cstdlib>

using namespace NCL;
using namespace CSC8503;
using namespace NCL::Maths;

/**
 * @brief 闂佸搫鐎氫即鎮╂潏鈺佺摠缁嬪愭煛閸屾稑鍡椻枔閹达箑鏋佹繛鍡樺灍閺屻倖绻涢幘铏鍟為柛銊︾懇婵
 * @param inWorld 濠电偞鎸搁幉锟犲垂濞嗘劗鈻旈柡宥夊疾闂
 * @param inRenderer 濠电偞鎸搁幉锟犲垂濞嗗繈鎺楀矗婢跺苯甯闂侀潻绲婚崝
 * @param inPhysics 闂佺粯銇涢弲婵嬪箖婵犲嫬瀵查柤濮愬楅崺鐘绘煏
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
		navMesh = new NavigationMesh("smalltest.navmesh");
	}

	// start in borderless fullscreen (no resolution change)
	if (auto* win = Window::GetWindow()) {
		isFullscreen = true;
		win->SetFullScreen(true);
	}

	InitCamera();
	//InitWorld();
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
 * @brief 闂佽桨鐒﹂悷褔鍩㈤悡搴涙帡宕ㄦ繝鍌滀簽闂佹眹鍔岀氶柣搴濈矙瀵鎼佸礋闁搞倝浜跺闁
 */
TutorialGame::~TutorialGame()	{
	delete enemyAI;
	enemyAI = nullptr;
	delete mazeEnemyAI;
	mazeEnemyAI = nullptr;
	delete navMesh;
	navMesh = nullptr;
}

bool TutorialGame::HandleStartMenu(float dt) {
	(void)dt;
	if (!inStartMenu) {
		return false;
	}
	const Keyboard* kb = Window::GetKeyboard();
	const char* items[] = {
		"[1] Single Player",
		"[2] Multi Player",
		"[Q] Quit Game"
	};
	const int itemCount = 3;

	if (kb->KeyPressed(KeyCodes::W) || kb->KeyPressed(KeyCodes::UP)) {
		menuSelection = (menuSelection + itemCount - 1) % itemCount;
	}
	if (kb->KeyPressed(KeyCodes::S) || kb->KeyPressed(KeyCodes::DOWN)) {
		menuSelection = (menuSelection + 1) % itemCount;
	}
	if (kb->KeyPressed(KeyCodes::RETURN) || kb->KeyPressed(KeyCodes::SPACE)) {
		if (menuSelection == 0) {
			currentMode = GameMode::Single;
			inStartMenu = false;
			InitWorld();
		}
		else if (menuSelection == 1) {
			currentMode = GameMode::Multi; // placeholder, uses same world for now
			inStartMenu = false;
			InitWorld();
		}
		else { // Quit
			std::exit(0);
		}
	}
	// Toggle fullscreen with F11 (common shortcut)
	if (kb->KeyPressed(KeyCodes::F11)) {
		if (auto* win = Window::GetWindow()) {
			isFullscreen = !isFullscreen;
			win->SetFullScreen(isFullscreen);
		}
	}

	// Draw menu overlay
	Debug::Print("CSC8503 Coursework", Vector2(30, 80), Debug::CYAN);
	
	Debug::Print("Select Game Mode:", Vector2(35, 65), Debug::WHITE);

	float startY = 55.0f;
	float step = 5.0f;
	for (int i = 0; i < itemCount; ++i) {
		Vector4 col = (i == menuSelection) ? Debug::YELLOW : Debug::WHITE;
		std::string line = std::string(i == menuSelection ? "> " : "  ") + items[i];
		Debug::Print(line, Vector2(35, startY - step * i), col);
	}

	Debug::Print("Controls:", Vector2(5, 20), Debug::CYAN);
	Debug::Print("Move: WASD | Jump: SPACE | Grab: RMB", Vector2(5, 15), Debug::WHITE);
	Debug::Print("Menu: UP/DOWN to navigate, ENTER to select", Vector2(5, 10), Debug::WHITE);

	return true; // block game update until a choice is made
}

/**
 * @brief 濠甸敐鍛绠伴柟鑸靛哺瀵瀵告稒蓱閻撳┑鐐存尭閹诧繝宕瑰▎鎾寸劵闁哄嫬绻掔敮鍡涙煏
 * @param dt 闁汇垼搴＄偛鍊垮婂氬级閹寸姴浠遍柛锝嗘そ婵
 */
void TutorialGame::UpdateGame(float dt) {
	float fps = dt > 1e-6f ? 1.0f / dt : 0.0f;

	if (HandleStartMenu(dt)) {
		return;
	}

	if (Window::GetKeyboard()->KeyPressed(KeyCodes::F1)) {
		InitWorld(); //We can reset the simulation at any time with F1
		selectionObject = nullptr;
	}

	if (Window::GetKeyboard()->KeyPressed(KeyCodes::F2)) {
		InitCamera(); //F2 will reset the camera to a specific default place
	}
	// 闂傚倸鎳庣换鎴濃攦闂佸憡鐗楅悧婊呰勫▕瀵鍫曟偄缁楀搫瀛╅幆 - 闂佸憡鍨电换鎰版儍闁荤姳绶ょ槐鏇㈡偩婵犳艾纾婚煫鍥ㄦ尰閳
	if (Window::GetKeyboard()->KeyPressed(KeyCodes::F9)) {
		world.ShuffleConstraints(true);
	}
	if (Window::GetKeyboard()->KeyPressed(KeyCodes::F10)) {
		world.ShuffleConstraints(false);
	}
	// 闂傚倸鎳庣换鎴濃攦闂佸憡鐗楅悧鏇犳兜閸涚増婢栨俊鐐鍊曢幖缂
	if (Window::GetKeyboard()->KeyPressed(KeyCodes::F7)) {
		world.ShuffleObjects(true);
	}
	if (Window::GetKeyboard()->KeyPressed(KeyCodes::F8)) {
		world.ShuffleObjects(false);
	}

	if (lockedObject) {
		LockedObjectMovement(); // 婵炶揪缍濞夋洟寮閵堝洨涓嶆繝銏ｅ煐閹告娊寮闂佺呯焽閸愩劎鍘闂備礁銇樼粈渚鎮鹃崜褏鏁搁崨鐗堟緰
	}
	else {
		DebugObjectMovement();  // 闁荤姴閸熸娊鎯佸┑锟犲灳瀹曞洨闂佹眹鍔岀氱痪閸涚増婢栭梺鐟扮仢缁夊磭绱
	}
	//This year we can draw debug textures as well!
	//Debug::DrawTex(*defaultTex, Vector2(10, 10), Vector2(5, 5), Debug::WHITE);
	//Debug::DrawLine(Vector3(), Vector3(0, 100, 0), Vector4(1, 0, 0, 1));
	Debug::Print("FPS: " + std::to_string(fps), Vector2(5, 80), Debug::YELLOW);
	if (playerObject) {
		Vector3 pp = playerObject->GetTransform().GetPosition();
		Debug::Print("Player: " + std::to_string(pp.x) + "," + std::to_string(pp.y) + "," + std::to_string(pp.z), Vector2(5, 75), Debug::WHITE);
	}
	Debug::Print("Maze: " + mazeStatus, Vector2(5, 70), Debug::WHITE);
	Debug::Print("Score: " + std::to_string(playerScore), Vector2(80, 95), Debug::WHITE);
	HandleGrab();
	HandlePlayerMovement(dt); // 婵犮垼娉涚氶柟婵犲洦鍋濋柍杞扮劍閸庤偐娆㈤梺绋跨箞閸庡厖绨洪梺	
	UpdateGateAndPlate(dt);   // 闂佸搫娲ら悺銊╁蓟婵犲洤鍌ㄩ悗锝嗘叏韫囨稑绾у㈣泛鎽滈悷銏犮垹鐖㈤崶閿
	UpdateEnemyAI(dt);        // 闂佸搫娲ら悺銊╁蓟婵犲洤鏋侀悘鐐插悑閻绱窱
	UpdateCoinPickups();      // 闂佸綊娼у┑閳轰絼娑欑箾閺夋垿宕抽崜褎鏆滃ù锝囨嚀閻熷酣鏌涘▎鎰澹勭紒杈ㄧ箞閺屽棝宕归柛妯稿濆畷锝呫儱鎳撻崜
	// 闂佸憡鐟︽繛濠囧极婵炲稿Т濞撮庢暜閾忓湱鍛鐓愰柨婵嗘噹閻樿叉橀柣鐔告緲缁叉椽鎮樿箛鎾抽柣銊уХ閻ラ柛鏍电磿缁澶娢旈崒娴㈣偐鍨甯″畷銊╁箣閹烘洖骞闂佹眹鍔岀氬⒀勫ч幏
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
 * @brief 闂佸憡甯楃换鍌滐絿鍨鐓￠獮妤呭礋闁稿孩宀稿鍨銈﹂崹婢т粙鎮块崱娑樜
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
 * @brief 闂佸憡甯楃换鍌滐絿鍨褰冮妴鎺楀川婵犲倻浜炴繛鎴炴尰閻楁粓寮查梺鎸庣☉閺堢紒鏂挎碍鈷旈柕鍫濇欓敍瀣鎮楅悽闈涙嚀閺夎姤鐘靛仩濞夋盯宕归崡鐐鍊楅崕銈夊蓟婵犲洤鎹堕柣鎴炆戦悵闂
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
	mazeMin = Vector3();
	mazeMax = Vector3();
	mazeCenter = Vector3();
	mazeHalfSize = Vector3();
	mazeStatus = "not loaded";
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
	if (mazeEnemyAI) {
		delete mazeEnemyAI;
		mazeEnemyAI = nullptr;
	}
	mazeEnemyObject = nullptr;
	physics.UseGravity(true);
	BuildSlopeScene();
}
/**
 * @brief 闂佸搫鐎氱紓鍌涙尰缁嬪娾槈閹惧瓨濯撮悹鎭掑妽閺嗗繘鏌ｉ悙鎻掓殨缂傚秴缁辨棃骞嬮悙闈涙劙寮堕埡鍌滄牜浠﹂懞銉у牏鎲搁懜閸ㄥ搫螣婢跺瞼鐭嗛柣姘鍝ュ嬬姷鍋炲﹢瑙勭箾閸ラ幃浠嬪Ω閵娧呭骄闂佷紮绲介惉濂稿箖閸℃ɑ濯撮柟鎯ф贡缁犲氭偠濞戞垮伴柛銊у楅幏鍛村箻楠炲灝鍟扮划鍫ユ惞鐟欏嫮鏆犲┑鐐存尭閹诧繝宕瑰▎鎴犳暩閸涚増婢栭梺
 * @details 闁哄嗘櫆閻熴儵骞忛幍缁鈺冿綁寮婚悢鍛婂撮柟鎯у暱濮ｆ劙鏌℃担瑙勭凡闁汇劊鍨婚幏瀣鍩＄紓宥嗭耿瀵鎼佸礋缂傚倹鎸搁弶璺ㄥ曟慨姗堢稻閼测晜娈曢梺鍛婁焦纾垫繛鍡樺灦閻ｉ亶鏌ｅ炵粯娅囬柟婵犲啰鈻旈柡宥夊疾闂侀潧妫楅崐椋庣礊濮楀畷锝夋煠閸愯尙鍠撶粻鐑芥⒒閸モ晜鍟哄ù锝囪法绋愭繛鎴炴尰缁嬪愭煕閹达絽袚闁哄棛鍠栧鍫曞Ψ閵夈儳浠閻庣偣鍊楅惁绡塀缂備焦鏌ㄩ柡宀鍠愰幏鍛村箻
 * @param position 闂佽崵鍋涢崯鈺冪礊鐎ｉ幆鍐宕熺紓宥呮噽缁辨棃鏌
 * @param radius 闂佽崵鍋涢崯鈺冪礊鐎ｉ幆鍐宕熺规洘鍔曢弶鍨
 * @param inverseMass 闂佽崵鍋涢崯鈺冪礊鐎ｉ幆鍐宕熼柛鈩冭壘缁愭盯姊洪幓鎺旂疄
 * @return 闂佸憡甯楃粙鎴犵磽閹剧粯鍎嶉柛鏇㈠箖閸℃ɑ濯撮柟鎯х跨厬闂佽勬綑缁绘劗娑甸崨鐗堟緰闂
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
		material ? *material : checkerMaterial;  // class 闂佺懓鐡ㄩ崝鏇㈠箟閹惰棄鐭楁慨妤呭闯
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

void TutorialGame::InitMazeEnemyAgent(const Vector3& pos) {
	if (mazeEnemyObject) {
		return;
	}
	mazeEnemyObject = AddEnemyToWorld(pos);
	if (mazeEnemyAI) {
		delete mazeEnemyAI;
		mazeEnemyAI = nullptr;
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
		params.floorMin = mazeMin - Vector3(0, -1.0f, 0);
		params.floorMax = mazeMax + Vector3(0, 2.0f, 0);
		mazeEnemyAI = new EnemyAI(*navMesh, params);
		mazeEnemyAI->SetOwner(mazeEnemyObject);
		mazeEnemyAI->SetTarget(playerObject);
		mazeEnemyAI->SetOnCatch([this]() { OnPlayerCaught(); });
	}
}

void TutorialGame::UpdateEnemyAI(float dt) {
	if (enemyAI && enemyObject && playerObject) {
		enemyAI->SetTarget(playerObject);
		enemyAI->Update(dt);
	}
	if (mazeEnemyAI && mazeEnemyObject && playerObject) {
		mazeEnemyAI->SetTarget(playerObject);
		mazeEnemyAI->Update(dt);
	}
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

void TutorialGame::LoadMazeFromTxt(const std::string& path, const Vector3& offset) {
	mazeStatus = "loading";
	// Resolve relative path into the engine Asset data directory
	std::string resolvedPath = path;
	if (resolvedPath.find(":") == std::string::npos) {
		const std::string dataPrefix = "Assets/Data/";
		std::string relative = resolvedPath;
		if (relative.find(dataPrefix) == 0) {
			relative = relative.substr(dataPrefix.size());
		}
		resolvedPath = Assets::DATADIR + relative;
	}
	std::ifstream file(resolvedPath);
	if (!file.is_open()) {
		mazeStatus = "load fail";
		Debug::Print("Maze load failed: " + resolvedPath, Vector2(5, 60), Debug::RED);
		return;
	}
	struct Cell { Vector3 pos; Vector3 size; };
	std::vector<Cell> cells;
	Vector3 minP(1e9f, 1e9f, 1e9f);
	Vector3 maxP(-1e9f, -1e9f, -1e9f);
	std::string line;
	while (std::getline(file, line)) {
		if (line.empty()) {
			continue;
		}
		std::istringstream iss(line);
		float px, py, pz, sx, sy, sz;
		if (!(iss >> px >> py >> pz >> sx >> sy >> sz)) {
			continue;
		}
		Vector3 pos(px, py, pz);
		Vector3 worldPos = pos + offset;
		minP.x = std::min(minP.x, worldPos.x); minP.y = std::min(minP.y, worldPos.y); minP.z = std::min(minP.z, worldPos.z);
		maxP.x = std::max(maxP.x, worldPos.x); maxP.y = std::max(maxP.y, worldPos.y); maxP.z = std::max(maxP.z, worldPos.z);
		cells.push_back({ pos, Vector3(sx, sy, sz) });
	}
	if (cells.empty()) {
		mazeStatus = "maze empty";
		Debug::Print("Maze empty", Vector2(5, 60), Debug::RED);
		return;
	}
	mazeMin = minP;
	mazeMax = maxP;
	mazeCenter = (mazeMin + mazeMax) * 0.5f;
	mazeHalfSize = (mazeMax - mazeMin) * 0.5f;
	for (const auto& c : cells) {
		Vector3 pos = c.pos + offset; // use positions from file (plus optional offset)
		Vector3 halfDims = c.size * 0.5f;
		GameObject* wall = AddCubeToWorld(pos, halfDims, 0.0f);
		if (wall && wall->GetRenderObject()) {
			wall->GetRenderObject()->SetColour(Vector4(0.4f, 0.4f, 0.4f, 1.0f));
		}
	}
	mazeStatus = "cells " + std::to_string(cells.size());
	Debug::Print("Maze cells: " + std::to_string(cells.size()), Vector2(5, 60), Debug::GREEN);
}


void TutorialGame::BuildSlopeScene() {
	Vector3 floorPos = Vector3(0, 0, 50);

	AddFloorToWorld(floorPos); //鍦版澘

	// 鍔犺浇杩峰锛堝亸绉诲彲鎸夐渶璋冩暣锛

	LoadMazeFromTxt("Assets/Data/maze.txt", Vector3());

	// Spawn maze enemy using same AI settings as floor enemy
	if (mazeHalfSize.x > 0.0f || mazeHalfSize.z > 0.0f) {
		Vector3 spawn = mazeCenter;
		spawn.y = mazeMax.y + 2.5f;
		InitMazeEnemyAgent(spawn);
	}

	// 闅忔満鐢熸垚 10 涓閲戝竵锛屼綅缃鍦ㄥ湴鏉胯寖鍥村唴锛岄珮搴 floorCenter.y + 1.0f

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
	if (mazeEnemyAI && (mazeHalfSize.x > 0.0f || mazeHalfSize.z > 0.0f)) {
		std::vector<Vector3> patrol = {
			mazeCenter + Vector3( mazeHalfSize.x * 0.8f, 0.0f,  mazeHalfSize.z * 0.8f),
			mazeCenter + Vector3(-mazeHalfSize.x * 0.8f, 0.0f,  mazeHalfSize.z * 0.8f),
			mazeCenter + Vector3(-mazeHalfSize.x * 0.8f, 0.0f, -mazeHalfSize.z * 0.8f),
			mazeCenter + Vector3( mazeHalfSize.x * 0.8f, 0.0f, -mazeHalfSize.z * 0.8f),
		};
		mazeEnemyAI->SetPatrolPoints(patrol);
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
	if (mazeEnemyAI) {
		mazeEnemyAI->Reset();
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
