#include "NetworkedGame.h"
#include "NetworkPlayer.h"
#include "NetworkObject.h"
#include "GameServer.h"
#include "GameClient.h"
#include "GameWorld.h"
#include "PhysicsObject.h"
#include "Window.h"
#include "Matrix.h"
#include "Quaternion.h"
#include <algorithm>
#include <chrono>
#include <fstream>
#include "Debug.h"

#define COLLISION_MSG 30
#define PLAYER_ID_MSG 31 // Handshake message
#define HEAVY_GRAB_MSG 32

using namespace NCL;
using namespace CSC8503;

struct MessagePacket : public GamePacket {
	short playerID;
	short messageID;

	MessagePacket() {
		type = Message;
		size = sizeof(short) * 2;
	}
};

NetworkedGame::NetworkedGame(GameWorld& gameWorld, GameTechRendererInterface& renderer, PhysicsSystem& physics) : TutorialGame(gameWorld, renderer, physics)
{
	thisServer = nullptr;
	thisClient = nullptr;

	NetworkBase::Initialise();
	timeToNextPacket  = 0.0f;
	packetsToSnapshot = 0;
	heavyCoop = {};
}

NetworkedGame::~NetworkedGame()	{
	if (thisServer) {
		SaveHighScores();
	}
	if (heavyCoop.aToHeavy) {
		world.RemoveConstraint(heavyCoop.aToHeavy, true);
	}
	if (heavyCoop.bToHeavy) {
		world.RemoveConstraint(heavyCoop.bToHeavy, true);
	}
	if (heavyCoop.aToB) {
		world.RemoveConstraint(heavyCoop.aToB, true);
	}
	delete thisServer;
	delete thisClient;
}

void NetworkedGame::StartAsServer() {
	thisServer = new GameServer(NetworkBase::GetDefaultPort(), 4);

	thisServer->RegisterPacketHandler(Received_State, this);
	thisServer->RegisterPacketHandler(Player_Input, this);
	thisServer->RegisterPacketHandler(HighScore_Request, this);

	LoadHighScores();
	StartLevel();
}

void NetworkedGame::StartAsClient(char a, char b, char c, char d) {
	thisClient = new GameClient();
	thisClient->Connect(a, b, c, d, NetworkBase::GetDefaultPort());

	thisClient->RegisterPacketHandler(Delta_State, this);
	thisClient->RegisterPacketHandler(Full_State, this);
	thisClient->RegisterPacketHandler(Player_Connected, this);
	thisClient->RegisterPacketHandler(Player_Disconnected, this);
	thisClient->RegisterPacketHandler(Message, this); // handshake / misc messages
	thisClient->RegisterPacketHandler(HighScore_Data, this);

	StartLevel();
}

void NetworkedGame::UpdateGame(float dt) {
	if (thisServer) {
		thisServer->UpdateServer();
	}
	if (thisClient) {
		thisClient->UpdateClient();
	}

	timeToNextPacket -= dt;
	if (timeToNextPacket < 0) {
		if (thisServer) {
			UpdateAsServer(dt);
		}
		else if (thisClient) {
			UpdateAsClient(dt);
		}
		timeToNextPacket += 1.0f / 60.0f; // 30hz server/client update，降低输入-回传间隔
	}

	if (!inStartMenu && !thisServer && Window::GetKeyboard()->KeyPressed(KeyCodes::F9)) {
		StartAsServer();
	}
	if (!inStartMenu && !thisClient && Window::GetKeyboard()->KeyPressed(KeyCodes::F10)) {
		StartAsClient(127,0,0,1);
	}

	// 客户端插值平滑显示
	if (thisClient) {
		double now = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
		double interpBack = 0.05; // 50ms 回放窗口，提升跟手性
		for (auto* netObj : networkObjects) {
			if (netObj) {
				netObj->ClientInterpolate(now, interpBack);
			}
		}
	}

	// TAB 显示排行榜，并按需请求
	if (Window::GetKeyboard()->KeyDown(KeyCodes::TAB)) {
		double now = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
		if (thisClient) {
			if (now - lastScoreRequestTime > 0.5) {
				GamePacket req;
				req.type = HighScore_Request;
				req.size = 0;
				thisClient->SendPacket(req);
				lastScoreRequestTime = now;
			}
		}
		std::vector<ScoreEntry> toShow;
		if (thisServer) {
			toShow = serverHighScores;
		}
		else {
			toShow = clientHighScores;
		}
		float y = 40.0f;
		Vector2 pos(20.0f, y); // 居中
		Debug::Print("================= HIGH SCORES =================", pos, Debug::YELLOW);
		y += 3.0f;
		Debug::Print(" Rank     Name                Score", Vector2(pos.x, y), Debug::WHITE);
		y += 3.0f;
		Debug::Print("-----------------------------------------------", Vector2(pos.x, y), Debug::WHITE);
		y += 3.0f;
		if (toShow.empty()) {
			Debug::Print(" No Data", Vector2(pos.x, y), Debug::WHITE);
			y += 3.0f;
		}
		for (size_t i = 0; i < toShow.size() && i < 8; ++i) {
			const auto& e = toShow[i];
			char line[80];
			snprintf(line, sizeof(line), "  %-2zu     %-18s %-6d", i + 1, e.name.c_str(), e.score);
			Debug::Print(line, Vector2(pos.x, y), Debug::WHITE);
			y += 3.0f;
		}
		Debug::Print("===============================================", Vector2(pos.x, y), Debug::YELLOW);
		y += 3.0f;
		Debug::Print(" Your Score: " + std::to_string(playerScore), Vector2(pos.x, y), Debug::CYAN);
		y += 3.0f;
	}

	// Debug 输出重物抓取人数
	if (heavyObject) {
		Debug::Print("Heavy grabs: " + std::to_string(currentHeavyGrabbers), Vector2(5, 5), Debug::WHITE);
		if (thisServer && localHeavyGrabbing) {
			Debug::Print("Host grabbing heavy", Vector2(5, 8), Debug::CYAN);
		}
	}

	TutorialGame::UpdateGame(dt);
}

void NetworkedGame::UpdateAsClient(float dt) {
	PlayerInputPacket newPacket;
	newPacket.keyW = Window::GetKeyboard()->KeyDown(KeyCodes::W);
	newPacket.keyA = Window::GetKeyboard()->KeyDown(KeyCodes::A);
	newPacket.keyS = Window::GetKeyboard()->KeyDown(KeyCodes::S);
	newPacket.keyD = Window::GetKeyboard()->KeyDown(KeyCodes::D);
	newPacket.keySpace = Window::GetKeyboard()->KeyPressed(KeyCodes::SPACE);
	newPacket.keyGrab = Window::GetMouse()->ButtonDown(NCL::MouseButtons::Right) || Window::GetKeyboard()->KeyDown(KeyCodes::SHIFT); 
	newPacket.heavyGrabActive = (grabbedObject == heavyObject);
	newPacket.cameraYaw = world.GetMainCamera().GetYaw();
	
	thisClient->SendPacket(newPacket);
}

void NetworkedGame::BroadcastSnapshot(bool deltaFrame) {
	std::vector<GameObject*>::const_iterator first;
	std::vector<GameObject*>::const_iterator last;

	world.GetObjectIterators(first, last);

	for (auto i = first; i != last; ++i) {
		NetworkObject* o = (*i)->GetNetworkObject();
		if (!o) {
			continue;
		}
		int playerState = 0;
		GamePacket* newPacket = nullptr;
		if (o->WritePacket(&newPacket, false, playerState)) { // force full packets until delta ack implemented
			thisServer->SendGlobalPacket(*newPacket);
			delete newPacket;
		}
	}
}

void NetworkedGame::UpdateMinimumState() {
	//Periodically remove old data from the server
	int minID = INT_MAX;
	int maxID = 0; //we could use this to see if a player is lagging behind?

	for (auto i : stateIDs) {
		minID = std::min(minID, i.second);
		maxID = std::max(maxID, i.second);
	}
	//every client has acknowledged reaching at least state minID
	//so we can get rid of any old states!
	std::vector<GameObject*>::const_iterator first;
	std::vector<GameObject*>::const_iterator last;
	world.GetObjectIterators(first, last);

	for (auto i = first; i != last; ++i) {
		NetworkObject* o = (*i)->GetNetworkObject();
		if (!o) {
			continue;
		}
		o->UpdateStateHistory(minID); //clear out old states so they arent taking up memory...
	}
}

void NetworkedGame::SpawnPlayer() {
	// Not used directly, but maybe helper for StartLevel
}

void NetworkedGame::StartLevel() {
	serverHeavyGrabState.clear();
	currentHeavyGrabberIDs.clear();
	heavyCoop = {};
	lastSentHeavyCount = -1;

	InitWorld();
	// Spawn a local player (for host) or observer
	// If Server:
	if (thisServer) {
		// The player is already created by InitWorld() -> BuildSlopeScene().
		// We just need to get it and attach the network component.
		if (playerObject) { // Ensure playerObject was indeed created by InitWorld()
			localPlayer = playerObject; // Use the existing player
			localPlayer->SetNetworkObject(new NetworkObject(*localPlayer, 100)); // ID 100 for host
			networkObjects.push_back(localPlayer->GetNetworkObject());

			// Force reset physics to safe state
			if (localPlayer->GetPhysicsObject()) {
				localPlayer->GetPhysicsObject()->SetLinearVelocity(Vector3(0, 0, 0));
				localPlayer->GetPhysicsObject()->ClearForces();
			}
		}
		else {
			// Fallback: If playerObject somehow wasn't set, spawn a default one
			std::cout << "ERROR: playerObject was not set by InitWorld() for server! Spawning default.\n";
			localPlayer = AddPlayerToWorld(Vector3(0, 5, 0));
			localPlayer->SetNetworkObject(new NetworkObject(*localPlayer, 100));
			networkObjects.push_back(localPlayer->GetNetworkObject());
			playerObject = localPlayer;
		}
	}
	// If Client:
	if (thisClient) {
		// The client still needs the world geometry (floor, maze, etc.)
		// But its own player and other dynamic networked objects should be created from server snapshots.
		// So, clear the local player object which was created by InitWorld().
		if (playerObject) {
			// This playerObject was created by InitWorld(), but on the client,
			// our actual player will be spawned by the server. So we remove it.
			world.RemoveGameObject(playerObject, true); // Remove from world and delete
			playerObject = nullptr;
		}
		// 客户端保留重物，但需要挂上网络对象用于同步
		if (heavyObject && !heavyObject->GetNetworkObject()) {
			heavyObject->SetNetworkObject(new NetworkObject(*heavyObject, heavyObjectNetID));
			networkObjects.push_back(heavyObject->GetNetworkObject());
		}
	}
	// 服务器挂载重物网络对象
	if (thisServer && heavyObject && !heavyObject->GetNetworkObject()) {
		heavyObject->SetNetworkObject(new NetworkObject(*heavyObject, heavyObjectNetID));
		networkObjects.push_back(heavyObject->GetNetworkObject());
	}
}

void NetworkedGame::ReceivePacket(int type, GamePacket* payload, int source) {
	auto makeClientProxyKinematic = [](GameObject* obj) {
		if (!obj) return;
		if (PhysicsObject* phys = obj->GetPhysicsObject()) {
			phys->SetInverseMass(0.0f); // make it static on client; server drives movement
			phys->ClearForces();
			phys->SetLinearVelocity(Vector3());
			phys->SetAngularVelocity(Vector3());
		}
	};

	if (type == Full_State) {
		FullPacket* realPacket = (FullPacket*)payload;
		// Check if we already have this object
		bool found = false;
		for (auto& netObj : networkObjects) {
			if (netObj->GetNetworkID() == realPacket->objectID) {
				netObj->ReadPacket(*realPacket);
				found = true;
				break;
			}
		}
		// If not found, spawn a proxy for it (Client Side Only)
		if (!found && thisClient) {
			std::cout << "Client: Received new object " << realPacket->objectID << ", spawning proxy.\n";
			GameObject* obj = AddPlayerToWorld(realPacket->fullState.position);
			obj->SetNetworkObject(new NetworkObject(*obj, realPacket->objectID));
			makeClientProxyKinematic(obj);
			networkObjects.push_back(obj->GetNetworkObject());
			
			// If this object matches our assigned ID, it's US! Bind camera.
			if (localNetworkID == realPacket->objectID) {
				playerObject = obj;
				makeClientProxyKinematic(playerObject);
				std::cout << "Client: Bound camera to local player ID " << localNetworkID << std::endl;
			}
		}
	}
	else if (type == Delta_State) {
		DeltaPacket* realPacket = (DeltaPacket*)payload;
		for (auto& netObj : networkObjects) {
			if (netObj->GetNetworkID() == realPacket->objectID) {
				netObj->ReadPacket(*realPacket);
			}
		}
	}
	else if (type == Player_Input) {
		PlayerInputPacket* realPacket = (PlayerInputPacket*)payload;
		// Store the latest input from this client
		latestClientInput[source] = *realPacket;
	}
	else if (type == Message) {
		MessagePacket* realPacket = (MessagePacket*)payload;
		if (realPacket->messageID == COLLISION_MSG) {
			std::cout << "Client: Received collision message for " << realPacket->playerID << std::endl;
		}
		else if (realPacket->messageID == PLAYER_ID_MSG) {
			localNetworkID = realPacket->playerID;
			std::cout << "Client: Received assigned Network ID: " << localNetworkID << std::endl;
		}
		else if (realPacket->messageID == HEAVY_GRAB_MSG) {
			currentHeavyGrabbers = realPacket->playerID;
			// 更新客户端重物颜色
			if (heavyObject) {
				if (auto* phys = heavyObject->GetPhysicsObject()) {
					if (currentHeavyGrabbers >= 2) {
						phys->SetInverseMass(1.2f); // 变轻以便双人拖拽
						if (auto* ro = heavyObject->GetRenderObject()) {
							ro->SetColour(Vector4(0.2f, 0.8f, 0.2f, 1.0f));
						}
					}
					else {
						phys->SetInverseMass(0.0f);
						phys->SetLinearVelocity(Vector3());
						phys->SetAngularVelocity(Vector3());
						if (auto* ro = heavyObject->GetRenderObject()) {
							if (currentHeavyGrabbers == 1) {
								ro->SetColour(Vector4(0.9f, 0.7f, 0.2f, 1.0f));
							}
							else {
								ro->SetColour(Vector4(0.8f, 0.4f, 0.2f, 1.0f));
							}
						}
					}
				}
			}
		}
	}
	else if (type == HighScore_Request) {
		if (thisServer) {
			HighScorePacket pkt;
			pkt.count = std::min<int>((int)serverHighScores.size(), 8);
			for (int i = 0; i < pkt.count; ++i) {
				const auto& e = serverHighScores[i];
				strncpy_s(pkt.entries[i].name, e.name.c_str(), sizeof(pkt.entries[i].name) - 1);
				pkt.entries[i].name[sizeof(pkt.entries[i].name) - 1] = '\0';
				pkt.entries[i].score = e.score;
			}
			thisServer->SendPacketToPeer(pkt, source);
		}
	}
	else if (type == HighScore_Data) {
		if (thisClient) {
			HighScorePacket* pkt = (HighScorePacket*)payload;
			clientHighScores.clear();
			for (int i = 0; i < pkt->count && i < 8; ++i) {
				ScoreEntry e;
				e.name = pkt->entries[i].name;
				e.score = pkt->entries[i].score;
				clientHighScores.push_back(e);
			}
		}
	}
}

void NetworkedGame::OnPlayerCollision(NetworkPlayer* a, NetworkPlayer* b) {
	if (thisServer) { //detected a collision between players!
		MessagePacket newPacket;
		newPacket.messageID = COLLISION_MSG;
		newPacket.playerID  = a->GetPlayerNum();

		thisServer->SendGlobalPacket(newPacket);

		newPacket.playerID = b->GetPlayerNum();
		thisServer->SendGlobalPacket(newPacket);
	}
}

void NetworkedGame::SubmitScore(const std::string& name, int score) {
	if (!thisServer) {
		return;
	}
	UpdateHighScore(name, score);
	HighScorePacket pkt;
	pkt.count = std::min<int>((int)serverHighScores.size(), 8);
	for (int i = 0; i < pkt.count; ++i) {
		const auto& e = serverHighScores[i];
		strncpy_s(pkt.entries[i].name, e.name.c_str(), sizeof(pkt.entries[i].name) - 1);
		pkt.entries[i].name[sizeof(pkt.entries[i].name) - 1] = '\0';
		pkt.entries[i].score = e.score;
	}
	thisServer->SendGlobalPacket(pkt);
}

bool NetworkedGame::CanEnterHighScore(int score) const {
	const std::vector<ScoreEntry>& list = thisServer ? serverHighScores : clientHighScores;
	if (list.size() < 8) return true;
	int minScore = list.back().score;
	return score > minScore;
}

std::vector<NetworkedGame::ScoreEntry> NetworkedGame::GetScoresSnapshot() const {
	if (thisServer) {
		return serverHighScores;
	}
	return clientHighScores;
}

void NetworkedGame::UpdateHighScore(const std::string& name, int score) {
	bool found = false;
	for (auto& e : serverHighScores) {
		if (e.name == name) {
			e.score = std::max(e.score, score);
			found = true;
			break;
		}
	}
	if (!found) {
		serverHighScores.push_back({ name, score });
	}
	std::sort(serverHighScores.begin(), serverHighScores.end(), [](const ScoreEntry& a, const ScoreEntry& b) {
		return a.score > b.score;
	});
	if (serverHighScores.size() > 8) {
		serverHighScores.resize(8);
	}
	SaveHighScores();
}

void NetworkedGame::LoadHighScores() {
	serverHighScores.clear();
	std::ifstream f("HighScores.txt");
	if (!f.is_open()) {
		return;
	}
	std::string name;
	int score;
	while (f >> name >> score) {
		serverHighScores.push_back({ name, score });
	}
	std::sort(serverHighScores.begin(), serverHighScores.end(), [](const ScoreEntry& a, const ScoreEntry& b) {
		return a.score > b.score;
	});
	if (serverHighScores.size() > 8) {
		serverHighScores.resize(8);
	}
}

void NetworkedGame::SaveHighScores() {
	std::ofstream f("HighScores.txt", std::ios::trunc);
	if (!f.is_open()) return;
	for (const auto& e : serverHighScores) {
		f << e.name << " " << e.score << "\n";
	}
}

void NetworkedGame::UpdateAsServer(float dt) {
	int heavyGrabCount = 0;
	currentHeavyGrabberIDs.clear();

	auto clampPlayerToSpawn = [this](GameObject* player, int playerID) {
		if (!player) return;
		Vector3 pos = player->GetTransform().GetPosition();
		float floorY = floorCenter.y + floorHalfSize.y;
		if (pos.y < floorY - 10.0f) { // 掉到地板以下太远，拉回出生点
			Vector3 respawn = playerSpawnPos;
			respawn.x += 3.0f * float(playerID);
			player->GetTransform().SetPosition(respawn);
			if (auto* phys = player->GetPhysicsObject()) {
				phys->SetLinearVelocity(Vector3());
				phys->SetAngularVelocity(Vector3());
				phys->ClearForces();
			}
		}
	};

	// Apply client inputs
	for (auto const& [playerID, inputPacket] : latestClientInput) {
		GameObject* player = nullptr;
		if (serverPlayers.find(playerID) != serverPlayers.end()) {
			player = serverPlayers[playerID];
		}
		else {
			// If player not found, spawn them (initial connection)
			std::cout << "Server: Spawning player for client " << playerID << std::endl;
			Vector3 spawnPos = playerSpawnPos;
			// 防止多人重叠，按 peerID 稍微偏移
			spawnPos.x += 3.0f * float(playerID);
			player = AddPlayerToWorld(spawnPos);
			int newNetID = 1000 + playerID; // Assign a unique Network ID
			player->SetNetworkObject(new NetworkObject(*player, newNetID));
			networkObjects.push_back(player->GetNetworkObject());
			serverPlayers[playerID] = player;

			// Send handshake to tell the client its ID
			MessagePacket idPacket;
			idPacket.messageID = PLAYER_ID_MSG;
			idPacket.playerID = newNetID;
			thisServer->SendPacketToPeer(idPacket, playerID);
		}

		if (player) {
			MovePlayer(player, dt, inputPacket.keyW, inputPacket.keyA, inputPacket.keyS, inputPacket.keyD, inputPacket.keySpace, inputPacket.cameraYaw);
			clampPlayerToSpawn(player, playerID);
			if (heavyObject) {
				bool grabbed = inputPacket.heavyGrabActive;
				serverHeavyGrabState[playerID] = grabbed;
			}
		}
	}
	latestClientInput.clear(); // Clear inputs after processing

	// Host自己的抓取状态
	if (heavyObject) {
		serverHeavyGrabState[-1] = localHeavyGrabbing;
	}

	// 统计抓取人数（允许更大距离，超范围视为松手）
	if (heavyObject) {
		for (auto& kv : serverHeavyGrabState) {
			int pid = kv.first;
			bool grabbed = kv.second;
			if (!grabbed) continue;
			GameObject* g = nullptr;
			if (pid == -1) {
				g = playerObject;
			}
			else {
				auto it = serverPlayers.find(pid);
				if (it != serverPlayers.end()) {
					g = it->second;
				}
			}
			if (g) {
				float dist = Vector::Length(g->GetTransform().GetPosition() - heavyObject->GetTransform().GetPosition());
				if (dist <= heavyGrabRange * 2.0f) {
					heavyGrabCount++;
					currentHeavyGrabberIDs.insert(pid);
				} else {
					serverHeavyGrabState[pid] = false; // 过远视为松手
				}
			}
		}
	}

	// 设置重物是否可移动（需要至少两人同时抓取）
	if (heavyObject) {
		currentHeavyGrabbers = heavyGrabCount;
		// 同步/创建三体协作约束（服务器侧）
		if (heavyGrabCount >= 2) {
			std::vector<int> grabbers(currentHeavyGrabberIDs.begin(), currentHeavyGrabberIDs.end());
			std::sort(grabbers.begin(), grabbers.end());

			GameObject* ga = nullptr;
			GameObject* gb = nullptr;
			int count = 0;
			for (int grabberID : grabbers) {
				GameObject* g = nullptr;
				if (grabberID == -1) g = playerObject;
				else {
					auto it = serverPlayers.find(grabberID);
					if (it != serverPlayers.end()) g = it->second;
				}
				if (!g) continue;
				if (count == 0) ga = g;
				else if (count == 1) { gb = g; break; }
				count++;
			}
			bool needRebuild = false;
			if (!heavyCoop.aToHeavy || heavyCoop.a != ga || heavyCoop.b != gb) {
				needRebuild = true;
			}
			if (needRebuild) {
				if (heavyCoop.aToHeavy) world.RemoveConstraint(heavyCoop.aToHeavy, true);
				if (heavyCoop.bToHeavy) world.RemoveConstraint(heavyCoop.bToHeavy, true);
				if (heavyCoop.aToB)     world.RemoveConstraint(heavyCoop.aToB, true);
				heavyCoop = {};
				if (ga && gb) {
					// 以当前距离为基准（上限为抓取范围），略放松下限以留“弹性”
					float restA = std::clamp(Vector::Length(ga->GetTransform().GetPosition() - heavyObject->GetTransform().GetPosition()), 2.5f, heavyGrabRange);
					float restB = std::clamp(Vector::Length(gb->GetTransform().GetPosition() - heavyObject->GetTransform().GetPosition()), 2.5f, heavyGrabRange);
					float restAB = std::clamp(Vector::Length(ga->GetTransform().GetPosition() - gb->GetTransform().GetPosition()), 3.0f, heavyGrabRange * 2.0f);
					heavyCoop.aToHeavy = new PositionConstraint(ga, heavyObject, restA);
					heavyCoop.bToHeavy = new PositionConstraint(gb, heavyObject, restB);
					heavyCoop.aToB     = new PositionConstraint(ga, gb, restAB);
					heavyCoop.a = ga; heavyCoop.b = gb;
					world.AddConstraint(heavyCoop.aToHeavy);
					world.AddConstraint(heavyCoop.bToHeavy);
					world.AddConstraint(heavyCoop.aToB);
				}
			}
			// 清理旧的单体约束表
			for (auto& kv : heavyGrabConstraints) {
				world.RemoveConstraint(kv.second, true);
			}
			heavyGrabConstraints.clear();
		} else {
			// 不足两人抓取，移除所有约束
			for (auto& kv : heavyGrabConstraints) {
				world.RemoveConstraint(kv.second, true);
			}
			heavyGrabConstraints.clear();
			if (heavyCoop.aToHeavy) world.RemoveConstraint(heavyCoop.aToHeavy, true);
			if (heavyCoop.bToHeavy) world.RemoveConstraint(heavyCoop.bToHeavy, true);
			if (heavyCoop.aToB)     world.RemoveConstraint(heavyCoop.aToB, true);
			heavyCoop = {};
		}
		if (auto* phys = heavyObject->GetPhysicsObject()) {
			if (heavyGrabCount >= 2) {
				phys->SetInverseMass(1.2f); // 变轻以便双人拖拽
				if (auto* ro = heavyObject->GetRenderObject()) {
					ro->SetColour(Vector4(0.2f, 0.8f, 0.2f, 1.0f)); // 两人抓取绿色
				}
			} else {
				phys->SetInverseMass(0.0f);
				phys->SetLinearVelocity(Vector3());
				phys->SetAngularVelocity(Vector3());
				if (auto* ro = heavyObject->GetRenderObject()) {
					if (heavyGrabCount == 1) {
						ro->SetColour(Vector4(0.9f, 0.7f, 0.2f, 1.0f)); // 单人抓取金黄
					} else {
						ro->SetColour(Vector4(0.8f, 0.4f, 0.2f, 1.0f)); // 无人抓取橙色
					}
				}
			}
		}
	}

	// 把抓取人数广播给客户端（状态变化时）
	if (thisServer && heavyGrabCount != lastSentHeavyCount) {
		MessagePacket msg;
		msg.messageID = HEAVY_GRAB_MSG;
		msg.playerID = (short)heavyGrabCount; // reuse字段承载人数
		thisServer->SendGlobalPacket(msg);
		lastSentHeavyCount = heavyGrabCount;
	}

	// 更新主机分数到排行榜（只跟踪主机分数，演示用）
	if (playerScore > cachedServerScore) {
		cachedServerScore = playerScore;
		UpdateHighScore("Host", playerScore);
		// 推送最新榜单给所有客户端
		HighScorePacket pkt;
		pkt.count = std::min<int>((int)serverHighScores.size(), 8);
		for (int i = 0; i < pkt.count; ++i) {
			const auto& e = serverHighScores[i];
			strncpy_s(pkt.entries[i].name, e.name.c_str(), sizeof(pkt.entries[i].name) - 1);
			pkt.entries[i].name[sizeof(pkt.entries[i].name) - 1] = '\0';
			pkt.entries[i].score = e.score;
		}
		thisServer->SendGlobalPacket(pkt);
	}

	packetsToSnapshot--;
	if (packetsToSnapshot < 0) {
		BroadcastSnapshot(false);
		packetsToSnapshot = 5;
	}
	else {
		BroadcastSnapshot(false); // still send full packets for stability
	}
}
