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

#define COLLISION_MSG 30

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
}

NetworkedGame::~NetworkedGame()	{
	delete thisServer;
	delete thisClient;
}

void NetworkedGame::StartAsServer() {
	thisServer = new GameServer(NetworkBase::GetDefaultPort(), 4);

	thisServer->RegisterPacketHandler(Received_State, this);
	thisServer->RegisterPacketHandler(Player_Input, this);

	StartLevel();
}

void NetworkedGame::StartAsClient(char a, char b, char c, char d) {
	thisClient = new GameClient();
	thisClient->Connect(a, b, c, d, NetworkBase::GetDefaultPort());

	thisClient->RegisterPacketHandler(Delta_State, this);
	thisClient->RegisterPacketHandler(Full_State, this);
	thisClient->RegisterPacketHandler(Player_Connected, this);
	thisClient->RegisterPacketHandler(Player_Disconnected, this);

	StartLevel();
}

void NetworkedGame::UpdateGame(float dt) {
	timeToNextPacket -= dt;
	if (timeToNextPacket < 0) {
		if (thisServer) {
			UpdateAsServer(dt);
		}
		else if (thisClient) {
			UpdateAsClient(dt);
		}
		timeToNextPacket += 1.0f / 20.0f; //20hz server/client update
	}

	if (!thisServer && Window::GetKeyboard()->KeyPressed(KeyCodes::F9)) {
		StartAsServer();
	}
	if (!thisClient && Window::GetKeyboard()->KeyPressed(KeyCodes::F10)) {
		StartAsClient(127,0,0,1);
	}

	TutorialGame::UpdateGame(dt);
}

void NetworkedGame::UpdateAsServer(float dt) {
	packetsToSnapshot--;
	if (packetsToSnapshot < 0) {
		BroadcastSnapshot(false);
		packetsToSnapshot = 5;
	}
	else {
		BroadcastSnapshot(true);
	}
}

void NetworkedGame::UpdateAsClient(float dt) {
	PlayerInputPacket newPacket;
	newPacket.keyW = Window::GetKeyboard()->KeyDown(KeyCodes::W);
	newPacket.keyA = Window::GetKeyboard()->KeyDown(KeyCodes::A);
	newPacket.keyS = Window::GetKeyboard()->KeyDown(KeyCodes::S);
	newPacket.keyD = Window::GetKeyboard()->KeyDown(KeyCodes::D);
	newPacket.keySpace = Window::GetKeyboard()->KeyPressed(KeyCodes::SPACE);
	newPacket.keyGrab = Window::GetKeyboard()->KeyDown(KeyCodes::SHIFT); 
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
		if (o->WritePacket(&newPacket, deltaFrame, playerState)) {
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
	InitWorld();
	// Spawn a local player (for host) or observer
	// If Server:
	if (thisServer) {
		// Spawn Local Player
		localPlayer = AddPlayerToWorld(Vector3(0, 5, 0));
		localPlayer->SetNetworkObject(new NetworkObject(*localPlayer, 100)); // ID 100 for host
		networkObjects.push_back(localPlayer->GetNetworkObject());
		playerObject = localPlayer;
	}
	// If Client:
	if (thisClient) {
		// Do nothing, wait for packets to spawn objects?
		// For Phase 1, we need to create the dummy object to receive data
		// Let's manually spawn the Host Player on Client too so it has ID 100
		GameObject* remotePlayer = AddPlayerToWorld(Vector3(0, -100, 0)); // Hidden initially
		remotePlayer->SetNetworkObject(new NetworkObject(*remotePlayer, 100));
		networkObjects.push_back(remotePlayer->GetNetworkObject());
	}
}

void NetworkedGame::ReceivePacket(int type, GamePacket* payload, int source) {
	if (type == Full_State) {
		FullPacket* realPacket = (FullPacket*)payload;
		for (auto& netObj : networkObjects) {
			if (netObj->GetNetworkID() == realPacket->objectID) {
				netObj->ReadPacket(*realPacket);
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
		int playerID = source;
		
		GameObject* player = nullptr;
		if (serverPlayers.find(playerID) != serverPlayers.end()) {
			player = serverPlayers[playerID];
		}
		else {
			std::cout << "Server: Spawning player for client " << playerID << std::endl;
			player = AddPlayerToWorld(Vector3(0, 5, 0));
			// Give it a network ID? Client needs to know this ID.
			// For Phase 1, let's just use the Input to move the HOST player (ID 100) to verify!
			// Or spawn a non-networked player just to see physics.
			// Let's try to map it to a new NetworkObject.
			int newNetID = 1000 + playerID;
			player->SetNetworkObject(new NetworkObject(*player, newNetID));
			networkObjects.push_back(player->GetNetworkObject());
			serverPlayers[playerID] = player;
		}

		if (player) {
			float speed = 30.0f;
			float rotation = realPacket->cameraYaw;
			Vector3 fwd = Quaternion::AxisAngleToQuaterion(Vector3(0, 1, 0), rotation) * Vector3(0, 0, -1);
			Vector3 right = Vector::Cross(Vector3(0, 1, 0), fwd);

			if (realPacket->keyW) player->GetPhysicsObject()->AddForce(fwd * speed);
			if (realPacket->keyS) player->GetPhysicsObject()->AddForce(-fwd * speed);
			if (realPacket->keyA) player->GetPhysicsObject()->AddForce(-right * speed);
			if (realPacket->keyD) player->GetPhysicsObject()->AddForce(right * speed);
			if (realPacket->keySpace) player->GetPhysicsObject()->ApplyLinearImpulse(Vector3(0, 10, 0));
		}
	}
}

void NetworkedGame::OnPlayerCollision(NetworkPlayer* a, NetworkPlayer* b) {
	if (thisServer) { //detected a collision between players!
		MessagePacket newPacket;
		newPacket.messageID = COLLISION_MSG;
		newPacket.playerID  = a->GetPlayerNum();

		thisClient->SendPacket(newPacket);

		newPacket.playerID = b->GetPlayerNum();
		thisClient->SendPacket(newPacket);
	}
}
