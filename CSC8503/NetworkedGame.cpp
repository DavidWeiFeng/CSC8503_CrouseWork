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
#define PLAYER_ID_MSG 31 // Handshake message

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
		timeToNextPacket += 1.0f / 20.0f; //20hz server/client update
	}

	if (!inStartMenu && !thisServer && Window::GetKeyboard()->KeyPressed(KeyCodes::F9)) {
		StartAsServer();
	}
	if (!inStartMenu && !thisClient && Window::GetKeyboard()->KeyPressed(KeyCodes::F10)) {
		StartAsClient(127,0,0,1);
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
	}
}

void NetworkedGame::ReceivePacket(int type, GamePacket* payload, int source) {
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
			networkObjects.push_back(obj->GetNetworkObject());
			
			// If this object matches our assigned ID, it's US! Bind camera.
			if (localNetworkID == realPacket->objectID) {
				playerObject = obj;
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

void NetworkedGame::UpdateAsServer(float dt) {
	// Apply client inputs
	for (auto const& [playerID, inputPacket] : latestClientInput) {
		GameObject* player = nullptr;
		if (serverPlayers.find(playerID) != serverPlayers.end()) {
			player = serverPlayers[playerID];
		}
		else {
			// If player not found, spawn them (initial connection)
			std::cout << "Server: Spawning player for client " << playerID << std::endl;
			player = AddPlayerToWorld(Vector3(0, 5, 0));
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
		}
	}
	latestClientInput.clear(); // Clear inputs after processing

	packetsToSnapshot--;
	if (packetsToSnapshot < 0) {
		BroadcastSnapshot(false);
		packetsToSnapshot = 5;
	}
	else {
		BroadcastSnapshot(true);
	}
}
