#pragma once
#include "TutorialGame.h"
#include "NetworkBase.h"

namespace NCL::CSC8503 {
	class GameServer;
	class GameClient;
	class NetworkPlayer;
	class NetworkObject;

	struct PlayerInputPacket : public GamePacket {
		int		playerID;
		bool	keyW;
		bool	keyA;
		bool	keyS;
		bool	keyD;
		bool	keySpace;
		bool	keyGrab;
		float	cameraYaw;

		PlayerInputPacket() {
			type = BasicNetworkMessages::Player_Input;
			size = sizeof(PlayerInputPacket) - sizeof(GamePacket);
		}
	};

	struct HighScoreEntryNet {
		char name[32];
		int  score;
	};

	struct HighScorePacket : public GamePacket {
		int count = 0;
		HighScoreEntryNet entries[8];
		HighScorePacket() {
			type = BasicNetworkMessages::HighScore_Data;
			size = sizeof(HighScorePacket) - sizeof(GamePacket);
		}
	};

	class NetworkedGame : public TutorialGame, public PacketReceiver 
	{
	public:
		NetworkedGame(GameWorld& gameWorld, GameTechRendererInterface& renderer, PhysicsSystem& physics);
		~NetworkedGame();

		void StartAsServer();
		void StartAsClient(char a, char b, char c, char d);

		void UpdateGame(float dt) override;

		void SpawnPlayer();

		void StartLevel();

		void ReceivePacket(int type, GamePacket* payload, int source) override;

		void OnPlayerCollision(NetworkPlayer* a, NetworkPlayer* b);

	protected:
		void UpdateAsServer(float dt);
		void UpdateAsClient(float dt);

		void BroadcastSnapshot(bool deltaFrame);
		void UpdateMinimumState();
		std::map<int, int> stateIDs;

		GameServer* thisServer;
		GameClient* thisClient;
		float timeToNextPacket;
		int packetsToSnapshot;

		std::vector<NetworkObject*> networkObjects;

		std::map<int, GameObject*> serverPlayers;
		std::map<int, PlayerInputPacket> latestClientInput; // Added
		GameObject* localPlayer;
		int localNetworkID = -1;

		struct ScoreEntry {
			std::string name;
			int score = 0;
		};
		std::vector<ScoreEntry> serverHighScores;
		std::vector<ScoreEntry> clientHighScores;
		double lastScoreRequestTime = 0.0;
		int cachedServerScore = 0;

		void UpdateHighScore(const std::string& name, int score);
		void LoadHighScores();
		void SaveHighScores();
	};
}
