#pragma once
#include "GameObject.h"

namespace NCL::CSC8503 {
	class TutorialGame;

	class PlayerObject : public GameObject {
	public:
		PlayerObject(TutorialGame& game, const std::string& name);
		void OnCollisionBegin(GameObject* otherObject) override;
	private:
		TutorialGame& owner;
	};
}
