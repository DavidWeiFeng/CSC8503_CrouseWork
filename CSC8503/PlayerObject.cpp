#include "PlayerObject.h"
#include "TutorialGame.h"

using namespace NCL;
using namespace CSC8503;

PlayerObject::PlayerObject(TutorialGame& game, const std::string& name)
	: GameObject(name), owner(game) {}

void PlayerObject::OnCollisionBegin(GameObject* otherObject) {
	if (!otherObject) {
		return;
	}
	if (otherObject->GetName() == "Coin") {
		owner.OnPlayerCollectCoin(otherObject);
	}
}
