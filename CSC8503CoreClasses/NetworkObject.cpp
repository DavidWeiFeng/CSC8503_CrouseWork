#include <algorithm>
#include <chrono>
#include "NetworkObject.h"
#include "./enet/enet.h"

using namespace NCL;
using namespace CSC8503;

namespace {
	double NowSeconds() {
		using namespace std::chrono;
		return duration<double>(steady_clock::now().time_since_epoch()).count();
	}
}

NetworkObject::NetworkObject(GameObject& o, int id) : object(o) {
	deltaErrors = 0;
	fullErrors  = 0;
	networkID   = id;
}

NetworkObject::~NetworkObject() {
}

bool NetworkObject::ReadPacket(GamePacket& p) {
	if (p.type == Delta_State) {
		return ReadDeltaPacket((DeltaPacket&)p);
	}
	if (p.type == Full_State) {
		return ReadFullPacket((FullPacket&)p);
	}
	return false;
}

bool NetworkObject::WritePacket(GamePacket** p, bool deltaFrame, int stateID) {
	if (deltaFrame) {
		if (WriteDeltaPacket(p, stateID)) {
			return true;
		}
	}
	return WriteFullPacket(p);
}

bool NetworkObject::ReadDeltaPacket(DeltaPacket &p) {
	if (p.fullID != lastFullState.stateID) {
		deltaErrors++;
		return false;
	}
	UpdateStateHistory(p.fullID);

	Vector3 fullPos = lastFullState.position;
	Quaternion fullOrientation = lastFullState.orientation;

	fullPos.x += p.pos[0];
	fullPos.y += p.pos[1];
	fullPos.z += p.pos[2];

	fullOrientation.x += ((float)p.orientation[0]) / 127.0f;
	fullOrientation.y += ((float)p.orientation[1]) / 127.0f;
	fullOrientation.z += ((float)p.orientation[2]) / 127.0f;
	fullOrientation.w += ((float)p.orientation[3]) / 127.0f;

	object.GetTransform().SetPosition(fullPos);
	object.GetTransform().SetOrientation(fullOrientation);

	NetworkObject::TimedState tsDelta;
	tsDelta.state.position = fullPos;
	tsDelta.state.orientation = fullOrientation;
	tsDelta.state.stateID = p.fullID;
	tsDelta.timeSeconds = NowSeconds();
	bufferedStates.push_back(tsDelta);
	if (bufferedStates.size() > 60) {
		bufferedStates.pop_front();
	}
	return true;
}

bool NetworkObject::ReadFullPacket(FullPacket &p) {
	if (p.fullState.stateID < lastFullState.stateID) {
		return false;
	}
	lastFullState = p.fullState;

	object.GetTransform().SetPosition(lastFullState.position);
	object.GetTransform().SetOrientation(lastFullState.orientation);

	NetworkObject::TimedState tsFull;
	tsFull.state = lastFullState;
	tsFull.timeSeconds = NowSeconds();
	bufferedStates.push_back(tsFull);
	if (bufferedStates.size() > 60) {
		bufferedStates.pop_front();
	}

	stateHistory.emplace_back(lastFullState);
	return true;
}

bool NetworkObject::WriteDeltaPacket(GamePacket**p, int stateID) {
	DeltaPacket* dp = new DeltaPacket();
	NetworkState state;
	if (!GetNetworkState(stateID, state)) {
		return false;
	}

	dp->fullID = stateID;
	dp->objectID = networkID;

	Vector3 currentPos = object.GetTransform().GetPosition();
	Quaternion currentOrientation = object.GetTransform().GetOrientation();

	currentPos -= state.position;
	currentOrientation -= state.orientation;

	dp->pos[0] = (char)currentPos.x;
	dp->pos[1] = (char)currentPos.y;
	dp->pos[2] = (char)currentPos.z;

	dp->orientation[0] = (char)(currentOrientation.x * 127.0f);
	dp->orientation[1] = (char)(currentOrientation.y * 127.0f);
	dp->orientation[2] = (char)(currentOrientation.z * 127.0f);
	dp->orientation[3] = (char)(currentOrientation.w * 127.0f);

	*p = dp;
	return true;
}

bool NetworkObject::WriteFullPacket(GamePacket**p) {
	FullPacket* fp = new FullPacket();

	fp->objectID = networkID;
	fp->fullState.position = object.GetTransform().GetPosition();
	fp->fullState.orientation = object.GetTransform().GetOrientation();
	fp->fullState.stateID = lastFullState.stateID++;

	stateHistory.emplace_back(fp->fullState);
	constexpr size_t maxHistory = 90;
	if (stateHistory.size() > maxHistory) {
		stateHistory.erase(stateHistory.begin(), stateHistory.begin() + (stateHistory.size() - maxHistory));
	}

	*p = fp;
	return true;
}

NetworkState& NetworkObject::GetLatestNetworkState() {
	return lastFullState;
}

bool NetworkObject::GetNetworkState(int stateID, NetworkState& state) {
	for (auto& entry : stateHistory) {
		if (entry.stateID == stateID) {
			state = entry;
			return true;
		}
	}
	return false;
}

void NetworkObject::UpdateStateHistory(int minID) {
	stateHistory.erase(
		std::remove_if(
			stateHistory.begin(), stateHistory.end(),
			[minID](const NetworkState& s) { return s.stateID < minID; }),
		stateHistory.end());
}

void NetworkObject::ClientInterpolate(double nowSeconds, double interpBackSeconds) {
	if (bufferedStates.empty()) {
		return;
	}

	double targetTime = nowSeconds - interpBackSeconds;

	while (bufferedStates.size() >= 2 && bufferedStates[1].timeSeconds < targetTime - 1.0) {
		bufferedStates.pop_front();
	}

	NetworkObject::TimedState a = bufferedStates.front();
	NetworkObject::TimedState b = bufferedStates.back();
	for (size_t i = 1; i < bufferedStates.size(); ++i) {
		if (bufferedStates[i].timeSeconds >= targetTime) {
			a = bufferedStates[i - 1];
			b = bufferedStates[i];
			break;
		}
	}

	double span = b.timeSeconds - a.timeSeconds;
	float t = 0.0f;
	if (span > 1e-5) {
		t = float((targetTime - a.timeSeconds) / span);
		t = std::clamp(t, 0.0f, 1.0f);
	}

	Vector3 pos = a.state.position * (1.0f - t) + b.state.position * t;
	Quaternion qa = a.state.orientation;
	Quaternion qb = b.state.orientation;
	Quaternion q = Quaternion::Slerp(qa, qb, t);

	object.GetTransform().SetPosition(pos);
	object.GetTransform().SetOrientation(q);
}
