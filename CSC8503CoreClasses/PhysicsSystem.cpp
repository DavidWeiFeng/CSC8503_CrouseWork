#include "PhysicsSystem.h"
#include "PhysicsObject.h"
#include "GameObject.h"
#include "CollisionDetection.h"
#include "Quaternion.h"

#include "Constraint.h"

#include "Debug.h"
#include "Window.h"
#include <functional>
using namespace NCL;
using namespace CSC8503;
using namespace NCL::Maths;

/**
 * @brief 构造一个新的物理系统。
 * @param g 物理系统将要操作的游戏世界。
 */
PhysicsSystem::PhysicsSystem(GameWorld& g) : gameWorld(g)	
{
	applyGravity	= false;
	useBroadPhase	= false;	
	dTOffset		= 0.0f;
	globalDamping	= 	0.995f;
	SetGravity(Vector3(0.0f, -9.8f, 0.0f));
}

/**
 * @brief 物理系统的析构函数。
 */
PhysicsSystem::~PhysicsSystem()	
{
}

/**
 * @brief 设置物理系统的重力加速度。
 * @param g 新的重力向量。
 */
void PhysicsSystem::SetGravity(const Vector3& g) 
{
	gravity = g;
}

/**
 * @brief 清除物理系统中所有当前的碰撞。
 * @details 如果“游戏”被重置，物理系统必须被“清除”，以移除任何可能仍然存在于碰撞列表中的旧碰撞。如果您的引擎扩展到允许从世界中移除对象，您需要遍历此碰撞列表以移除它们所在的任何碰撞。
 */
void PhysicsSystem::Clear() 
{
	allCollisions.clear();
}

bool useSimpleContainer = false;

int constraintIterationCount = 10;

//This is the fixed timestep we'd LIKE to have
const int   idealHZ = 120;
const float idealDT = 1.0f / idealHZ;

/*
This is the fixed update we actually have...
If physics takes too long it starts to kill the framerate, it'll drop the 
iteration count down until the FPS stabilises, even if that ends up
being at a low rate. 
*/
int realHZ		= idealHZ;
float realDT	= idealDT;

/**
 * @brief 这是物理引擎更新的核心。
 * @details 它使用固定的时间步长方法来推进物理模拟。如果物理更新花费时间过长，它可能会调整时间步长以保持稳定的帧率。这是我们实际拥有的固定更新...如果物理计算花费太长时间，它会开始降低帧率，它会降低迭代次数直到FPS稳定下来，即使最终的速率很低。
 * @param dt 自上一帧以来的真实世界时间增量。
 */
void PhysicsSystem::Update(float dt) 
{	
	if (Window::GetKeyboard()->KeyPressed(KeyCodes::B)) {
		useBroadPhase = !useBroadPhase;
		std::cout << "Setting broadphase to " << useBroadPhase << std::endl;
	}
	if (Window::GetKeyboard()->KeyPressed(KeyCodes::N)) {
		useSimpleContainer = !useSimpleContainer;
		std::cout << "Setting broad container to " << useSimpleContainer << std::endl;
	}
	if (Window::GetKeyboard()->KeyPressed(KeyCodes::I)) {
		constraintIterationCount--;
		std::cout << "Setting constraint iterations to " << constraintIterationCount << std::endl;
	}
	if (Window::GetKeyboard()->KeyPressed(KeyCodes::O)) {
		constraintIterationCount++;
		std::cout << "Setting constraint iterations to " << constraintIterationCount << std::endl;
	}

	dTOffset += dt;		//We accumulate time delta here - there might be remainders from previous frame!

	GameTimer t;
	t.GetTimeDeltaSeconds();

	if (useBroadPhase) {
		UpdateObjectAABBs();
	}
	int iteratorCount = 	0;
	while(dTOffset > realDT) {
		IntegrateAccel(realDT); 	//Update accelerations from external forces
		if (useBroadPhase) {
			BroadPhase();
			NarrowPhase();
		}
		else {
			BasicCollisionDetection();
		}

		//This is our simple iterative solver - 
		//we just run things multiple times, slowly moving things forward
		//and then rechecking that the constraints have been met		
		float constraintDt = realDT /  (float)constraintIterationCount;
		for (int i = 0; i < constraintIterationCount; ++i) {
			UpdateConstraints(constraintDt); 	
		}
		IntegrateVelocity(realDT); 	//update positions from new velocity changes

		dTOffset -= realDT;
		iteratorCount++;
	}

	ClearForces();			//Once we've finished with the forces, reset them to zero

	UpdateCollisionList(); 	//Remove any old collisions

	t.Tick();
	float updateTime = t.GetTimeDeltaSeconds();

	//Uh oh, physics is taking too long...
	if (updateTime > realDT) {
		realHZ /= 2;
		realDT *= 2;
		std::cout << "Dropping iteration count due to long physics time...(now " << realHZ << ")\n";
	}
	else if(dt*2 < realDT) { 	//we have plenty of room to increase iteration count!
		int temp = realHZ;
		realHZ *= 2;
		realDT /= 2;

		if (realHZ > idealHZ) {
			realHZ = idealHZ;
			realDT = idealDT;
		}
		if (temp != realHZ) {
			std::cout << "Raising iteration count due to short physics time...(now " << realHZ << ")\n";
		}
	}
}

/**
 * @brief 管理正在进行的碰撞列表。
 * @details 之后我们需要跨多个帧跟踪碰撞，因此我们将它们存储在一个集合中。

第一次添加它们时，我们告诉对象它们正在碰撞。
在它们将被移除的帧，我们告诉它们不再碰撞。

通过这个简单的机制，我们在 OnCollisionBegin / OnCollisionEnd 函数内部构建游戏交互（例如，被火箭发射器击中时减少生命值，玩家碰到金币时获得分数等等）。
 */
void PhysicsSystem::UpdateCollisionList() 
{
	for (std::set<CollisionDetection::CollisionInfo>::iterator i = allCollisions.begin(); i != allCollisions.end(); ) {
		if ((*i).framesLeft == numCollisionFrames) {
			i->a->OnCollisionBegin(i->b);
			i->b->OnCollisionBegin(i->a);
		}

		CollisionDetection::CollisionInfo& in = const_cast<CollisionDetection::CollisionInfo&>(*i);
		in.framesLeft--;

		if ((*i).framesLeft < 0) {
			i->a->OnCollisionEnd(i->b);
			i->b->OnCollisionEnd(i->a);
			i = allCollisions.erase(i);
		}
		else {
			++i;
		}
	}
}

/**
 * @brief 更新所有游戏对象的轴对齐边界框（AABB）。
 * @details 这通常用作宽相碰撞检测的一部分。
 */
void PhysicsSystem::UpdateObjectAABBs() 
{
	gameWorld.OperateOnContents(
		[](GameObject* g) {
			g->UpdateBroadphaseAABB();
		}
	);
}

/**
 * @brief 执行基本的、暴力的碰撞检测。
 * @details 这是我们在教程4中进行碰撞检测的方式。
我们对每一对对象进行一次检查（内部的 for 循环偏移量确保了这一点），并确定它们是否碰撞，如果是，则将它们添加到碰撞集合中以供后续处理。该集合将保证特定的一对只被添加一次，因此碰撞多帧的对象不会因为重复而淹没集合。
 */
void PhysicsSystem::BasicCollisionDetection() 
{
	GameObjectIterator first;
	GameObjectIterator last;
	gameWorld.GetObjectIterators(first, last);

	for (auto i = first; i != last; ++i) {
		for (auto j = i + 1; j != last; ++j) {
			GameObject* a = *i;
			GameObject* b = *j;
			if (!a->GetBoundingVolume() || !b->GetBoundingVolume()) {
				continue;
			}
			CollisionDetection::CollisionInfo info;
			if (CollisionDetection::ObjectIntersection(a, b, info)) {
				ImpulseResolveCollision(*a, *b, info.point);
				info.framesLeft = numCollisionFrames;
				allCollisions.insert(info);
			}
		}
	}
}

/**
 * @brief 使用基于冲量的物理来解决两个对象之间的碰撞。
 * @details 在教程5中，我们开始确定对碰撞的正确响应，以便对象能够分开。
 * @param a 第一个游戏对象。
 * @param b 第二个游戏对象。
 * @param p 接触点信息。
 */
void PhysicsSystem::ImpulseResolveCollision(GameObject& a, GameObject& b, CollisionDetection::ContactPoint& p) const 
{
	PhysicsObject* physA = a.GetPhysicsObject();
	PhysicsObject* physB = b.GetPhysicsObject();
	if (!physA || !physB) {
		std::cout << "!physA || !physB" << std::endl;
		return;
	}
	float invMassA = physA->GetInverseMass();
	float invMassB = physB->GetInverseMass();

	if (invMassA + invMassB == 0.0f) {
		std::cout << "both static" << std::endl;
		return; // both static
	}

	Vector3 normal = Vector::Normalise(p.normal);
	if (Vector::LengthSquared(normal) < 1e-6f) {
		normal = Vector3(0, 1, 0);
	}
	Vector3 ab = b.GetTransform().GetPosition() - a.GetTransform().GetPosition();
	if (Vector::Dot(normal, ab) < 0.0f) {
		normal = -normal; // 确保法线从 a 指向 b
	}
	// Positional correction
	const float percent = 0.6f; // reduce correction to avoid jitter
	const float slop = 0.001f;
	float correctionMag = std::max(p.penetration - slop, 0.0f) / (invMassA + invMassB) * percent;
	Vector3 correction = normal * correctionMag;
	std::cout << "!hi" << std::endl;
	Transform& tA = a.GetTransform();
	Transform& tB = b.GetTransform();
	tA.SetPosition(tA.GetPosition() - correction * invMassA);
	tB.SetPosition(tB.GetPosition() + correction * invMassB);

	Vector3 relativeVel = physB->GetLinearVelocity() - physA->GetLinearVelocity();
	float velAlongNormal = Vector::Dot(relativeVel, normal);

	// If velocities are separating, still allow positional fix but skip impulse
	if (velAlongNormal > 0.0f) {
		return;
	}

	float restitution = 0.4f; // soften bounce to reduce sticking with static bodies
	float j = -(1.0f + restitution) * velAlongNormal;
	j /= (invMassA + invMassB);

	Vector3 impulse = normal * j;
	physA->SetLinearVelocity(physA->GetLinearVelocity() - impulse * invMassA);
	physB->SetLinearVelocity(physB->GetLinearVelocity() + impulse * invMassB);
}

/**
 * @brief 执行碰撞检测的宽相阶段。
 * @details 稍后，我们用宽相和窄相碰撞检测方法替换 BasicCollisionDetection 方法。在宽相中，我们使用加速结构来划分世界，这样我们就可以只比较我们绝对需要比较的碰撞。
 */
void PhysicsSystem::BroadPhase() 
{

}

/**
 * @brief 执行碰撞检测的窄相阶段。
 * @details 宽相现在只会给我们可能的碰撞，所以我们现在可以遍历它们，确定它们是否真的在碰撞，如果是，就将它们添加到主碰撞列表中。
 */
void PhysicsSystem::NarrowPhase() 
{

}

/**
 * @brief 积分加速度并更新线速度和角速度。
 * @details 加速度和速度的积分是分开的，这样我们就可以在一次物理更新过程中多次移动对象，而不必担心重复的力会累积等问题。

此函数将根据上一游戏帧期间对象中累积的任何力来更新线性和角加速度。
 * @param dt 固定的时间步长增量。
 */
void PhysicsSystem::IntegrateAccel(float dt) 
{
	gameWorld.OperateOnContents(
		[&](GameObject* o) {
			PhysicsObject* phys = o->GetPhysicsObject();
			if (!phys) {
				return;
			}

			float inverseMass = phys->GetInverseMass();
			if (inverseMass <= 0.0f) {
				return;
			}

			Vector3 linearAcceleration = phys->GetForce() * inverseMass;
			if (applyGravity) {
				linearAcceleration += gravity;
			}
			phys->SetLinearVelocity(phys->GetLinearVelocity() + linearAcceleration * dt);

			Vector3 angularAcceleration = phys->GetInertiaTensor() * phys->GetTorque();
			phys->SetAngularVelocity(phys->GetAngularVelocity() + angularAcceleration * dt);
		}
	);
}

/**
 * @brief 积分速度并更新位置和方向。
 * @details 此函数将线速度和角速度积分到位置和方向中。它可能会在一次物理更新中被多次调用，以缓慢地移动世界中的对象，寻找碰撞。
 * @param dt 固定的时间步长增量。
 */
void PhysicsSystem::IntegrateVelocity(float dt) 
{
	gameWorld.OperateOnContents(
		[&](GameObject* o) {
			PhysicsObject* phys = o->GetPhysicsObject();
			if (!phys) {
				return;
			}

			float inverseMass = phys->GetInverseMass();
			if (inverseMass <= 0.0f) {
				return;
			}

			Transform& transform = o->GetTransform();
			transform.SetPosition(transform.GetPosition() + phys->GetLinearVelocity() * dt);

			Vector3 angVel = phys->GetAngularVelocity();
			if (Vector::LengthSquared(angVel) > 0.0f) {
				Quaternion orientation = transform.GetOrientation();
				Quaternion spin = Quaternion(angVel.x * dt * 0.5f, angVel.y * dt * 0.5f, angVel.z * dt * 0.5f, 0.0f);
				orientation = orientation + spin * orientation;
				orientation.Normalise();
				transform.SetOrientation(orientation);
			}

			phys->SetLinearVelocity(phys->GetLinearVelocity() * globalDamping);
			phys->SetAngularVelocity(phys->GetAngularVelocity() * globalDamping);
		}
	);
}

/**
 * @brief 清除所有物理对象上累积的力。
 * @details 一旦我们完成了一次物理更新，我们就必须清除所有累积的力，为在下一个“游戏”帧中接收新的力做准备。
 */
void PhysicsSystem::ClearForces() 
{
	gameWorld.OperateOnContents(
		[](GameObject* o) {
			o->GetPhysicsObject()->ClearForces();
		}
	);
}


/**
 * @brief 更新物理世界中的所有约束。
 * @details作为最终物理教程的一部分，我们增加了基于一些额外计算来约束对象的能力，从而可以模拟弹簧和绳索等。
 * @param dt 约束求解器的时间步长增量。
 */
void PhysicsSystem::UpdateConstraints(float dt) 
{
	std::vector<Constraint*>::const_iterator first;
	std::vector<Constraint*>::const_iterator last;
	gameWorld.GetConstraintIterators(first, last);

	for (auto i = first; i != last; ++i) {
		(*i)->UpdateConstraint(dt);
	}
}
