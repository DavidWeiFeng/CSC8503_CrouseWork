# Player Movement & Physics Bootstrap

## What was added
- **Player control** (TutorialGame): camera-relative WASD force, jump impulse on grounded check (short downward ray), horizontal damping (ground/air). Player object is stored when created.
- **Physics integration** (PhysicsSystem): basic linear integration so forces and gravity affect velocity/position; global damping applied per step; skips static bodies (inverseMass == 0) to avoid moving floors.

## How it works
- `UpdatePlayerMovement` (TutorialGame.cpp):
  - Builds right/forward vectors from the main camera (Y flattened), normalises them, accumulates WASD input as a force, scales by `playerMoveForce`, and calls `AddForce`.
  - If grounded and Space pressed, applies `ApplyLinearImpulse` upward using `playerJumpImpulse`.
  - Applies horizontal damping each frame (`groundMoveDamping` or `airMoveDamping`) by scaling X/Z velocity components.
- `IsPlayerGrounded`: casts a short downward ray from the player to detect ground within ~1.1 units, ignoring the player.
- `IntegrateAccel` / `IntegrateVelocity` (PhysicsSystem.cpp):
  - Acceleration = gravity (if enabled) + accumulated force * inverseMass.
  - Velocity integrates from acceleration; position integrates from velocity; multiplies velocity by `globalDamping` each step.
  - Does not yet handle angular velocity, collisions, constraints, or sleeping.

## Tuning knobs (TutorialGame.h)
- `playerMoveForce` (default 150) - movement thrust.
- `playerJumpImpulse` (30) - hop height.
- `groundMoveDamping` (0.92) - friction-like horizontal decay when grounded.
- `airMoveDamping` (0.98) - drag while airborne.

## Known gaps / next steps
- Collision detection/response functions are still stubs; objects will overlap until `BasicCollisionDetection` and `ImpulseResolveCollision` are implemented.
- No angular motion or torque for the player yet.
- Ground check uses a simple ray; replace with contact accumulation once collision system is live.
