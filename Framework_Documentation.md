# 游戏框架代码库说明文档

## 1. 简介

这是一个功能强大的C++游戏开发框架，主要用于教学目的。它采用模块化设计，将底层功能、核心引擎系统和顶层游戏逻辑清晰地分离开来。框架内置了完整的3D渲染管线、物理引擎、网络功能和AI系统，为您提供了一个稳固的游戏开发基础。

## 2. 项目架构

该框架的核心设计思想是**分层结构**，主要由以下三个部分组成：

- **`NCLCoreClasses` (底层核心库)**
  - **职责**: 提供最基础的、与平台和图形API相关的底层功能。
  - **包含**:
    - 数学库 (`Maths.h`, `Matrix.h`, `Quaternion.h`)
    - 窗口和输入管理 (`Keyboard.h`, `Mouse.h`)
    - 渲染抽象基类 (`RendererBase.h`, `Shader.h`, `Mesh.h`, `Texture.h`)
    - 资源加载 (`Assets.h`)

- **`CSC8503CoreClasses` (中层引擎核心)**
  - **职责**: 实现游戏引擎的核心 gameplay 系统，不依赖于特定的图形API。
  - **包含**:
    - **游戏世界与对象**: `GameWorld.h`, `GameObject.h`
    - **物理系统**: `PhysicsSystem.h`, `CollisionDetection.h` (自定义的3D物理引擎)
    - **网络**: `NetworkBase.h`, `GameClient.h`, `GameServer.h` (基于 ENet 库的客户端-服务器架构)
    - **AI**: `StateMachine.h` (状态机), `BehaviourNode.h` (行为树)
    - **场景管理**: `QuadTree.h` (四叉树空间划分)

- **`CSC8503` (顶层应用)**
  - **职责**: 游戏的具体实现和入口。
  - **包含**:
    - **主程序入口**: `Main.cpp`
    - **渲染器实现**: `GameTechRenderer.h` (基于OpenGL的高级渲染器)
    - **游戏逻辑**: `TutorialGame.h`, `NetworkedGame.h` (具体游戏玩法的实现)

## 3. 核心功能详解

### 3.1 游戏对象 (GameObject)

游戏世界的核心是 `GameObject` (`CSC8503CoreClasses/GameObject.h`)。它采用了一种混合式的面向对象设计：

- 每个 `GameObject` 是一个独立的实体。
- 它不使用动态组件系统，而是内置了固定的、可选的“组件”指针，如 `RenderObject`, `PhysicsObject`, `NetworkObject`。您可以通过为 `GameObject` 添加这些对象来赋予它渲染、物理或网络能力。

### 3.2 渲染系统

渲染系统是抽象化的，允许未来支持不同的图形API（如Vulkan）。

- **抽象层**: `NCLCoreClasses/RendererBase.h` 定义了所有渲染器都必须遵守的接口，如 `BeginFrame()`, `RenderFrame()`, `EndFrame()`。
- **实现层**: `CSC8503/GameTechRenderer.h` 是一个功能丰富的OpenGL渲染器，它继承自 `RendererBase` 并实现了高级功能，包括：
  - 多渲染通道 (Multi-pass rendering)
  - 阴影贴图 (Shadow Mapping)
  - 天空盒 (Skybox)
  - 透明物体排序与渲染
  - Gamma校正
  - 调试信息绘制

### 3.3 物理系统

框架包含一个**自定义的3D物理引擎** (`CSC8503CoreClasses/PhysicsSystem.h`)，而非使用第三方库（如PhysX或Bullet）。

- **碰撞检测**:
  - **宽阶段 (Broadphase)**: 使用 `QuadTree` (2D) 或类似的加速结构快速筛选可能碰撞的对象对。
  - **窄阶段 (Narrowphase)**: 使用 GJK 算法 (`CollisionDetection.h`) 精确检测各种碰撞体（AABB, OBB, Sphere, Capsule）之间的碰撞。
- **碰撞响应**: 采用基于冲量 (Impulse-based) 的方法来解决碰撞和模拟物理效果。
- **约束 (Constraints)**: 支持位置和方向约束，可用于创建铰链、布娃娃等效果。

### 3.4 网络系统

网络部分基于 `enet` 库实现了一个可靠的客户端-服务器 (Client-Server) 模型。

- **核心**: `NetworkBase.h`, `GameClient.h`, `GameServer.h`。
- **网络对象**: `NetworkObject.h` 用于将游戏对象的状态在网络中同步。
- **示例**: `NetworkedGame.h` 展示了如何创建一个简单的网络游戏。

### 3.5 AI 系统

框架提供了两种主流的AI决策模型：

1.  **状态机 (State Machine)**:
    - 定义于 `StateMachine.h`。
    - 适用于简单、状态明确的AI行为（如：巡逻 -> 追击 -> 攻击）。
    - `StateGameObject.h` 是一个使用状态机的 `GameObject` 示例。

2.  **行为树 (Behavior Tree)**:
    - 定义于 `BehaviourNode.h` 及其派生类 (`BehaviourSelector`, `BehaviourSequence`)。
    - 适用于更复杂的、分层的AI逻辑。

## 4. 如何开始

- **项目入口**: `CSC8503/Main.cpp` 是整个程序的起点。
- **核心逻辑**: `main` 函数会创建 `TutorialGame` 或 `NetworkedGame` 的实例。
- **游戏循环**: `TutorialGame::UpdateGame(float dt)` 是游戏的主更新循环，所有游戏逻辑、物理模拟和渲染都在这里被驱动。

要开始您的项目，建议从修改 `TutorialGame.cpp` 开始，添加您自己的 `GameObject`，并与物理和渲染系统进行交互。
