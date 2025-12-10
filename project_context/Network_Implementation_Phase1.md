# Fragile Express - 网络联机系统实现文档 (Phase 1)

## 1. 概述
本文档描述了 Fragile Express 多人联机模式的核心实现细节。该系统采用 **Client-Server 架构**，**服务器权威 (Server Authoritative)** 模式。目前已完成基础连接、输入同步、状态广播及客户端身份握手。

## 2. 核心架构

### 2.1 类职能
*   **`NetworkedGame`**: 继承自 `TutorialGame`，负责管理 `GameServer` 和 `GameClient` 实例，处理网络数据包的分发与游戏循环。
*   **`GameServer`**: 基于 ENet，负责监听端口 (Default: 1234)，管理 Peer 连接，处理 `PlayerInputPacket`，广播 `FullPacket`。
*   **`GameClient`**: 基于 ENet，连接服务器，发送 `PlayerInputPacket`，接收并处理 `FullPacket` 以更新本地代理对象。
*   **`NetworkObject`**: 负责 GameObject 的序列化与反序列化（目前主要使用 `ReadFullPacket` / `WriteFullPacket`）。

### 2.2 数据流向
1.  **Input**: Client 采集键盘输入 (WASD/Space) -> `PlayerInputPacket` -> Server。
2.  **Simulation**: Server 收到 Input -> `latestClientInput` 队列 -> `UpdateAsServer` 每帧调用 `MovePlayer` 应用物理力 -> 物理引擎计算新位置。
3.  **State**: Server 物理计算后 -> `BroadcastSnapshot` -> `FullPacket` (Position/Orientation) -> Client。
4.  **Render**: Client 收到 `FullPacket` -> 更新本地 `GameObject` 位置 -> 渲染下一帧。

---

## 3. 关键机制实现

### 3.1 身份握手 (Handshake)
解决 "Client 不知道自己控制哪个物体" 的问题。
1.  **Client 连接**: 发送首个 Input 包。
2.  **Server 响应**:
    *   发现是新 Client -> 生成新 Player 实体 -> 分配 `NetworkID` (1000 + peerID)。
    *   发送 `MessagePacket (ID: PLAYER_ID_MSG)` 告知 Client 该 ID。
3.  **Client 绑定**:
    *   收到 `PLAYER_ID_MSG` -> 记录 `localNetworkID`。
    *   收到 `FullPacket` -> 若包中 ID == `localNetworkID` -> 将该物体赋值给 `playerObject` -> 摄像机自动跟随。

### 3.2 物理同步 (Server Authoritative)
*   **移动逻辑**: 重构了 `MovePlayer` 函数，由 Server 端统一调用，包含摩擦力、速度上限、地面检测。
*   **防作弊**: Client 只发按键状态，不发位置。
*   **防穿模**: Server 端强制重置物理状态，防止初始生成时的 Tunneling 效应。

### 3.3 世界初始化 (World Sync)
*   **Server**: `InitWorld()` -> `BuildSlopeScene()` -> 生成完整物理世界（包含动态物体）。
*   **Client**: `InitWorld()` -> `BuildSlopeScene()` -> **立即删除本地生成的 `playerObject`**。
    *   Client 的动态物体（玩家、敌人等）完全依赖 Server 的 `FullPacket` 自动生成 (Proxy Spawning)。
    *   静态物体（地板、墙壁）目前由 Client 本地生成（未来 Phase 4 需优化同步）。

---

## 4. 协议定义 (Packets)

| Packet Type | ID | 用途 | 数据内容 |
| :--- | :--- | :--- | :--- |
| **Full_State** | 5 | 服务器->客户端：位置同步 | ObjectID, Position, Orientation |
| **Player_Input** | 10 | 客户端->服务器：操作指令 | WASD, Jump, Grab, CameraYaw |
| **Message** | 2 | 双向：通用消息/握手 | MessageID (31=Handshake), PlayerID |

---

## 5. 使用说明 (测试方法)

1.  **启动 Server (Host)**:
    *   运行游戏 -> 按 `Enter` 进入场景 -> 按 **`F9`**。
    *   Server 玩家 (ID 100) 生成，摄像机绑定。

2.  **启动 Client**:
    *   运行游戏 -> 按 `Enter` 进入场景 -> 按 **`F10`**。
    *   Client 自动连接本地 Server。
    *   Server 生成 Client 玩家 (ID 1000+)，Client 收到握手包，摄像机绑定。

---

## 6. 下一步计划 (Phase 2 & 3)

*   **高分榜系统**: 实现文件 I/O 与网络请求。
*   **双人协作逻辑**: 实现重物 (Heavy Object) 的双人抓取检测与质量切换。
*   **网络优化**: 启用 Delta Compression (`WriteDeltaPacket`) 减少带宽。
