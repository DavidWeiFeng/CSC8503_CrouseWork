# Repository Guidelines

## Project Structure & Module Organization
- Root CMake drives subprojects: `NCLCoreClasses` (math, assets, input), `CSC8503CoreClasses` (game framework), `CSC8503` (playable client), `GLTFLoader`, and renderer backends in `OpenGLRendering` and `VulkanRendering`.
- Game data lives in `Assets/` (shaders, models, nav meshes); build products appear in `build/`, `x64/`, or `out/build/<config>/` when using VS/Ninja presets.
- Shared CMake helpers sit in `CMake/`; renderer selection is controlled via `USE_VULKAN` and `USE_OPENGL`, with `ASSET_ROOT` exposed to code as `ASSETROOTLOCATION`.

## Build, Test, and Development Commands
- Configure (Ninja preset): `cmake -S . -B out/build/x64-Debug -G "Ninja" -DUSE_VULKAN=ON` (mirrors `CMakeSettings.json`).
- Alternative VS solution: `cmake -S . -B build -G "Visual Studio 17 2022" -A x64 -DUSE_VULKAN=OFF -DUSE_OPENGL=ON`.
- Build: `cmake --build out/build/x64-Debug` (or `cmake --build build --config Debug`).
- Run: `./out/build/x64-Debug/CSC8503.exe` (or `./build/Debug/CSC8503.exe`) with working dir so `Assets/` is discoverable.

## Coding Style & Naming Conventions
- C++20, prefer existing tab indentation and brace placement; keep headers using `#pragma once`.
- Classes/types use `PascalCase`; methods follow the same; member variables and locals use `camelCase`; constants/macros stay `ALL_CAPS`.
- Organize includes from local headers to core headers; guard against Windows macro collisions with `WIN32_LEAN_AND_MEAN` already defined in CMake.

## Testing Guidelines
- No automated test suite is present; add new checks using `ctest`-friendly targets if you introduce them.
- Manually validate new features by running the client and exercising scene loads, input, and renderer mode (OpenGL vs Vulkan) you touched.

## Commit & Pull Request Guidelines
- Current history is sparse; keep commits small with clear, present-tense subjects (e.g., `render: tidy camera frustum setup`).
- For PRs, include: a short summary, build config used (`Debug/Release`, `USE_VULKAN/USE_OPENGL`), steps to verify, and screenshots or clips for visual changes.

## Configuration Tips
- Project targets Windows x64; ensure matching SDK and GPU drivers for the renderer you enable.
- Keep `Assets/` alongside the executable or update `ASSET_ROOT` consistently when changing layouts.
