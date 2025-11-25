# Repository Guidelines

## Project Structure & Module Organization
- `CSC8503/` holds the gameplay entry point, render wrappers, and scenes; `Main.cpp` launches `TutorialGame`.
- `CSC8503CoreClasses/` contains physics, AI, and networking primitives (e.g., `PhysicsSystem`, `NavigationGrid`, `NetworkBase`).
- `NCLCoreClasses/` provides math, window/input, timing, and rendering helpers plus `stb` utilities.
- `OpenGLRendering/` and `VulkanRendering/` are backend-specific pipelines; `GLTFLoader/` manages asset import; shared art sits in `Assets/`; build outputs land in `build/` or `x64/`.

## Build, Test, and Development Commands
- Configure (OpenGL default): `cmake -S . -B build -G "Visual Studio 17 2022" -A x64 -DUSE_OPENGL=ON -DUSE_VULKAN=OFF`.
- Build Debug: `cmake --build build --config Debug`; Release: `cmake --build build --config Release`.
- Toggle backends: set `-DUSE_VULKAN=ON` or `-DUSE_OPENGL=OFF` as needed; Vulkan requires the SDK on PATH.
- Run the game: `build/Debug/CSC8503.exe` (or via the generated Visual Studio solution; startup project is `CSC8503`).

## Coding Style & Naming Conventions
- C++20 across the codebase; mirror existing files: tabs for indentation, braces on the same line as control statements, and short include blocks grouped by module.
- Classes/structs use PascalCase, functions camelCase, and constants/macros uppercase (e.g., `ASSETROOTLOCATION`).
- Keep headers light, prefer `#pragma once`, and avoid unnecessary copies; prefer `const &` for read-only parameters.

## Testing Guidelines
- No automated test target yet; rely on runtime checks. Always run a Debug build after changes and watch for console output or `Debug::` overlays.
- For feature work, sanity-check both `Debug` and `Release` builds and the backend you touched (OpenGL by default, Vulkan when enabled).
- Verify frame time stability (<0.1s spikes are skipped) and ensure new assets resolve from `Assets/`.

## Commit & Pull Request Guidelines
- There is no existing Git history; use imperative, scoped commit subjects (e.g., "Add player respawn cooldown").
- Each PR should state the feature/fix, the backend tested (OpenGL/Vulkan), repro steps, and screenshots or clips for visual changes.
- Note any new assets and their sources/licenses; keep PRs small and focused, and link issues or tasks when available.

## Assets & Configuration Notes
- Assets load via the `ASSETROOTLOCATION` define pointing to `Assets/`; keep new files organized by type and strip unused content from diffs.
- When switching renderers, keep the inactive backend buildable—avoid hardcoded API calls in shared modules.
