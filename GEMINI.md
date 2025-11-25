# Gemini Project Context: CSC8503 Coursework

This document provides context for the CSC8503 Coursework project, a C++ game engine framework.

## Project Overview

This is a C++20 project utilizing CMake for its build system. It appears to be a game or game engine, with a modular structure that separates core logic, rendering, and gameplay code. The project is licensed under the MIT License.

The architecture includes:
- A core gameplay module (`CSC8503`).
- A module for core classes like physics, AI, and networking (`CSC8503CoreClasses`).
- A foundational module for math, windowing, and input (`NCLCoreClasses`).
- Switchable rendering backends for OpenGL and Vulkan (`OpenGLRendering`, `VulkanRendering`).
- A GLTF asset loader (`GLTFLoader`).

## Building and Running

The project uses CMake and is configured to be built with Visual Studio 2022 on Windows (x64).

**1. Configure CMake (OpenGL default):**
```shell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 -DUSE_OPENGL=ON -DUSE_VULKAN=OFF
```

**2. Build the Project:**
*   **Debug:**
    ```shell
    cmake --build build --config Debug
    ```
*   **Release:**
    ```shell
    cmake --build build --config Release
    ```

**3. Run the Game:**
The main executable will be located in the build directory.
```shell
./build/Debug/CSC8503.exe
```
Alternatively, open the generated `build/CSC8503.sln` file in Visual Studio and run the `CSC8503` project.

**Renderer Backend:**
You can switch to the Vulkan renderer by changing the CMake options: `-DUSE_OPENGL=OFF -DUSE_VULKAN=ON`. This requires the Vulkan SDK to be installed and accessible in your system's PATH.

## Development Conventions

### Coding Style
- **C++ Standard:** C++20.
- **Indentation:** Tabs.
- **Braces:** Opening braces on the same line as the control statement.
- **Headers:** Use `#pragma once` for include guards. Keep include blocks short and grouped by module.
- **Parameters:** Pass read-only, non-trivial parameters by `const &`.

### Naming Conventions
- **Classes/Structs:** `PascalCase`
- **Functions:** `camelCase`
- **Constants/Macros:** `UPPERCASE_SNAKE_CASE`

### Project Structure
- `CSC8503/`: Main gameplay entry point (`Main.cpp`, `TutorialGame.cpp`), and renderer wrappers.
- `CSC8503CoreClasses/`: Physics (`PhysicsSystem`), AI (`NavigationGrid`), and networking (`NetworkBase`) primitives.
- `NCLCoreClasses/`: Low-level utilities for math, window management, input handling, and timing.
- `OpenGLRendering/` & `VulkanRendering/`: Backend-specific rendering pipelines.
- `GLTFLoader/`: GLTF asset import functionality.
- `Assets/`: Directory for all game assets.
- `build/` or `x64/`: Default build output directories.
