# Smart Flying Navigation Plugin for Unreal Engine

This repository contains the documentation and resources for the **Smart Flying Navigation** plugin, an advanced Octree-based voxel navigation solution designed for 3D environments, particularly for flying agents within Unreal Engine.

## Key Features

*   **Octree-based Voxel Grid**: Efficiently represents 3D navigable space.
*   **Dynamic Obstacle Avoidance**: Enables agents to react to moving obstacles in real-time.
*   **Multi-Agent Support**: Facilitates coordinated movement for multiple agents.
*   **Customizable Navigation Parameters**: Allows fine-tuning of pathfinding behavior.
*   **Debug Visualization**: Provides clear visual feedback for navigation data and agent paths.

## Quick Start Guide

Follow these steps to integrate and utilize the Smart Flying Navigation plugin in your Unreal Engine project:

### 1. Set up the Navigation Volume

Place an `OctreeVoxelVolume` Actor in your level to define the 3D navigable space. Configure its properties using the dedicated Pathfinding Tuner UI.

### 2. Bake Navigation Data

Convert your static environment geometry into an optimized voxel grid. This process generates a `BakedOctreeDataAsset` which is used for runtime pathfinding.

### 3. Set up the Agent

Equip your flying agents with a `UPathfindingComponent`. This component allows them to utilize the baked navigation data. Define a `TargetActor` to set their pathfinding goals.

### 4. Execute a Pathfinding Test

In Play In Editor (PIE) mode, call pathfinding functions (e.g., `Request Path to Target Actor` or `Request Path to Location`) on your agent's `PathfindingComponent` to observe their navigation behavior.

## Visuals and Further Documentation

For detailed visual examples, screenshots, and comprehensive documentation, please refer to the [TeamHamDev GitHub Pages website](https://teamhamdev.github.io/).

*   [Quick-Start Guide](https://teamhamdev.github.io/docs/quick-start-guide/)
*   [Component Reference](https://teamhamdev.github.io/docs/component-reference/)
