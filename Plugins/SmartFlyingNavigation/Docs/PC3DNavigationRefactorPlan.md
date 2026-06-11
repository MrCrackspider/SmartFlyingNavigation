# PC 3D Navigation Refactor Plan

## Scope

This plan targets the PC runtime path first. Android/mobile-lite airway graph work is intentionally deferred.

The current plugin already has useful pieces:

- Static octree bake.
- Sparse dynamic obstacle bitmask.
- Agent-radius-aware segment checks.
- A* with Lazy Theta style smoothing.
- ORCA-style local avoidance.
- Slate tooling for bake and runtime tuning.

The main issue is not a single algorithm. The issue is ownership: pathfinding, path following, ORCA, sweep movement, bake metadata, and editor validation currently overlap. The refactor should make the data and responsibility boundaries explicit.

## Research Notes

Relevant findings from the literature:

- Massonnat et al., "Efficient Octree-based 3D Pathfinding", describes an octree-based 3D navigation mesh, movement graph over transition surfaces, Hertel-Mehlhorn style cell merging, path pruning, a 3D funnel-style refinement, and local graph updates for dynamic environments. This is the closest match to this plugin's needs because it is game-oriented, 3D, octree-based, and tested on large maps including Warframe-style benchmarks.
  - https://www.sable.mcgill.ca/~clump/papers/massonnat-24-efficient.pdf
- "Efficient Lazy Theta* Path Planning over a Sparse Grid to Explore Large 3D Volumes with a Multirotor UAV" supports keeping any-angle planning, but highlights the importance of maintaining a free flight corridor, not just a centerline path.
  - https://pmc.ncbi.nlm.nih.gov/articles/PMC6339096/
- ORCA is a local collision avoidance method, not a global motion planner. The original paper explicitly separates local collision avoidance from complete path planning. It assumes known neighbor state and chooses velocities from permitted half-planes.
  - https://gamma.cs.unc.edu/ORCA/publications/ORCA.pdf
- JPS-3D is a strong speed baseline for uniform voxel grids, but the paper notes large voxel-grid state spaces and that SVO paths can be poor when defined only through octant centers. This supports keeping octree for memory and coverage, while improving the path representation beyond center-to-center nodes.
  - https://ojs.aaai.org/index.php/SOCS/article/download/21762/21526/25805

Conclusion: do not replace the current octree with a pure voxel grid. The better direction is to evolve the octree into a 3D navigation cell/portal/corridor representation, then run A* over that graph and refine the path inside the corridor.

## Target Architecture

### 0. Baked Data Ownership

The current saved-data model is not suitable as the final open-world model:

- One `UOctreeDataAsset` is produced per `AOctreeVoxelVolume`.
- Each asset stores compressed `FOctreeNode` data, compressed `FAirwayGraphNode` data, and uncompressed portal maps.
- `FOctreeNode` and `FAirwayGraphNode` duplicate center/neighbor/clearance information in different shapes.
- Portal links use soft object references between volume data assets, which becomes expensive to manage when many small volumes are baked.

Final PC/open-world direction:

- Keep one world-level navigation manifest per map or World Partition data layer.
- Store many baked chunks inside a small number of package assets or external bulk-data payloads, not one asset per small volume.
- Use stable chunk ids, not soft asset references, for cross-chunk portals.
- Keep editor actors as authoring/bake bounds only; runtime should load chunk records from the manifest.
- Split heavy runtime data by need:
  - Cell/portal graph for normal PC pathfinding.
  - Optional full octree payload only for debug or high-precision fallback.
  - Optional lightweight airway graph for mobile or low-memory modes.
- Store build hashes per chunk so unchanged chunks can be skipped safely.

This is the real fix for large projects. The current loaded-volume bake UX only reduces the chance of accidental expensive bakes; it does not solve asset count or duplicated payloads by itself.

Current migration decision:

- `UWorldNavigationDataAsset` is the primary saved navigation asset.
- Per-volume `UOctreeDataAsset` is now treated as a legacy compatibility payload while runtime loading is migrated.
- New manifest chunks store stable chunk ids and flattened portals; legacy soft asset references are kept only as transitional payload pointers.
- Runtime now loads the world manifest outside editor-only code and builds chunk-id mappings through `UOctreeNavSubsystem`.

### 1. Navigation Query Layer

Single authority for collision queries:

- `IsPointBlockedForAgent(Location, AgentRadius, bCheckStatic, bCheckDynamic)`
- `IsSegmentBlockedForAgent(Start, End, AgentRadius, bCheckStatic, bCheckDynamic)`
- future: `GetSegmentClearance(Start, End)` and `GetCellClearance(NodeIndex)`

Status: initial version added in `FOctreePathfindingData` and `AOctreeVoxelVolume`.

### 2. Baked Cell Graph

The bake output should be more than leaf center neighbors.

Add a baked graph abstraction:

- Cell id.
- Cell bounds.
- Clearance.
- Adjacent portals.
- Portal center.
- Portal plane/normal.
- Portal half extents or rectangle bounds.
- Optional portal clearance.

This can be generated from current `FOctreeNode::Neighbors` first, then improved later with merged cells.

### 3. Path Corridor

Path result should become a corridor, not only `TArray<FVector>`.

Proposed runtime data:

```cpp
struct FNavPathPortal
{
    int32 FromCell = -1;
    int32 ToCell = -1;
    FVector Center;
    FVector Normal;
    FVector ExtentU;
    FVector ExtentV;
    float Clearance = 0.f;
};

struct FNavPathCorridor
{
    TArray<int32> CellPath;
    TArray<FNavPathPortal> Portals;
    TArray<FVector> Waypoints;
    float RequiredAgentRadius = 0.f;
    bool bDynamicValidated = false;
};
```

The current waypoint array can remain as a compatibility output, but internally the component should own a corridor.

### 4. Planner

Keep PC A* as the first planner, but change successor acceptance:

- Candidate neighbor must have enough clearance.
- The transition portal must have enough clearance.
- Optional dynamic validation checks the segment or portal region.

Path smoothing should become two stages:

- visibility pruning using `IsSegmentBlockedForAgent`
- corridor-aware refinement that never leaves the valid cell path

This avoids a common failure mode where smoothing cuts across a wall because endpoints are individually valid.

### 5. Local Avoidance

ORCA should not be responsible for static walls or global route decisions.

Pipeline:

1. Path corridor computes `PreferredVelocity`.
2. ORCA adjusts for other agents and optionally moving dynamic obstacles.
3. Corridor clamp projects or rejects the ORCA velocity if it would leave the path corridor.
4. Movement executor applies the final velocity.
5. Repath triggers if the agent cannot make progress within the corridor.

Static geometry should be handled by the navigation query/corridor, with movement sweep as the last physical safety check only.

### 6. Dynamic Obstacles

Dynamic obstacle handling should be split:

- Small moving agents: ORCA.
- Larger blocking objects: dynamic bitmask invalidates affected path segments/corridors.

Do not rely on ORCA to solve blocked passages. If a dynamic obstacle occupies a portal or key corridor section, the global path must be invalidated and replanned.

### 7. Bake and Editor UX

The bake UI must expose the bake constraints before running:

- Voxel extent must be positive.
- Current octree bake requires cubic power-of-two map size.
- Baked asset is stale if map size, voxel extent, level, or max octree depth changed.
- Future: show estimated cell count and memory cost before bake.

Status: the Bake button now blocks invalid voxel extent and non-cubic/non-power-of-two map sizes.

Additional editor work added:

- `UPathfindingEditorSubsystem` owns loaded-volume discovery, shared bake validation, and loaded-volume bake orchestration.
- Slate now has a World Partition workflow section that summarizes loaded nav volumes and can select or bake the loaded set.
- In World Partition maps, this is intentionally scoped to currently loaded cells. It is the first UX step before adding background cell bake queues or streaming-aware manifests.
- Bake confirmation now reports an estimated voxel/component/storage budget before expensive bakes.

## Implementation Phases

### Phase 1: Query Unification

Done:

- Unified segment/point checks.
- A* and path following use the same checks.
- Agent radius changes invalidate path.
- Dynamic bitmask indexing no longer requires `MapSize.X % 128 == 0`.
- Bake and Slate UI validate core octree assumptions.

### Phase 2: Corridor Data Model

Started:

- Add `FNavPathPortal` and `FNavPathCorridor`.
- Store corridor in `UPathfindingComponent`.
- Keep existing `FoundPath` for Blueprint compatibility.

Next:

- Return both corridor and waypoint path from planner internally.

### Phase 3: Portal Graph

Next:

- Convert leaf-neighbor adjacency to portal records.
- Compute portal clearance.
- Use portal clearance during A* expansion.
- Add debug drawing for cell path and portals in the tuner.

### Phase 4: Corridor-Constrained Following

Next:

- Add `ClampVelocityToCorridor`.
- ORCA output must pass through this before movement.
- Trigger repath on sustained corridor violation or no-progress state.

### Phase 5: Bake Scalability

Next:

- Separate bake orchestration from `AOctreeVoxelVolume`.
- Add progress/report object for Slate UI.
- Add pre-bake validation summary and estimated cost.
- Add optional merged-cell bake path after current leaf graph is stable.

### Phase 6: Baked Data Format

Started:

- Introduce `UWorldNavigationDataAsset` as the primary runtime entry point.
- Add chunk records with stable ids, bounds, transform, hash, compressed payload offset, and portal ranges.
- Replace `FVolumePortalLink::TargetVolume` soft asset references with target chunk id + node/cell id.
- Add runtime manifest lookup APIs for chunk-by-location and portals-by-chunk.

Next:

- Stop saving both full octree and airway graph by default. Save the selected runtime representation and make debug/fallback payloads optional.
- Add migration compatibility so existing `UOctreeDataAsset` volumes still load while the new format is introduced.

## Decisions

- Keep octree for PC.
- Do not move to pure voxel-grid JPS-3D as the main representation yet.
- Treat ORCA as local agent-agent avoidance, not path planning.
- Treat dynamic blocking obstacles as path invalidation inputs.
- Promote corridor to a first-class runtime path object.
