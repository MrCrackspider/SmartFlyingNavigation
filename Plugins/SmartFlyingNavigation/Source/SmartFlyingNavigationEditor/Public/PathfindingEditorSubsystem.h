// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "EditorSubsystem.h"
#include "PathfindingEditorSubsystem.generated.h"

class AOctreeVoxelVolume;
class UPathfindingTunerSettings;
class ULandscapeComponent;
struct FOctreeNode;

#if WITH_EDITOR
/**
 * A temporary struct to hold the results of a single volume bake during the world bake process.
 */
struct FWorldBakeResult
{
	AOctreeVoxelVolume* Volume;
	TArray<FOctreeNode> NodePool;
};
#endif

/**
 * Manages all editor-only pathfinding operations, primarily the baking process for OctreeVoxelVolumes.
 * This subsystem centralizes the logic that was previously in SPathfindingTuner.
 */
UCLASS()
class SMARTFLYINGNAVIGATIONEDITOR_API UPathfindingEditorSubsystem : public UEditorSubsystem
{
	GENERATED_BODY()

#if WITH_EDITOR
public:
	/**
	 * Bakes a specific list of already-existing OctreeVoxelVolumes.
	 * This is the entry point for "Bake Selected".
	 * @param VolumesToBake The list of volumes to process.
	 */
	void BakeSpecificVolumes(const TArray<AOctreeVoxelVolume*>& VolumesToBake);

	/**
	 * Bakes the entire world by creating a grid of volumes, baking them, and connecting portals.
	 * This is the entry point for "Bake World".
	 * @param TunerSettings The settings asset that defines how the world bake should operate.
	 */
	//void BakeEntireWorld(const UPathfindingTunerSettings* TunerSettings);

private:
	// --- Helper functions for the World Bake process ---

	/** Deletes all previously baked OctreeVoxelVolume actors and their associated Data Assets. */
	void CleanupPreviousAssets(UWorld* InWorld, const FString& InSavePath) const;

	/** Calculates the total bounds of all static meshes and landscapes in the world to determine the bake area. */
	bool CalculateWorldBakeBounds(UWorld* InWorld, FBox& OutBounds) const;

	/** Spawns volumes in a grid, bakes their internal data, and collects the results. */
	bool BakeAndCreateVolumesInGrid(
		UWorld* InWorld,
		const FBox& InWorldBounds,
		const UPathfindingTunerSettings* InSettings,
		TArray<FWorldBakeResult>& OutBakeResults);

	/** Saves all the generated Data Assets and the level containing the new volume actors. */
	void SaveBakedResults(const TArray<TObjectPtr<UPackage>>& InPackagesToSave, const TArray<TObjectPtr<AActor>>& InCreatedActors) const;

	/** Temporarily enables collision on all landscape components to ensure they are included in the bake. */
	void SetupLandscapeCollisions_Internal(UWorld* InWorld, TMap<ULandscapeComponent*, TEnumAsByte<ECollisionEnabled::Type>>& OutOriginalStates) const;

	/** Restores the original collision settings of landscape components after the bake is complete. */
	void RestoreLandscapeCollisions_Internal(const TMap<ULandscapeComponent*, TEnumAsByte<ECollisionEnabled::Type>>& InOriginalStates) const;

	void BakeAndConnectVolumes_Internal(const TArray<AOctreeVoxelVolume*>& VolumesToProcess, UWorld* InWorld);

#endif
};
