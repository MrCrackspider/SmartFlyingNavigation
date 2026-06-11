// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "EditorSubsystem.h"
#include "PathfindingEditorSubsystem.generated.h"

class AOctreeVoxelVolume;
class UPathfindingTunerSettings;
class UWorld;
class UWorldNavigationDataAsset;
struct FOctreeNode;
struct FNavPortalBakeLink;
struct FNavDistanceField;

#if WITH_EDITOR
struct FNavVolumeBakeIssue
{
	TWeakObjectPtr<AOctreeVoxelVolume> Volume;
	FText Message;
};

struct FLoadedNavVolumeSummary
{
	int32 Total = 0;
	int32 MissingBakeData = 0;
	int32 Stale = 0;
	int32 Invalid = 0;
};

struct FNavBakeCostEstimate
{
	int32 InputVolumeCount = 0;
	int32 VolumesToBake = 0;
	int32 UpToDateSkipped = 0;
	int32 RelevantComponentCount = 0;
	int64 TotalVoxelCount = 0;
	int64 MaxVolumeVoxelCount = 0;
	int64 EstimatedWorstCaseUncompressedBytes = 0;
	bool bLikelyExpensive = false;
};
#endif

/**
 * Manages all editor-only pathfinding operations, primarily the baking process for OctreeVoxelVolumes.
 * Bake output is written exclusively into the world navigation manifest (UWorldNavigationDataAsset).
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
	 */
	void BakeSpecificVolumes(const TArray<AOctreeVoxelVolume*>& VolumesToBake);

	void BakeLoadedVolumes(UWorld* InWorld);

	void GetLoadedOctreeVolumes(UWorld* InWorld, TArray<AOctreeVoxelVolume*>& OutVolumes) const;
	void GetVolumesNeedingBake(const TArray<AOctreeVoxelVolume*>& CandidateVolumes, TArray<AOctreeVoxelVolume*>& OutVolumes) const;
	FLoadedNavVolumeSummary GetLoadedVolumeSummary(UWorld* InWorld) const;
	FNavBakeCostEstimate EstimateBakeCost(UWorld* InWorld, const TArray<AOctreeVoxelVolume*>& CandidateVolumes, bool bOnlyVolumesNeedingBake) const;
	FText FormatBakeCostEstimate(const FNavBakeCostEstimate& Estimate) const;
	bool ValidateVolumesForBake(const TArray<AOctreeVoxelVolume*>& VolumesToBake, TArray<FNavVolumeBakeIssue>& OutIssues) const;

	/**
	 * Returns true when the manifest has no payload for the volume or the volume's
	 * bake-relevant settings changed since the manifest was written.
	 */
	bool IsVolumeBakeStale(const AOctreeVoxelVolume* Volume, TArray<FText>* OutReasons = nullptr) const;

private:
	void BakeAndConnectVolumes_Internal(const TArray<AOctreeVoxelVolume*>& VolumesToProcess, UWorld* InWorld);
	void SaveBakedDataToManifest_Internal(
		const TArray<AOctreeVoxelVolume*>& AllRelevantVolumes,
		const TMap<AOctreeVoxelVolume*, TArray<FOctreeNode>>& BakedVolumeData,
		const TMap<AOctreeVoxelVolume*, TMap<int32, TArray<FNavPortalBakeLink>>>& PortalData,
		const TMap<AOctreeVoxelVolume*, FNavDistanceField>& DistanceFields,
		UWorld* InWorld) const;
	int32 CountRelevantBakeComponents(UWorld* InWorld, const AOctreeVoxelVolume* Volume) const;
	bool HasManifestPayloadForVolume(UWorld* InWorld, const AOctreeVoxelVolume* Volume) const;

#endif
};
