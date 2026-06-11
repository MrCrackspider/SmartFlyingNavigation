// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "NavCore/DynamicObstacleRegistry.h"
#include "OctreeNavSubsystem.generated.h"

class AOctreeVoxelVolume;
class UWorldNavigationDataAsset;
DECLARE_DYNAMIC_MULTICAST_DELEGATE(FOnNavSubsystemReady);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnVolumeRegistered, AOctreeVoxelVolume*, RegisteredVolume);

/**
 * World registry of navigation volume chunks. Maps chunk ids from the world
 * navigation manifest to streamed-in volume actors and initializes their payloads.
 */
UCLASS()
class SMARTFLYINGNAVIGATION_API UOctreeNavSubsystem : public UWorldSubsystem
{
	GENERATED_BODY()

public:
	virtual void Initialize(FSubsystemCollectionBase& Collection) override;
	void RegisterVolumeChunk(AOctreeVoxelVolume* Volume);
	void UnregisterVolumeChunk(AOctreeVoxelVolume* Volume);

	void SetWorldNavigationData(UWorldNavigationDataAsset* InWorldNavigationData);

	AOctreeVoxelVolume* GetVolumeChunkAt(const FVector& WorldLocation) const;
	AOctreeVoxelVolume* GetVolumeByChunkId(int32 ChunkId) const;
	bool InitializeVolumeFromManifestPayload(AOctreeVoxelVolume* Volume);

	UPROPERTY(BlueprintAssignable, Category = "Navigation")
	FOnNavSubsystemReady OnSubsystemReady;

	UPROPERTY(BlueprintAssignable, Category = "Navigation")
	FOnVolumeRegistered OnVolumeRegistered;

	const TArray<TObjectPtr<AOctreeVoxelVolume>>& GetRegisteredVolumes() const { return RegisteredVolumeChunks; }

	/** World-level dynamic obstacle registry: registered once, consumed by A* and ORCA. */
	FDynamicObstacleRegistry& GetObstacleRegistry() { return ObstacleRegistry; }
	const FDynamicObstacleRegistry& GetObstacleRegistry() const { return ObstacleRegistry; }

#if WITH_EDITOR
	AOctreeVoxelVolume* FindVolumeContainingPoint_BakeTime(const FVector& WorldLocation) const;

	void ClearAllRegisteredVolumes();
#endif

protected:
	UPROPERTY()
	TArray<TObjectPtr<AOctreeVoxelVolume>> RegisteredVolumeChunks;

	UPROPERTY()
	TObjectPtr<UWorldNavigationDataAsset> CurrentWorldNavigationData;

	UPROPERTY()
	TMap<int32, TObjectPtr<AOctreeVoxelVolume>> ChunkIdToVolumeMap;

	int32 FindChunkIdForVolume(const AOctreeVoxelVolume* Volume) const;

	FDynamicObstacleRegistry ObstacleRegistry;

private:
	bool bIsReady = false;
public:
	bool IsReady() const { return bIsReady; }
};
