// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "NavVolume/OctreeVoxelVolume.h"
#include "OctreeNavSubsystem.generated.h"

class UOctreeDataAsset;
DECLARE_DYNAMIC_MULTICAST_DELEGATE(FOnNavSubsystemReady);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnVolumeRegistered, AOctreeVoxelVolume*, RegisteredVolume);

/**
 * 
 */
UCLASS()
class SMARTFLYINGNAVIGATION_API UOctreeNavSubsystem : public UWorldSubsystem
{
	GENERATED_BODY()

public:

	virtual void Initialize(FSubsystemCollectionBase& Collection) override;
	void RegisterVolumeChunk(AOctreeVoxelVolume* Volume);
	void UnregisterVolumeChunk(AOctreeVoxelVolume* Volume);

	void UpdateVolumeDataAssetMapping(AOctreeVoxelVolume* Volume);

	AOctreeVoxelVolume* GetVolumeChunkAt(const FVector& WorldLocation) const;
	AOctreeVoxelVolume* GetVolumeByDataAsset(const TSoftObjectPtr<UOctreeDataAsset>& DataAsset) const;

	UPROPERTY(BlueprintAssignable, Category = "Navigation")
	FOnNavSubsystemReady OnSubsystemReady;

	UPROPERTY(BlueprintAssignable, Category = "Navigation")
	FOnVolumeRegistered OnVolumeRegistered;

	const TArray<TObjectPtr<AOctreeVoxelVolume>>& GetRegisteredVolumes() const { return RegisteredVolumeChunks; }

#if WITH_EDITOR
	AOctreeVoxelVolume* FindVolumeContainingPoint_BakeTime(const FVector& WorldLocation) const;

	void ClearAllRegisteredVolumes();
#endif

protected:
	UPROPERTY()
	TArray<TObjectPtr<AOctreeVoxelVolume>> RegisteredVolumeChunks;

	UPROPERTY()
	TMap<TSoftObjectPtr<UOctreeDataAsset>, TObjectPtr<AOctreeVoxelVolume>> VolumeDataAssetMap;

private:
	bool bIsReady = false;
public:
	bool IsReady() const { return bIsReady; }
};