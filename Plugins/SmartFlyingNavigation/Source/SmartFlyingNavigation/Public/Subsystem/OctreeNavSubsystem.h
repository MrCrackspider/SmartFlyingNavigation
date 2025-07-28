// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "NavVolume/OctreeVoxelVolume.h"
#include "OctreeNavSubsystem.generated.h"

class UOctreeDataAsset;

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

	AOctreeVoxelVolume* GetVolumeChunkAt(const FVector& WorldLocation) const;
	AOctreeVoxelVolume* GetVolumeByDataAsset(const TSoftObjectPtr<UOctreeDataAsset>& DataAsset) const;

	UPROPERTY(BlueprintAssignable, Category = "Navigation")
	FOnVolumeRegistered OnVolumeRegistered;

	const TArray<TObjectPtr<AOctreeVoxelVolume>>& GetRegisteredVolumes() const { return RegisteredVolumeChunks; }

#if WITH_EDITOR
	AOctreeVoxelVolume* FindVolumeContainingPoint_BakeTime(const FVector& WorldLocation) const;

	void ClearAllRegisteredVolumes();
#endif

protected:
	// 월드에 존재하는 모든 Volume Chunk들을 관리
	// 빠른 조회를 위해 그리드나 다른 공간 분할 구조로 개선 가능
	UPROPERTY()
	TArray<TObjectPtr<AOctreeVoxelVolume>> RegisteredVolumeChunks;

	UPROPERTY()
	TMap<TSoftObjectPtr<UOctreeDataAsset>, TObjectPtr<AOctreeVoxelVolume>> VolumeDataAssetMap;
};