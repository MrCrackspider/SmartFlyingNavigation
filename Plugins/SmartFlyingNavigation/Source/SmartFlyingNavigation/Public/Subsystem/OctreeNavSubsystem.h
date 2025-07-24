// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "OctreeNavSubsystem.generated.h"

class AOctreeVoxelVolume;
/**
 * 
 */
UCLASS()
class SMARTFLYINGNAVIGATION_API UOctreeNavSubsystem : public UWorldSubsystem
{
	GENERATED_BODY()

public:
	void RegisterVolumeChunk(AOctreeVoxelVolume* Volume);

	void UnregisterVolumeChunk(AOctreeVoxelVolume* Volume);

	AOctreeVoxelVolume* GetVolumeChunkAt(const FVector& WorldLocation) const;

	// TO-DO: 계층적 길찾기 구현
	// int32 RequestHierarchicalPath(FPathfindingRequest& Request);

protected:
	// 월드에 존재하는 모든 Volume Chunk들을 관리
	// 빠른 조회를 위해 그리드나 다른 공간 분할 구조로 개선 가능
	UPROPERTY()
	TArray<TObjectPtr<AOctreeVoxelVolume>> RegisteredVolumeChunks;
};