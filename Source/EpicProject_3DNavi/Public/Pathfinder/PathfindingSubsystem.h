// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "PathfindingSubsystem.generated.h"

class AVoxelVolumeBase;

/**
 * 
 */
UCLASS()
class EPICPROJECT_3DNAVI_API UPathfindingSubsystem : public UWorldSubsystem
{
	GENERATED_BODY()

public:
	virtual void Initialize(FSubsystemCollectionBase& Collection) override;
	virtual void Deinitialize() override;

	AVoxelVolumeBase* CreateVoxelVolume(const FIntVector& InMapSize, const uint32 Extent);
	void DestroyVoxelVolume(AVoxelVolumeBase* VolumeToDestroy);
	void ClearAllVolumes();

	bool BuildVoxelVolumes();

	void RenderVoxelVolume(const FLinearColor& WalkableColor, const FLinearColor& ObstacleColor) const;

	// Begin Test
	void CreateTestVolume();

	// End Test
private:
	UPROPERTY(BlueprintReadWrite, Category = "Pathfinding", meta = (AllowPrivateAccess))
	TArray<TObjectPtr<AVoxelVolumeBase>> Volumes;

public:
	const TArray<TObjectPtr<AVoxelVolumeBase>>& GetVoxelVolumes() const { return Volumes; }


};
