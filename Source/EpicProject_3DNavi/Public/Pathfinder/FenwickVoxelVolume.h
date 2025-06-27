// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Pathfinder/VoxelVolume.h"
#include "FenwickVoxelVolume.generated.h"

UCLASS()
class EPICPROJECT_3DNAVI_API AFenwickVoxelVolume : public AVoxelVolume
{
	GENERATED_BODY()

public:
	// Sets default values for this actor's properties
	AFenwickVoxelVolume();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
public:
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	virtual void BuildStaticObstacles() override;
	virtual void UpdateDynamicObstacles() override;
	bool IsRegionWalkable(const FIntVector& VoxelIndex, int32 AgentRadiusInVoxels);
	bool IsPathSegmentWalkable(const FIntVector& FromStartIndex, const FIntVector& ToStartIndex, int32 AgentRadiusInVoxels) const;
	
	void FenwUpdate(const FIntVector& Index, int32 Delta);
	int32 FenwQuery(const FIntVector& Index) const;
	int32 FenwRangeSum(const FIntVector& Min, const FIntVector& Max) const;
	FORCEINLINE int32 GetFenwickFlatIndex(int32 x, int32 y, int32 z) const;
private:
	FIntVector FenwickTreeSize;
	TArray<int32> FenwickTree;
	bool bIsObstacleUpdateInProgress;
	
};
