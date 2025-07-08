// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "StaticVoxelChunkData.generated.h"

/**
 * A Data Asset that stores pre-computed static obstacle data for a single voxel chunk.
 * This asset is generated through the 'baking' process in the editor.
 */
UCLASS(BlueprintType)
class EPICPROJECT_3DNAVI_API UStaticVoxelChunkData : public UDataAsset
{
	GENERATED_BODY()

public:
	/** The coordinates of the chunk this data belongs to. */
	UPROPERTY(VisibleAnywhere, Category = "Voxel Chunk Data")
	FIntVector ChunkCoords;

	/** A bitmask representing the locations of static obstacles within this chunk. */
	UPROPERTY(VisibleAnywhere, Category = "Voxel Chunk Data")
	TArray<uint64> ObstacleBitmasks_X;
	
	/** The Summed-Area Table (SAT) for O(1) queries of obstacle counts within this chunk.  */
	UPROPERTY(VisibleAnywhere, Category = "Voxel Chunk Data")
	TArray<int32> SummedAreaTable;
};
