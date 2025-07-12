// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "Containers/BitArray.h"
#include "StaticVoxelChunkData.generated.h"

/**
 * A Data Asset that stores pre-computed static obstacle data for a single voxel chunk.
 * This asset is generated through the 'baking' process in the editor.
 */
UCLASS(BlueprintType)
class NAVIGATION3D_API UStaticVoxelChunkData : public UDataAsset
{
	GENERATED_BODY()

public:
	/** The coordinates of the chunk this data belongs to. */
	UPROPERTY(VisibleAnywhere, Category = "Voxel Chunk Data")
	FIntVector ChunkCoords;
	
	UPROPERTY(VisibleAnywhere, Category = "Voxel Chunk Data")
	TArray<int32> SummedAreaTable;

	TBitArray<> ObstacleBitmasks_X;

	virtual void Serialize(FStructuredArchive::FRecord Record) override;
};
