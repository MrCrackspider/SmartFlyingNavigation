// Copyright © 2025 TeamHamDev. All Rights Reserved.

#include "NavData/StaticVoxelChunkData.h"
#include "Serialization/Archive.h"
const FName UStaticVoxelChunkData::VoxelChunkAssetType = TEXT("VoxelChunk");

FPrimaryAssetId UStaticVoxelChunkData::GetPrimaryAssetId() const
{
	FString AssetName = FString::Printf(TEXT("Chunk_%d_%d_%d"), ChunkCoords.X, ChunkCoords.Y, ChunkCoords.Z);
    
	return FPrimaryAssetId(UStaticVoxelChunkData::VoxelChunkAssetType, FName(*AssetName));
}

void UStaticVoxelChunkData::Serialize(FStructuredArchiveRecord Record)
{
	// First, let the parent class serialize its properties (like ChunkCoords).
	Super::Serialize(Record);

	// Get the underlying archive from the structured archive record.
	FArchive& Ar = Record.GetUnderlyingArchive();

	// Now, manually serialize our bulk data.
	// Because ObstacleBitsBulk is NOT a UPROPERTY, this is the only
	// place it will be serialized.
	// Setting bAttemptFileMapping to false is the key to fixing the crash.
	ObstacleBitsBulk.Serialize(Ar, this, /*bAttemptFileMapping=*/false);
}
