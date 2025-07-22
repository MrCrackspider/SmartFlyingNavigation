// Copyright © 2025 TeamHamDev. All Rights Reserved.

#include "NavData/OctreeDataAsset.h"
#include "Misc/Compression.h"
#include "Serialization/MemoryReader.h"

const FName UOctreeDataAsset::OctreeAssetType = TEXT("OctreeData");


#if WITH_EDITOR
void UOctreeDataAsset::SetAndCompressBakedData(const TArray<FOctreeNode>& InNodePool, const FIntVector& InMapSize, int32 InMaxDepth)
{
	// 1. Store all metadata first to ensure consistency.
	this->BakedMapSize = InMapSize;
	this->BakedMaxOctreeDepth = InMaxDepth;

	// 2. Serialize the node pool into a raw byte array.
	TArray<uint8> RawData;
	FMemoryWriter Ar(RawData, true);
	Ar << const_cast<TArray<FOctreeNode>&>(InNodePool); // FMemoryWriter requires non-const ref

	if (RawData.Num() == 0)
	{
		this->CompressedData.Empty();
		this->UncompressedSize = 0;
		return;
	}

	// 3. Store the original size for decompression.
	this->UncompressedSize = RawData.Num();

	// 4. Compress the raw data using Zlib.
	const FName CompressionFormat = FName(TEXT("Zlib"));
	const int32 MaxCompressedSize = FCompression::CompressMemoryBound(CompressionFormat, RawData.Num());

	this->CompressedData.Empty(); // Clear any previous data.
	this->CompressedData.SetNumUninitialized(MaxCompressedSize);

	int32 ActualCompressedSize = MaxCompressedSize;
	if (FCompression::CompressMemory(CompressionFormat, this->CompressedData.GetData(), ActualCompressedSize, RawData.GetData(), RawData.Num()))
	{
		// Shrink the array to the actual compressed size to save space.
		this->CompressedData.SetNum(ActualCompressedSize, false);
	}
	else
	{
		// On failure, clear all data to prevent corruption.
		this->CompressedData.Empty();
		this->UncompressedSize = 0;
		UE_LOG(LogTemp, Error, TEXT("UOctreeDataAsset: Failed to compress navigation data!"));
	}
}
#endif


void UOctreeDataAsset::DecompressAndInitialize()
{
    // --- Decompression logic starts here ---

    // 1. Pre-check: Do not run if the NodePool is already populated or if there's no data to decompress.

	if (NodePool.Num() > 0 || CompressedData.Num() == 0)
    {
        return;
    }

    // A temporary array to hold the decompressed raw byte data.
    TArray<uint8> RawData;
    RawData.SetNumUninitialized(this->UncompressedSize);

    // 3. Perform the actual decompression from CompressedData into the RawData buffer.
    const FName CompressionFormat = FName(TEXT("Zlib"));
    const bool bDecompressionSuccess = FCompression::UncompressMemory(
        CompressionFormat,
        RawData.GetData(),
        this->UncompressedSize,
        this->CompressedData.GetData(),
        this->CompressedData.Num()
    );

    if (bDecompressionSuccess)
    {
        // 4. If decompression was successful, deserialize the raw byte data back into the TArray<FOctreeNode>.
    	FMemoryReader Ar(RawData, true);
        Ar << this->NodePool;
    	CompressedData.Empty();
    }
    else
    {
        // If decompression fails, log an error and ensure the NodePool is empty to signify failure.
        this->NodePool.Empty();
        UE_LOG(LogTemp, Error, TEXT("UOctreeDataAsset: Failed to decompress navigation data!"));
    }
    // --- Decompression logic ends ---
}

FPrimaryAssetId UOctreeDataAsset::GetPrimaryAssetId() const
{
	return FPrimaryAssetId(OctreeAssetType, GetFName());
}
