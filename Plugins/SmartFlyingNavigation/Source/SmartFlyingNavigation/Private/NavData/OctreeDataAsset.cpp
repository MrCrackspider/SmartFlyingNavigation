// Copyright © 2025 TeamHamDev. All Rights Reserved.

#include "NavData/OctreeDataAsset.h"
#include "Misc/Compression.h"
#include "Serialization/MemoryReader.h"

const FName UOctreeDataAsset::OctreeAssetType = TEXT("OctreeData");

#if WITH_EDITOR
void UOctreeDataAsset::SetAndCompressBakedData(const TArray<FOctreeNode>& InNodePool, const FIntVector& InMapSize, int32 InMaxDepth)
{
	this->BakedVolumeSize = InMapSize;
	this->BakedMaxOctreeDepth = InMaxDepth;

	TArray<uint8> RawData;
	FMemoryWriter Ar(RawData, true);
	Ar << const_cast<TArray<FOctreeNode>&>(InNodePool);

	if (RawData.Num() == 0)
	{
		this->CompressedNodePool.Empty();
		this->UncompressedNodePoolSize = 0;
		return;
	}

	this->UncompressedNodePoolSize = RawData.Num();

	const FName CompressionFormat = FName(TEXT("Zlib"));
	const int32 MaxCompressedSize = FCompression::CompressMemoryBound(CompressionFormat, RawData.Num());

	this->CompressedNodePool.Empty();
	this->CompressedNodePool.SetNumUninitialized(MaxCompressedSize);

	int32 ActualCompressedSize = MaxCompressedSize;
	if (FCompression::CompressMemory(CompressionFormat, this->CompressedNodePool.GetData(), ActualCompressedSize, RawData.GetData(), RawData.Num()))
	{
		this->CompressedNodePool.SetNum(ActualCompressedSize, false);
	}
	else
	{
		this->CompressedNodePool.Empty();
		this->UncompressedNodePoolSize = 0;
		UE_LOG(LogTemp, Error, TEXT("UOctreeDataAsset: Failed to compress node pool data!"));
	}
}
#endif


void UOctreeDataAsset::DecompressAndInitialize()
{
	if (NodePool.Num() > 0 || CompressedNodePool.Num() == 0)
	{
		return;
	}

	TArray<uint8> RawData;
	RawData.SetNumUninitialized(this->UncompressedNodePoolSize);

	const FName CompressionFormat = FName(TEXT("Zlib"));
	const bool bDecompressionSuccess = FCompression::UncompressMemory(
		CompressionFormat,
		RawData.GetData(),
		this->UncompressedNodePoolSize,
		this->CompressedNodePool.GetData(),
		this->CompressedNodePool.Num()
	);

	if (bDecompressionSuccess)
	{
		FMemoryReader Ar(RawData, true);
		Ar << this->NodePool;
		CompressedNodePool.Empty();
	}
	else
	{
		this->NodePool.Empty();
		UE_LOG(LogTemp, Error, TEXT("UOctreeDataAsset: Failed to decompress node pool data!"));
	}
}

FPrimaryAssetId UOctreeDataAsset::GetPrimaryAssetId() const
{
	return FPrimaryAssetId(OctreeAssetType, GetFName());
}