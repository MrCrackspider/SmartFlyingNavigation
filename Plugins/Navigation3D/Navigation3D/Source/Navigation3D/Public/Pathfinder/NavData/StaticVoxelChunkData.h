// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "Containers/BitArray.h"
#include "Serialization/BulkData.h"
#include "StaticVoxelChunkData.generated.h"


/**
 * A Data Asset that stores pre-computed static obstacle data for a single voxel chunk.
 * This asset is generated through the 'baking' process in the editor.
 */
USTRUCT(BlueprintType)
struct F128BitChunk
{
	GENERATED_BODY()

	UPROPERTY()
	uint64 Words[2] = {0, 0};

	F128BitChunk()
	{
		Words[0] = 0;
		Words[1] = 0;
	}
};

UCLASS()
class NAVIGATION3D_API UStaticVoxelChunkData : public UPrimaryDataAsset
{
	GENERATED_BODY()
public:
	static const FName VoxelChunkAssetType;
	
	// 청크당 128³ 비트 = 2,097,152비트 = 262,144바이트
	FByteBulkData ObstacleBitsBulk;

	UPROPERTY() // UPROPERTY를 붙여서 직렬화되도록 해야 합니다.
	FIntVector ChunkCoords; // int32 대신 FIntVector를 사용하는 것이 좋습니다.

	virtual FPrimaryAssetId GetPrimaryAssetId() const override;

	
	// 에디터 전용 함수임을 명시하는 것이 좋습니다.
	virtual void Serialize(FStructuredArchive::FRecord Record) override;
#if WITH_EDITOR
	// 에디터 베이킹 단계에서:
	void FillBulkData(const TArray<F128BitChunk>& ChunkArray)
	{
		// 1. Clear any pre-existing data.
		ObstacleBitsBulk.RemoveBulkData();
		ObstacleBitsBulk.Lock(LOCK_READ_WRITE);

		const int64 ByteCount = int64(ChunkArray.Num()) * sizeof(F128BitChunk);
		if (ByteCount <= 0)
		{
			UE_LOG(LogTemp, Warning,
				   TEXT("FillBulkData: ChunkArray.Num() == 0 또는 ByteCount <= 0 (ByteCount=%lld)"),
				   ByteCount);
			return;
		}

		
		if (ByteCount > 0)
		{
			// 2. Use Realloc() to allocate the required memory. 
			//    Realloc also locks the data and returns a valid pointer.
			void* BulkPtr = ObstacleBitsBulk.Realloc(ByteCount);
			if (!BulkPtr)
			{
				UE_LOG(LogTemp, Error,
					   TEXT("FillBulkData: Realloc 실패, 반환 포인터가 nullptr (ByteCount=%lld)"),
					   ByteCount);
				return;
			}
			// 3. Copy the source data into the newly allocated buffer.
			FMemory::Memcpy(BulkPtr, ChunkArray.GetData(), ByteCount);

			// 4. Unlock the data, as it was locked by Realloc().
			ObstacleBitsBulk.Unlock();
		}
	}
#endif // WITH_EDITOR
};