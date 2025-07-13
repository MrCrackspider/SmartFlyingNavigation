// Fill out your copyright notice in the Description page of Project Settings.


#include "Pathfinder/StaticVoxelChunkData.h"
#include "Serialization/Archive.h"
const FName UStaticVoxelChunkData::VoxelChunkAssetType = TEXT("VoxelChunk");

// void operator<<(FStructuredArchive::FSlot Slot, FByteBulkData& BulkData)
// {
// 	// FStructuredArchive::FStream을 열어 바이너리 데이터를 처리합니다.
// 	FStructuredArchive::FStream Stream = Slot.EnterStream();
//     
// 	// 스트림의 전체 크기를 먼저 기록/로드합니다.
// 	int64 NumBytes = BulkData.GetBulkDataSize();
// 	Stream.EnterElement() << NumBytes;
//
// 	void* Data = nullptr;
// 	if (Stream.GetUnderlyingArchive().IsLoading())
// 	{
// 		// 로딩 시: 데이터를 저장할 공간을 확보합니다.
// 		BulkData.Lock(NumBytes);
// 		Data = BulkData.Realloc(NumBytes);
// 	}
// 	else
// 	{
// 		// 저장 시: 기존 데이터를 가져옵니다.
// 		Data = BulkData.Lock(LOCK_READ_ONLY);
// 	}
//
// 	if (Data)
// 	{
// 		// 실제 바이너리 데이터를 스트림에 씁니다.
// 		Stream.EnterElement().Serialize(Data, NumBytes);
// 	}
//
// 	// 작업이 끝나면 반드시 잠금을 해제합니다.
// 	BulkData.Unlock();
// }


FPrimaryAssetId UStaticVoxelChunkData::GetPrimaryAssetId() const
{
	FString AssetName = FString::Printf(TEXT("Chunk_%d_%d_%d"), ChunkCoords.X, ChunkCoords.Y, ChunkCoords.Z);
    
	// 미리 정의된 타입과 생성된 이름을 사용하여 PrimaryAssetId를 반환합니다.
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
