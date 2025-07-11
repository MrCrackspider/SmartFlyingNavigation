#include "Pathfinder/ChunkDataHolder.h"

#include "EngineUtils.h"
#include "ThreadSafeBitmask.h"
#include "Pathfinder/FenwickVoxelVolume.h"


AChunkDataHolder::AChunkDataHolder()
{
	PrimaryActorTick.bCanEverTick = false;
	PrimaryActorTick.bStartWithTickEnabled = false;
	if (UActorComponent* RootComp = GetRootComponent())
	{
		RootComp->PrimaryComponentTick.bCanEverTick = false;
		RootComp->PrimaryComponentTick.bStartWithTickEnabled = false;
		RootComp->SetComponentTickEnabled(false);
	}
}

void AChunkDataHolder::BeginPlay()
{
	Super::BeginPlay();

	// OwningVolume이 유효하지 않은 경우에만 찾기를 시도합니다.
	if (!OwningVolume.IsValid())
	{
		// 현재 월드에 있는 모든 AFenwickVoxelVolume 액터를 순회합니다.
		for (TActorIterator<AFenwickVoxelVolume> It(GetWorld()); It; ++It)
		{
			AFenwickVoxelVolume* FoundVolume = *It;
			if (FoundVolume)
			{
				const FIntVector VolumeMapSizeInChunks(
					FoundVolume->MapSize.X / FThreadSafeBitmaskGrid::CHUNK_SIZE,
					FoundVolume->MapSize.Y / FThreadSafeBitmaskGrid::CHUNK_SIZE,
					FoundVolume->MapSize.Z / FThreadSafeBitmaskGrid::CHUNK_SIZE
				);

				if (ChunkCoords.X >= 0 && ChunkCoords.X < VolumeMapSizeInChunks.X &&
					ChunkCoords.Y >= 0 && ChunkCoords.Y < VolumeMapSizeInChunks.Y &&
					ChunkCoords.Z >= 0 && ChunkCoords.Z < VolumeMapSizeInChunks.Z)
				{
					// 이 볼륨이 나를 관리하는 볼륨이다!
					OwningVolume = FoundVolume;
					OwningVolume->RegisterChunkFromLevel(this);
					break; // 찾았으므로 루프 종료
				}
			}
		}
	}
}

void AChunkDataHolder::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (OwningVolume.IsValid())
	{
		OwningVolume->UnregisterChunkFromLevel(this);
	}
	Super::EndPlay(EndPlayReason);
}