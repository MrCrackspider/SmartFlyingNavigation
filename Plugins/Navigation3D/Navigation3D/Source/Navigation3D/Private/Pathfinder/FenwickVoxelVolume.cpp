#include "Pathfinder/FenwickVoxelVolume.h"
#include "Pathfinder/StaticVoxelChunkData.h"
#include "Pathfinder/ChunkDataHolder.h"
#include "Pathfinder/PathfindingComponent.h"

#include "Engine/LevelStreamingDynamic.h"
#include "HAL/PlatformMemory.h"
#include "Math/UnrealMathUtility.h"

// SIMD 내장 함수를 사용하기 위한 헤더
#include "emmintrin.h" // SSE2
#include "smmintrin.h" // SSE4.1
#include "AssetRegistry/AssetRegistryModule.h"
#include "Engine/AssetManager.h"
#include "Engine/StreamableManager.h"

#if WITH_EDITOR
#include "ObjectTools.h"
#include "AssetToolsModule.h"
#include "FileHelpers.h"
#include "Misc/ScopedSlowTask.h"
#include "Misc/MessageDialog.h"
#include "Editor/EditorEngine.h" // UEditorLoadingAndSavingUtils
#endif

//======================================================================
// --- INITIALIZATION & LIFECYCLE ---
//======================================================================

AFenwickVoxelVolume::AFenwickVoxelVolume()
{
	PrimaryActorTick.bCanEverTick = true;
	bIsReady = false;
}

void AFenwickVoxelVolume::BeginPlay()
{
	Super::BeginPlay();
	InitializeDataStructures();
	
	GetWorldTimerManager().SetTimer(StreamingUpdateTimer, this, &AFenwickVoxelVolume::UpdateStreamingState, 0.5f, true, 0.f);
	GetWorldTimerManager().SetTimer(DynamicObstacleTimer, this, &AFenwickVoxelVolume::UpdateDynamicObstacles, 0.2f, true, 0.1f);

	UE_LOG(LogTemp, Log, TEXT("Voxel Volume streaming system initialized."));
	bIsReady = true;
}

void AFenwickVoxelVolume::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (GetWorld())
	{
		GetWorld()->GetTimerManager().ClearAllTimersForObject(this);
	}
	Super::EndPlay(EndPlayReason);
}

void AFenwickVoxelVolume::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

void AFenwickVoxelVolume::BuildObstacles()
{
	Super::BuildObstacles();
}

void AFenwickVoxelVolume::InitializeDataStructures()
{
	TRACE_CPUPROFILER_EVENT_SCOPE(VoxelVolume_InitializeDataStructures);
	
	check(MapSize.X > 0 && MapSize.X % 128 == 0);
	check(MapSize.Y > 0);
	check(MapSize.Z > 0);
	check(CHUNK_SIZE == 128); // SIMD 처리를 위해 청크 크기는 128로 고정

	const int32 NumChunks128 = (MapSize.X / 128) * MapSize.Y * MapSize.Z;
	
	StaticObstacleBitmask.Empty(NumChunks128);
	StaticObstacleBitmask.AddZeroed(NumChunks128);

	DynamicObstacleBitmask.Empty(NumChunks128);
	DynamicObstacleBitmask.AddZeroed(NumChunks128);
}

//======================================================================
// --- PUBLIC QUERY API IMPLEMENTATION ---
//======================================================================

bool AFenwickVoxelVolume::IsRegionWalkable(const FIntVector& VoxelIndex, int32 AgentRadiusInVoxels) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE(VoxelVolume_IsRegionWalkable);
	if (!IsReadyForPathfinding() || AgentRadiusInVoxels < 0) return false;

	const FIntVector AgentRadius(AgentRadiusInVoxels);
	const FIntVector MinCorner = VoxelIndex - AgentRadius;
	const FIntVector MaxCorner = VoxelIndex + AgentRadius;

	return IsRegionEmpty_Bitmask(MinCorner, MaxCorner);
}

bool AFenwickVoxelVolume::IsPathSegmentWalkable(const FIntVector& FromStartIndex, const FIntVector& ToStartIndex, int32 AgentRadiusInVoxels) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE(VoxelVolume_IsPathSegmentWalkable);
	if (!IsReadyForPathfinding()) return false;

	const FIntVector AgentRadius(AgentRadiusInVoxels);
	const FIntVector FromMin = FromStartIndex - AgentRadius;
	const FIntVector FromMax = FromStartIndex + AgentRadius;
	const FIntVector ToMin = ToStartIndex - AgentRadius;
	const FIntVector ToMax = ToStartIndex + AgentRadius;

	const FIntVector SweptMin(FMath::Min(FromMin.X, ToMin.X), FMath::Min(FromMin.Y, ToMin.Y), FMath::Min(FromMin.Z, ToMin.Z));
	const FIntVector SweptMax(FMath::Max(FromMax.X, ToMax.X), FMath::Max(FromMax.Y, ToMax.Y), FMath::Max(FromMax.Z, ToMax.Z));

	return IsRegionEmpty_Bitmask(SweptMin, SweptMax);
}

bool AFenwickVoxelVolume::IsVoxelWalkable(const FIntVector& Index) const
{
	if (!IsVoxelIndexValid(Index)) return false;

	const int32 ChunksPerRow = MapSize.X / 128;
	const int32 ChunkIndex = (Index.Z * MapSize.Y + Index.Y) * ChunksPerRow + (Index.X / 128);
	const int32 BitIndex = Index.X % 128;

	const F128BitChunk& StaticChunk = StaticObstacleBitmask[ChunkIndex];
	const F128BitChunk& DynamicChunk = DynamicObstacleBitmask[ChunkIndex];

	const uint64 Word = (BitIndex < 64) ? (StaticChunk.Words[0] | DynamicChunk.Words[0]) : (StaticChunk.Words[1] | DynamicChunk.Words[1]);
	const uint64 Mask = 1ULL << (BitIndex % 64);
	
	return (Word & Mask) == 0;
}

void AFenwickVoxelVolume::GetDebugObstacleVoxelIndices(TArray<FIntVector>& OutObstacleIndices) const
{
	OutObstacleIndices.Empty();
	if (!IsReadyForPathfinding()) return;

	const int32 ChunksPerRow = MapSize.X / 128;

	for (int32 z = 0; z < MapSize.Z; ++z)
	{
		for (int32 y = 0; y < MapSize.Y; ++y)
		{
			for (int32 chunkX = 0; chunkX < ChunksPerRow; ++chunkX)
			{
				const int32 ChunkIndex = (z * MapSize.Y + y) * ChunksPerRow + chunkX;
				const F128BitChunk& StaticChunk = StaticObstacleBitmask[ChunkIndex];
				const F128BitChunk& DynamicChunk = DynamicObstacleBitmask[ChunkIndex];
				
				const __m128i Combined = _mm_or_si128(
					_mm_load_si128(reinterpret_cast<const __m128i*>(&StaticChunk)),
					_mm_load_si128(reinterpret_cast<const __m128i*>(&DynamicChunk))
				);

				if (_mm_testz_si128(Combined, Combined) == 0) // If not all zero
				{
					F128BitChunk Temp;
					_mm_store_si128(reinterpret_cast<__m128i*>(&Temp), Combined);
					
					for (int i = 0; i < 2; ++i)
					{
						if (Temp.Words[i] != 0)
						{
							for (int bit = 0; bit < 64; ++bit)
							{
								if ((Temp.Words[i] >> bit) & 1)
								{
									OutObstacleIndices.Emplace(chunkX * 128 + i * 64 + bit, y, z);
								}
							}
						}
					}
				}
			}
		}
	}
}

bool AFenwickVoxelVolume::IsRegionEmpty_Bitmask(const FIntVector& MinCorner, const FIntVector& MaxCorner) const
{
    const FIntVector ClampedMin(
        FMath::Max(MinCorner.X, 0), FMath::Max(MinCorner.Y, 0), FMath::Max(MinCorner.Z, 0)
    );
    const FIntVector ClampedMax(
        FMath::Min(MaxCorner.X, MapSize.X - 1), FMath::Min(MaxCorner.Y, MapSize.Y - 1), FMath::Min(MaxCorner.Z, MapSize.Z - 1)
    );

    if (ClampedMin.X > ClampedMax.X || ClampedMin.Y > ClampedMax.Y || ClampedMin.Z > ClampedMax.Z)
    {
        return true;
    }

    const int32 ChunksPerRow = MapSize.X / 128;

    auto CreateMask = [](int32 StartBit, int32 EndBit) -> __m128i
    {
        uint64 LowMask = (StartBit < 64) ? (~0ULL << StartBit) : 0;
        if (EndBit < 64) LowMask &= (~0ULL >> (63 - EndBit));
        
        uint64 HighMask = (EndBit >= 64) ? (~0ULL >> (127 - EndBit)) : 0;
        if (StartBit >= 64) HighMask &= (~0ULL << (StartBit - 64));
        
        return _mm_set_epi64x(HighMask, LowMask);
    };

    for (int32 z = ClampedMin.Z; z <= ClampedMax.Z; ++z)
    {
        for (int32 y = ClampedMin.Y; y <= ClampedMax.Y; ++y)
        {
            const int32 RowStartChunkIndex = (z * MapSize.Y + y) * ChunksPerRow;
            
            const int32 StartX = ClampedMin.X;
            const int32 EndX = ClampedMax.X;

            const int32 StartChunkIndex = RowStartChunkIndex + StartX / 128;
            const int32 EndChunkIndex = RowStartChunkIndex + EndX / 128;
            
            const int32 StartBitInChunk = StartX % 128;
            const int32 EndBitInChunk = EndX % 128;

            if (StartChunkIndex == EndChunkIndex)
            {
                const __m128i Mask = CreateMask(StartBitInChunk, EndBitInChunk);
                const __m128i StaticData = _mm_load_si128(reinterpret_cast<const __m128i*>(&StaticObstacleBitmask[StartChunkIndex]));
                const __m128i DynamicData = _mm_load_si128(reinterpret_cast<const __m128i*>(&DynamicObstacleBitmask[StartChunkIndex]));
                const __m128i CombinedData = _mm_or_si128(StaticData, DynamicData);
                if (_mm_testz_si128(CombinedData, Mask) == 0) return false;
            }
            else
            {
                __m128i StartMask = CreateMask(StartBitInChunk, 127);
                __m128i StaticData = _mm_load_si128(reinterpret_cast<const __m128i*>(&StaticObstacleBitmask[StartChunkIndex]));
                __m128i DynamicData = _mm_load_si128(reinterpret_cast<const __m128i*>(&DynamicObstacleBitmask[StartChunkIndex]));
                if (_mm_testz_si128(_mm_or_si128(StaticData, DynamicData), StartMask) == 0) return false;

                for (int32 i = StartChunkIndex + 1; i < EndChunkIndex; ++i)
                {
                    const __m128i MiddleStaticData = _mm_load_si128(reinterpret_cast<const __m128i*>(&StaticObstacleBitmask[i]));
                    const __m128i MiddleDynamicData = _mm_load_si128(reinterpret_cast<const __m128i*>(&DynamicObstacleBitmask[i]));
                    const __m128i CombinedData = _mm_or_si128(MiddleStaticData, MiddleDynamicData);
                    if (_mm_testz_si128(CombinedData, CombinedData) == 0) return false;
                }

                __m128i EndMask = CreateMask(0, EndBitInChunk);
                StaticData = _mm_load_si128(reinterpret_cast<const __m128i*>(&StaticObstacleBitmask[EndChunkIndex]));
                DynamicData = _mm_load_si128(reinterpret_cast<const __m128i*>(&DynamicObstacleBitmask[EndChunkIndex]));
                if (_mm_testz_si128(_mm_or_si128(StaticData, DynamicData), EndMask) == 0) return false;
            }
        }
    }

    return true;
}

//======================================================================
// --- AGENT REGISTRATION & STREAMING ---
//======================================================================

void AFenwickVoxelVolume::RegisterStreamingAgent(AActor* Agent)
{
	if (Agent) StreamingAgents.Add(Agent);
}

void AFenwickVoxelVolume::UnregisterStreamingAgent(AActor* Agent)
{
	if (Agent) StreamingAgents.Remove(Agent);
}

void AFenwickVoxelVolume::UpdateStreamingState()
{
	TSet<FIntVector> NewRequiredChunks;
	const int32 NumChunksX = MapSize.X / CHUNK_SIZE;
	const int32 NumChunksY = MapSize.Y / CHUNK_SIZE;
	const int32 NumChunksZ = MapSize.Z / CHUNK_SIZE;

	for (auto It = StreamingAgents.CreateIterator(); It; ++It)
	{
		if (AActor* Agent = It->Get())
		{
			const FIntVector AgentChunkCoords = WorldToChunkCoords(Agent->GetActorLocation());
			for (int32 z = -StreamingRangeInChunks.Z; z <= StreamingRangeInChunks.Z; ++z)
			{
				for (int32 y = -StreamingRangeInChunks.Y; y <= StreamingRangeInChunks.Y; ++y)
				{
					for (int32 x = -StreamingRangeInChunks.X; x <= StreamingRangeInChunks.X; ++x)
					{
						const FIntVector Coords = AgentChunkCoords + FIntVector(x, y, z);
						if (Coords.X >= 0 && Coords.X < NumChunksX &&
							Coords.Y >= 0 && Coords.Y < NumChunksY &&
							Coords.Z >= 0 && Coords.Z < NumChunksZ)
						{
							NewRequiredChunks.Add(Coords);
						}
					}
				}
			}
		}
		else
		{
			It.RemoveCurrent();
		}
	}

	const TSet<FIntVector> ChunksToLoad = NewRequiredChunks.Difference(RequiredChunks);
	const TSet<FIntVector> ChunksToUnload = RequiredChunks.Difference(NewRequiredChunks);

	for (const FIntVector& Coords : ChunksToLoad)
	{
		RequestChunkLoad(Coords);
	}
	for (const FIntVector& Coords : ChunksToUnload)
	{
		UnloadChunk(Coords);
	}

	RequiredChunks = NewRequiredChunks;
}

void AFenwickVoxelVolume::RequestChunkLoad(const FIntVector& ChunkCoords)
{
	// 1) 중복 요청 방지
	{
		FReadScopeLock ReadLock(VolumeDataLock);
		if (PendingChunkHandles.Contains(ChunkCoords) || ActiveChunks.Contains(ChunkCoords))
		{
			return;
		}
	}

	// 2) FPrimaryAssetId 생성
	const FString AssetName = FString::Printf(TEXT("Chunk_%d_%d_%d"), ChunkCoords.X, ChunkCoords.Y, ChunkCoords.Z);

	const FString Dir = GetFinalSaveDirectory();
	//하드코딩 수정하기.
	const FString PackagePath = FString::Printf(TEXT("%s/Data/%s.%s"), *Dir , *AssetName, *AssetName);
	GEngine->AddOnScreenDebugMessage(-1, 10, FColor::Red,  FString::Printf(TEXT("%s"), *PackagePath));
	
	const FPrimaryAssetId AssetId(UStaticVoxelChunkData::StaticClass()->GetFName(), FName(*AssetName));
	FSoftObjectPath SoftPath(PackagePath);
	FStreamableManager& Streamable = UAssetManager::GetStreamableManager();

	TSharedPtr<FStreamableHandle> Handle = Streamable.RequestAsyncLoad(
	   SoftPath,
	   FStreamableDelegate::CreateLambda([this, SoftPath, ChunkCoords]()
	   {
	   	// 4) Pending에서 핸들 제거
			TSharedPtr<FStreamableHandle> CompletedHandle;
			{
				FWriteScopeLock WriteLock(VolumeDataLock);
				PendingChunkHandles.RemoveAndCopyValue(ChunkCoords, CompletedHandle);
			}
	   	
		   UObject* Loaded = SoftPath.ResolveObject();
		   if (UStaticVoxelChunkData* LoadedData = Cast<UStaticVoxelChunkData>(Loaded))
		   {
			   RegisterChunk(LoadedData);
			   FWriteScopeLock WriteLock(VolumeDataLock);
			   ActiveChunks.Add(ChunkCoords, LoadedData);
		   }
	   })
   );

	// // 3) 비동기 로드 요청
	// //UAssetManager& Manager = UAssetManager::Get();
	// TSharedPtr<FStreamableHandle> Handle = Manager.LoadPrimaryAsset(
	// 	{ AssetId },                 // 로드할 에셋 ID 리스트
	// 	{}, 
	// 	FStreamableDelegate::CreateLambda([this, AssetId, ChunkCoords]()
	// 	{
	// 		// 4) Pending에서 핸들 제거
	// 		TSharedPtr<FStreamableHandle> CompletedHandle;
	// 		{
	// 			FWriteScopeLock WriteLock(VolumeDataLock);
	// 			PendingChunkHandles.RemoveAndCopyValue(ChunkCoords, CompletedHandle);
	// 		}
	//
	// 		// 5) 로드 완료 후 처리
	// 		UObject* LoadedObject = UAssetManager::Get().GetPrimaryAssetObject(AssetId);
	// 		if (UStaticVoxelChunkData* LoadedData = Cast<UStaticVoxelChunkData>(LoadedObject))
	// 		{
	// 			RegisterChunk(LoadedData);
	// 			FWriteScopeLock WriteLock(VolumeDataLock);
	// 			ActiveChunks.Add(ChunkCoords, LoadedData);
	// 		}
	// 		
	// 	})
	// );

	// 6) Pending에 핸들 등록
	if (Handle.IsValid())
	{
		FWriteScopeLock WriteLock(VolumeDataLock);
		PendingChunkHandles.Add(ChunkCoords, Handle);
	}
}

void AFenwickVoxelVolume::OnChunkLoadCompleted(FIntVector ChunkCoords, FStreamableHandle* StreamableHandle)
{
	// PendingChunkHandles.Remove(ChunkCoords);
	//
	// if (StreamableHandle && StreamableHandle->HasLoadCompleted())
	// {
	// 	UStaticVoxelChunkData* LoadedData = Cast<UStaticVoxelChunkData>(StreamableHandle->GetLoadedAsset());
	// 	if (LoadedData)
	// 	{
	// 		RegisterChunk(LoadedData);
	// 		ActiveChunks.Add(ChunkCoords, LoadedData);
	// 	}
	// }
}

void AFenwickVoxelVolume::UnloadChunk(const FIntVector& ChunkCoords)
{
	const FString AssetName = FString::Printf(TEXT("Chunk_%d_%d_%d"), ChunkCoords.X, ChunkCoords.Y, ChunkCoords.Z);
	const FPrimaryAssetId AssetIdToUnload(UStaticVoxelChunkData::VoxelChunkAssetType, FName(*AssetName));

	UAssetManager& AssetManager = UAssetManager::Get();
	
	AssetManager.UnloadPrimaryAsset(AssetIdToUnload);


	// 2. 진행 중인 로드가 있었다면 취소하고, 우리 내부 관리 목록에서 제거합니다.
	//    이 과정은 스레드 안전해야 합니다.
	FWriteScopeLock WriteLock(VolumeDataLock);

	// 1. FindAndRemove를 사용하여 TSharedPtr를 가져오는 동시에 맵에서 제거합니다.
	// 이 방법은 '찾아서 제거한다'는 의도를 명확히 하고, 단 한 번의 맵 조회로 끝납니다.
	TSharedPtr<FStreamableHandle> HandleToCancel;
	if (PendingChunkHandles.RemoveAndCopyValue(ChunkCoords, HandleToCancel))
	{
		// 2. 키가 존재해서 성공적으로 제거하고 값을 복사해왔다면, 핸들을 취소합니다.
		if (HandleToCancel.IsValid())
		{
			HandleToCancel->CancelHandle();
			UE_LOG(LogTemp, Log, TEXT("Cancelled pending load for chunk %s."), *ChunkCoords.ToString());
		}
	}

	// 3. 이미 활성화된 청크라면, 우리 비트마스크에서 데이터를 지우고 목록에서 제거합니다.
	if (ActiveChunks.Contains(ChunkCoords))
	{
		UnregisterChunk(ChunkCoords);
		ActiveChunks.Remove(ChunkCoords);
	}
}

void AFenwickVoxelVolume::RegisterChunk(UStaticVoxelChunkData* ChunkData)
{
		if (!ChunkData || ChunkData->ObstacleBitsBulk.GetBulkDataSize() == 0)
	{
		return;
	}

	// StaticObstacleBitmask에 쓰기 작업을 하므로, 쓰기 잠금을 겁니다.
	FWriteScopeLock WriteLock(VolumeDataLock);

	const FIntVector& ChunkCoords = ChunkData->ChunkCoords;
	const FIntVector ChunkOrigin = ChunkCoords * CHUNK_SIZE;
	const int32 ChunksPerRow = MapSize.X / 128;
	const int32 ChunksPerAssetRow = CHUNK_SIZE / 128;
	const int64 BytesToCopyPerRow = ChunksPerAssetRow * sizeof(F128BitChunk);
	const int64 BulkSize = ChunkData->ObstacleBitsBulk.GetBulkDataSize();

	// 1. 벌크 데이터를 읽기 전용으로 잠그고 포인터를 얻습니다.
	const void* SrcBulkPtr = ChunkData->ObstacleBitsBulk.LockReadOnly();
	if (!SrcBulkPtr)
	{
		UE_LOG(LogTemp, Error, TEXT("Failed to lock BulkData for chunk %s"), *ChunkCoords.ToString());
		return;
	}

	// 2. 청크의 각 Y, Z 슬라이스를 순회하며 한 줄씩 복사합니다.
	for (int32 z = 0; z < CHUNK_SIZE; ++z)
	{
		for (int32 y = 0; y < CHUNK_SIZE; ++y)
		{
			// 소스(벌크 데이터)에서의 바이트 오프셋
			const int64 SrcByteOffset = ((int64)z * CHUNK_SIZE + y) * BytesToCopyPerRow;
			
			// 목적지(StaticObstacleBitmask)에서의 128비트 청크 인덱스
			const int32 DestIndex = ((ChunkOrigin.Z + z) * MapSize.Y + (ChunkOrigin.Y + y)) * ChunksPerRow + (ChunkOrigin.X / 128);

			// 경계 및 유효성 검사
			if (StaticObstacleBitmask.IsValidIndex(DestIndex) && (SrcByteOffset + BytesToCopyPerRow) <= BulkSize)
			{
				// 목적지 메모리 주소
				void* DestPtr = &StaticObstacleBitmask[DestIndex];
				// 소스 메모리 주소 (오프셋 적용)
				const void* SrcPtr = static_cast<const uint8*>(SrcBulkPtr) + SrcByteOffset;
				
				FMemory::Memcpy(DestPtr, SrcPtr, BytesToCopyPerRow);
			}
		}
	}

	// 3. 데이터 사용이 끝났으므로 반드시 잠금을 해제합니다.
	ChunkData->ObstacleBitsBulk.Unlock();
	
	UE_LOG(LogTemp, Log, TEXT("Chunk %s registered from BulkData."), *ChunkCoords.ToString());
}

void AFenwickVoxelVolume::UnregisterChunk(const FIntVector& ChunkCoords)
{
	const FIntVector ChunkOrigin = ChunkCoords * CHUNK_SIZE;
	const int32 ChunksPerRow = MapSize.X / 128;
	const int32 ChunksPerAssetRow = CHUNK_SIZE / 128;

	for (int32 z = 0; z < CHUNK_SIZE; ++z)
	{
		for (int32 y = 0; y < CHUNK_SIZE; ++y)
		{
			const int32 DestIndex = ((ChunkOrigin.Z + z) * MapSize.Y + (ChunkOrigin.Y + y)) * ChunksPerRow + (ChunkOrigin.X / 128);
			if (StaticObstacleBitmask.IsValidIndex(DestIndex))
			{
				FMemory::Memset(&StaticObstacleBitmask[DestIndex], 0, ChunksPerAssetRow * sizeof(F128BitChunk));
			}
		}
	}
	UE_LOG(LogTemp, Log, TEXT("Chunk %s unregistered."), *ChunkCoords.ToString());
}

//======================================================================
// --- DYNAMIC OBSTACLE HANDLING ---
//======================================================================

void AFenwickVoxelVolume::UpdateDynamicObstacles()
{
	TRACE_CPUPROFILER_EVENT_SCOPE(VoxelVolume_UpdateDynamicObstacles);
	if (!IsReadyForPathfinding() || !GetWorld() || !ObstacleFilterProfile) return;

	TSet<FIntVector> NewFrameObstacles;
	GatherCurrentObstacleVoxels(NewFrameObstacles);
	ApplyDynamicObstacleChanges(NewFrameObstacles);

	CurrentDynamicObstacles = MoveTemp(NewFrameObstacles);
}

void AFenwickVoxelVolume::GatherCurrentObstacleVoxels(TSet<FIntVector>& OutVoxelIndices) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE(VoxelVolume_GatherObstacles);
	if (!IsValid(ObstacleFilterProfile)) return;

	FCollisionObjectQueryParams ObjectQueryParams;
	for (const TEnumAsByte<ECollisionChannel>& Channel : ObstacleFilterProfile->CollisionChannels)
	{
		ObjectQueryParams.AddObjectTypesToQuery(Channel);
	}

	TArray<FOverlapResult> OverlapResults;
	if (!GetOverlappingComponentsInVolume(ObjectQueryParams, OverlapResults)) return;

	for (const FOverlapResult& Result : OverlapResults)
	{
		UPrimitiveComponent* OverlappedComponent = Result.GetComponent();
		AActor* OverlappedActor = Result.GetActor();

		if (OverlappedActor && OverlappedActor->FindComponentByClass<UPathfindingComponent>())
		{
			continue;
		}

		if (ObstacleFilterProfile->DoesActorMatch(OverlappedActor, OverlappedComponent))
		{
			const FBox ObstacleBounds = OverlappedComponent->Bounds.GetBox();
			FIntVector MinVoxelIndex = WorldToVoxelIndex(ObstacleBounds.Min);
			FIntVector MaxVoxelIndex = WorldToVoxelIndex(ObstacleBounds.Max - FVector(KINDA_SMALL_NUMBER));

			FIntVector ClampedMin, ClampedMax;
			if (!ClampVoxelIndicesToVolume(MinVoxelIndex, MaxVoxelIndex, ClampedMin, ClampedMax)) continue;

			for (int32 z = ClampedMin.Z; z <= ClampedMax.Z; ++z)
			{
				for (int32 y = ClampedMin.Y; y <= ClampedMax.Y; ++y)
				{
					for (int32 x = ClampedMin.X; x <= ClampedMax.X; ++x)
					{
						const FIntVector CurrentVoxelIndex(x, y, z);
						if (CheckVoxelOverlap(CurrentVoxelIndex, OverlappedComponent))
						{
							OutVoxelIndices.Add(CurrentVoxelIndex);
						}
					}
				}
			}
		}
	}
}

void AFenwickVoxelVolume::ApplyDynamicObstacleChanges(const TSet<FIntVector>& NewFrameObstacles)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(VoxelVolume_ApplyDynamicChanges);
	
	const TSet<FIntVector> RemovedObstacles = CurrentDynamicObstacles.Difference(NewFrameObstacles);
	for (const FIntVector& Index : RemovedObstacles)
	{
		UpdateDynamicVoxelState(Index, false);
	}

	const TSet<FIntVector> AddedObstacles = NewFrameObstacles.Difference(CurrentDynamicObstacles);
	for (const FIntVector& Index : AddedObstacles)
	{
		UpdateDynamicVoxelState(Index, true);
	}
}

void AFenwickVoxelVolume::UpdateDynamicVoxelState(const FIntVector& Index, bool bIsObstacle)
{
	if (!IsVoxelIndexValid(Index)) return;

	const int32 ChunksPerRow = MapSize.X / 128;
	const int32 ChunkIndex = (Index.Z * MapSize.Y + Index.Y) * ChunksPerRow + (Index.X / 128);
	const int32 BitIndex = Index.X % 128;

	F128BitChunk& TargetChunk = DynamicObstacleBitmask[ChunkIndex];
	const int32 WordIndex = BitIndex / 64;
	const uint64 BitMask = 1ULL << (BitIndex % 64);

	if (bIsObstacle)
	{
		TargetChunk.Words[WordIndex] |= BitMask;
	}
	else
	{
		TargetChunk.Words[WordIndex] &= ~BitMask;
	}
}

//======================================================================
// --- HELPER FUNCTIONS ---
//======================================================================

FIntVector AFenwickVoxelVolume::WorldToChunkCoords(const FVector& WorldPos) const
{
	const FVector LocalPos = GetTransform().InverseTransformPosition(WorldPos);
	const float ChunkWorldSize = CHUNK_SIZE * VoxelExtent;
	return FIntVector(
		FMath::FloorToInt(LocalPos.X / ChunkWorldSize),
		FMath::FloorToInt(LocalPos.Y / ChunkWorldSize),
		FMath::FloorToInt(LocalPos.Z / ChunkWorldSize)
	);
}

FORCEINLINE bool AFenwickVoxelVolume::IsVoxelIndexValid(const FIntVector& VoxelIndex) const
{
	return VoxelIndex.X >= 0 && VoxelIndex.X < MapSize.X &&
		VoxelIndex.Y >= 0 && VoxelIndex.Y < MapSize.Y &&
		VoxelIndex.Z >= 0 && VoxelIndex.Z < MapSize.Z;
}

FString AFenwickVoxelVolume::GetFinalSaveDirectory() const
{
	if (!GetWorld())
	{
		UE_LOG(LogTemp, Error, TEXT("GetFinalSaveDirectory: GetWorld() returned null."));
		return BakedDataSaveDirectory;
	}
	const FString LevelName = FPackageName::GetShortName(GetWorld()->GetName());
	const FString ActorName = GetName();
	return FPaths::Combine(BakedDataSaveDirectory, LevelName, ActorName);
}

FString AFenwickVoxelVolume::GetChunkAssetPath(const FIntVector& ChunkCoords) const
{
	const FString DataDirectory = FPaths::Combine(GetFinalSaveDirectory(), TEXT("Data"));
	const FString AssetName = FString::Printf(TEXT("DA_ChunkData_%d_%d_%d"), ChunkCoords.X, ChunkCoords.Y, ChunkCoords.Z);
	return FPaths::Combine(DataDirectory, AssetName);
}

//======================================================================
// --- EDITOR BAKING ---
//======================================================================
#if WITH_EDITOR

void AFenwickVoxelVolume::BakeStaticDataToChunks()
{
	if (!ValidateBakePrerequisites()) return;

	UWorld* OriginalWorld = GetWorld();
	if (!OriginalWorld) return;
	
	UPackage* WorldPackage = OriginalWorld->GetPackage();
	if (!SaveCurrentWorld(WorldPackage)) return;
	
	const FString OriginalWorldPackageName = WorldPackage->GetName();
	const FString FinalSaveDirectory = GetFinalSaveDirectory();

	FScopedSlowTask BakeTask(5, FText::FromString(TEXT("Baking Voxel Data...")));
	BakeTask.MakeDialog();

	BakeTask.EnterProgressFrame(1, FText::FromString(TEXT("Cleaning up old assets...")));
	CleanUpAllChunkAssets();

	BakeTask.EnterProgressFrame(1, FText::FromString(TEXT("Gathering exclusions...")));
	TSet<FIntVector> ExcludedVoxels;
	GatherBakeExclusionVoxels(ExcludedVoxels);

	BakeTask.EnterProgressFrame(1, FText::FromString(TEXT("Scanning static obstacles...")));
	TSet<FIntVector> AllStaticObstacles;
	ScanForAllStaticObstacles(ExcludedVoxels, AllStaticObstacles);

	BakeTask.EnterProgressFrame(1, FText::FromString(TEXT("Grouping obstacles into chunks...")));
	TMap<FIntVector, TArray<FIntVector>> ObstaclesByChunk;
	GroupObstaclesIntoChunks(AllStaticObstacles, ObstaclesByChunk);

	BakeTask.EnterProgressFrame(1, FText::FromString(TEXT("Creating new chunk assets...")));
	CreateOrUpdateChunkAssets(ObstaclesByChunk, FinalSaveDirectory);
	
	UE_LOG(LogTemp, Log, TEXT("Baking process completed. Restoring original level: %s"), *OriginalWorldPackageName);
	UEditorLoadingAndSavingUtils::LoadMap(OriginalWorldPackageName);
}

bool AFenwickVoxelVolume::SaveCurrentWorld(UPackage* WorldPackage)
{
	if (WorldPackage && WorldPackage->IsDirty())
	{
		const bool bSaveResult = FEditorFileUtils::SaveDirtyPackages(true, true, false);
		if (!bSaveResult || WorldPackage->IsDirty())
		{
			FMessageDialog::Open(EAppMsgType::Ok, FText::FromString(TEXT("Baking requires the current level to be saved first.")));
			return false;
		}
	}
    return true;
}

bool AFenwickVoxelVolume::ValidateBakePrerequisites() const
{
	if (MapSize.X <= 0 || MapSize.Y <= 0 || MapSize.Z <= 0 || MapSize.X % CHUNK_SIZE != 0 || MapSize.Y % CHUNK_SIZE != 0 || MapSize.Z % CHUNK_SIZE != 0)
	{
		UE_LOG(LogTemp, Error, TEXT("Baking failed: MapSize must be a positive multiple of CHUNK_SIZE (%d)."), CHUNK_SIZE);
		return false;
	}
	if (BakedDataSaveDirectory.IsEmpty())
	{
		UE_LOG(LogTemp, Error, TEXT("Baking failed: Please specify a valid 'Baked Data Save Directory'."));
		return false;
	}
	return true;
}

void AFenwickVoxelVolume::GatherBakeExclusionVoxels(TSet<FIntVector>& OutExcludedVoxels)
{
	OutExcludedVoxels.Empty();

	// 헬퍼 람다: 특정 액터의 바운딩 박스를 복셀화하여 제외 목록에 추가합니다.
	auto AddActorBoundsToExclusionSet = [this, &OutExcludedVoxels](AActor* ActorToExclude)
	{
		if (!ActorToExclude) return;
		
		FVector Origin, BoxExtent;
		ActorToExclude->GetActorBounds(true, Origin, BoxExtent);
		
		FIntVector MinVoxel, MaxVoxel;
		if (ClampVoxelIndicesToVolume(WorldToVoxelIndex(Origin - BoxExtent), WorldToVoxelIndex(Origin + BoxExtent), MinVoxel, MaxVoxel))
		{
			for (int32 z = MinVoxel.Z; z <= MaxVoxel.Z; ++z)
			{
				for (int32 y = MinVoxel.Y; y <= MaxVoxel.Y; ++y)
				{
					for (int32 x = MinVoxel.X; x <= MaxVoxel.X; ++x)
					{
						OutExcludedVoxels.Add(FIntVector(x, y, z));
					}
				}
			}
		}
	};

	// 에디터에서 직접 지정한 액터들을 제외 목록에 추가합니다.
	for (AActor* ExcludedActor : BakeExcludeActors)
	{
		AddActorBoundsToExclusionSet(ExcludedActor);
	}

	// 이 볼륨을 사용하는 모든 경로탐색 컴포넌트의 소유 액터도 제외 목록에 추가합니다.
	// (예: 플레이어, NPC 등은 베이크 대상이 아니어야 함)
	for (TObjectIterator<UPathfindingComponent> It; It; ++It)
	{
		UPathfindingComponent* PFComp = *It;
		if (PFComp && PFComp->GetWorld() == GetWorld() && PFComp->GetTargetVolume() == this)
		{
			if (AActor* OwnerActor = PFComp->GetOwner())
			{
				AddActorBoundsToExclusionSet(OwnerActor);
			}
		}
	}
	UE_LOG(LogTemp, Log, TEXT("Gathered %d voxels to exclude from baking."), OutExcludedVoxels.Num());
}

void AFenwickVoxelVolume::ScanForAllStaticObstacles(const TSet<FIntVector>& ExcludedVoxels, TSet<FIntVector>& OutStaticObstacles) const
{
	OutStaticObstacles.Empty();
	
	// 월드에 배치된 정적(Static) 지오메트리만 쿼리합니다.
	FCollisionObjectQueryParams StaticObjectQueryParams(ECC_WorldStatic);
	TArray<FOverlapResult> OverlapResults;
	GetOverlappingComponentsInVolume(StaticObjectQueryParams, OverlapResults);

	for (const FOverlapResult& Result : OverlapResults)
	{
		if (UPrimitiveComponent* Comp = Result.GetComponent())
		{
			const FBox ComponentBounds = Comp->Bounds.GetBox();

			FIntVector MinVoxel, MaxVoxel;
			if (!ClampVoxelIndicesToVolume(WorldToVoxelIndex(ComponentBounds.Min), WorldToVoxelIndex(ComponentBounds.Max), MinVoxel, MaxVoxel))
			{
				continue;
			}
			
			for (int32 z = MinVoxel.Z; z <= MaxVoxel.Z; ++z)
			{
				for (int32 y = MinVoxel.Y; y <= MaxVoxel.Y; ++y)
				{
					for (int32 x = MinVoxel.X; x <= MaxVoxel.X; ++x)
					{
						const FIntVector VoxelIndex(x, y, z);
						
						// 이미 장애물로 판명되었거나, 제외 목록에 있다면 건너뜁니다.
						if (OutStaticObstacles.Contains(VoxelIndex) || ExcludedVoxels.Contains(VoxelIndex))
						{
							continue;
						}
						
						// 복셀과 컴포넌트가 실제로 겹치는지 정밀 검사를 수행합니다.
						if (CheckVoxelOverlap(VoxelIndex, Comp))
						{
							OutStaticObstacles.Add(VoxelIndex);
						}
					}
				}
			}
		}
	}
	UE_LOG(LogTemp, Log, TEXT("Scan complete. Found %d unique static obstacle voxels."), OutStaticObstacles.Num());
}

void AFenwickVoxelVolume::GroupObstaclesIntoChunks(const TSet<FIntVector>& AllObstacles, TMap<FIntVector, TArray<FIntVector>>& OutObstaclesByChunk) const
{
	OutObstaclesByChunk.Empty();
	for (const FIntVector& ObstacleVoxel : AllObstacles)
	{
		// 복셀의 월드 인덱스를 청크 좌표로 변환합니다.
		const FIntVector ChunkCoord(
			FMath::FloorToInt(static_cast<float>(ObstacleVoxel.X) / CHUNK_SIZE),
			FMath::FloorToInt(static_cast<float>(ObstacleVoxel.Y) / CHUNK_SIZE),
			FMath::FloorToInt(static_cast<float>(ObstacleVoxel.Z) / CHUNK_SIZE)
		);
		
		// 해당 청크 좌표의 배열에 복셀 인덱스를 추가합니다.
		OutObstaclesByChunk.FindOrAdd(ChunkCoord).Add(ObstacleVoxel);
	}
	UE_LOG(LogTemp, Log, TEXT("Grouped obstacles into %d non-empty chunks."), OutObstaclesByChunk.Num());
}


void AFenwickVoxelVolume::CleanUpAllChunkAssets() const
{
	// 1. 애셋 레지스트리 모듈을 로드합니다.
	FAssetRegistryModule& AssetRegistryModule = FModuleManager::LoadModuleChecked<FAssetRegistryModule>("AssetRegistry");
	IAssetRegistry& AssetRegistry = AssetRegistryModule.Get();

	// 2. 애셋 레지스트리가 모든 애셋을 스캔할 때까지 기다려 최신 상태를 보장합니다. (선택적이지만 안전함)
	AssetRegistry.SearchAllAssets(true);

	// 3. UStaticVoxelChunkData 클래스를 가진 모든 애셋 데이터를 찾습니다.
	TArray<FAssetData> ChunkAssetsToDelete;
	// GetAssetsByClass는 FTopLevelAssetPath를 인자로 받습니다.
	AssetRegistry.GetAssetsByClass(UStaticVoxelChunkData::StaticClass()->GetClassPathName(), ChunkAssetsToDelete, true); // bSearchSubClasses = true

	// 4. 베이킹 경로 필터링 (선택적이지만 권장)
	// 만약 다른 곳에서도 UStaticVoxelChunkData를 사용할 경우, 우리 볼륨의 베이킹 경로에 있는 것만 삭제하도록 필터링합니다.
	const FString FinalSaveDir = GetFinalSaveDirectory();
	ChunkAssetsToDelete.RemoveAll([&FinalSaveDir](const FAssetData& AssetData)
	{
		return !AssetData.PackagePath.ToString().StartsWith(FinalSaveDir);
	});


	if (ChunkAssetsToDelete.IsEmpty())
	{
		UE_LOG(LogTemp, Log, TEXT("No old chunk assets found to clean up."));
		return;
	}

	//UE_LOG(LogTemp, Log, TEXT("Found %d old chunk assets to delete in path %s."), ChunkAssetsToDelete.Num(), *FinalSaveDir);
	//if (ObjectTools::DeleteAssets(ChunkAssetsToDelete, false) > 0)
	//{
	//	UE_LOG(LogTemp, Log, TEXT("Successfully deleted old chunk assets."));
	//}
	//else
	//{
	//	UE_LOG(LogTemp, Warning, TEXT("Failed to delete old chunk assets. They might be in use or have failed to load."));
	//}

	// TO-DO: Added Safety checks but not all cases are handled.
	TArray<UObject*> LoadedAssetsToDelete;
	for (const FAssetData& AssetData : ChunkAssetsToDelete)
	{
		UObject* LoadedAsset=AssetData.GetAsset();
		if (LoadedAsset)
		{
			LoadedAssetsToDelete.Add(LoadedAsset);
		}
		else
		{
			UE_LOG(LogTemp, Warning, TEXT("Failed to load asset for deletion (possibly corrupt): %s. Skipping."), *AssetData.GetObjectPathString());
		}
	}

	if(LoadedAssetsToDelete.IsEmpty())
	{
		UE_LOG(LogTemp, Warning, TEXT("Found %d assets in path, but none could be loaded for deletion."), ChunkAssetsToDelete.Num());
		return;
	}	
	const int32 DeletedCount = ObjectTools::DeleteObjects(LoadedAssetsToDelete, false);
	if (DeletedCount > 0)
	{
		UE_LOG(LogTemp, Log, TEXT("Successfully deleted %d old chunk assets."), DeletedCount);
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("Attempted to delete %d assets, but none were deleted. They might be in use."), LoadedAssetsToDelete.Num());
	}
}

void AFenwickVoxelVolume::CreateOrUpdateChunkAssets(const TMap<FIntVector, TArray<FIntVector>>& ObstaclesByChunk, const FString& InFinalSaveDirectory)
{
    if (ObstaclesByChunk.Num() == 0) return;
    check(GEditor);

    const FString DataAssetDirectory = FPaths::Combine(InFinalSaveDirectory, TEXT("Data"));
    IFileManager::Get().MakeDirectory(*FPaths::ConvertRelativePathToFull(DataAssetDirectory), true); // 디렉토리가 없으면 생성

    TArray<UPackage*> PackagesToSave;

    for (const auto& Pair : ObstaclesByChunk)
    {
        const FIntVector& ChunkCoord = Pair.Key;
        const FString AssetName = FString::Printf(TEXT("Chunk_%d_%d_%d"), ChunkCoord.X, ChunkCoord.Y, ChunkCoord.Z);
        const FString PackagePath = FPaths::Combine(DataAssetDirectory, AssetName);

        // --- 널 포인터 추적 시작 ---

        UPackage* Package = FindPackage(nullptr, *PackagePath);
        if (!Package)
        {
            Package = CreatePackage(*PackagePath);
        }
        
        if (!IsValid(Package))
        {
            UE_LOG(LogTemp, Fatal, TEXT("CreateOrUpdateChunkAssets: CreatePackage() failed for path: %s"), *PackagePath);
            return; // 여기서 멈추면 패키지 생성 실패
        }

        UStaticVoxelChunkData* ChunkAsset = NewObject<UStaticVoxelChunkData>(Package, FName(*AssetName), RF_Public | RF_Standalone | RF_Transactional);
        if (!IsValid(ChunkAsset))
        {
            UE_LOG(LogTemp, Fatal, TEXT("CreateOrUpdateChunkAssets: NewObject<UStaticVoxelChunkData>() failed for asset: %s in package: %s"), *AssetName, *PackagePath);
            return; // 여기서 멈추면 애셋 생성 실패
        }

        UE_LOG(LogTemp, Log, TEXT("Successfully created asset: %s"), *ChunkAsset->GetPathName());

        ChunkAsset->ChunkCoords = ChunkCoord;
        TArray<F128BitChunk> TempObstacleData;
        
        const int32 ChunksPerAssetRow = CHUNK_SIZE / 128;
        const int32 TotalChunksInAsset = ChunksPerAssetRow * CHUNK_SIZE * CHUNK_SIZE;
        TempObstacleData.Init(F128BitChunk(), TotalChunksInAsset);

        for (const FIntVector& ObstacleVoxel : Pair.Value)
        {
            const int32 LocalX = ObstacleVoxel.X % CHUNK_SIZE;
            const int32 LocalY = ObstacleVoxel.Y % CHUNK_SIZE;
            const int32 LocalZ = ObstacleVoxel.Z % CHUNK_SIZE;
            const int32 ChunkIndex1D = (LocalZ * CHUNK_SIZE * ChunksPerAssetRow) + (LocalY * ChunksPerAssetRow) + (LocalX / 128);
            const int32 BitIndexInChunk128 = LocalX % 128;
            if (TempObstacleData.IsValidIndex(ChunkIndex1D))
            {
                TempObstacleData[ChunkIndex1D].Words[BitIndexInChunk128 / 64] |= (1ULL << (BitIndexInChunk128 % 64));
            }
        }
        
        ChunkAsset->FillBulkData(TempObstacleData);

        FAssetRegistryModule::GetRegistry().AssetCreated(ChunkAsset);
        Package->MarkPackageDirty();
        PackagesToSave.Add(Package);
    }

    if (PackagesToSave.Num() > 0)
    {
        UE_LOG(LogTemp, Log, TEXT("Attempting to save %d packages."), PackagesToSave.Num());
        if (!UEditorLoadingAndSavingUtils::SavePackages(PackagesToSave, true))
        {
            UE_LOG(LogTemp, Error, TEXT("CreateOrUpdateChunkAssets: SavePackages failed."));
        }
        else
        {
            UE_LOG(LogTemp, Log, TEXT("CreateOrUpdateChunkAssets: SavePackages successful."));
        }
    }
}
#endif // WITH_EDITOR