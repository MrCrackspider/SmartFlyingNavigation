// Fill out your copyright notice in the Description page of Project Settings.


#include "Pathfinder/VoxelVolume.h"

#include "EpicProject_3DNavi/Voxel/DynamicObstacle.h"

// Sets default values
AVoxelVolume::AVoxelVolume() : MapSize(64, 64, 64), VoxelExtent(100.f)
{
	PrimaryActorTick.bCanEverTick = true;
}

void AVoxelVolume::BeginPlay()
{
	Super::BeginPlay();

	// Strategy를 어떻게 Initiaize할지 결정해야함	

	InitializeMap(MapSize);
	BuildStaticObstacles();
}

void AVoxelVolume::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	UpdateDynamicObstacles();
	if (bDrawDebugObstacles)
	{
		DrawDebugVoxelObstacles();
	}
}

bool AVoxelVolume::IsVoxelWalkable(const FIntVector& Index) const
{
	if (!IsIndexValid(Index)) return false;

	const int32 ChunkX = Index.X / 64;
	const int32 BitPos = Index.X % 64;
	const int32 ChunkArrIndex = GetFlatIndex_X(ChunkX, Index.Y, Index.Z);

	return (ObstacleBitmasks_X[ChunkArrIndex] & (1ULL << BitPos)) == 0;
}

uint64 AVoxelVolume::GetObstacleChunk_X(int32 ChunkX, int32 Y, int32 Z) const
{
	const int32 ChunksAlongX = MapSize.X / static_cast<int32>(CHUNK_SIZE);

	if (ChunkX < 0 || ChunkX >= ChunksAlongX || Y < 0 || Y >= MapSize.Y || Z < 0 || Z >= MapSize.Z)
	{
		return ~0ULL;
	}
	return ObstacleBitmasks_X[GetFlatIndex_X(ChunkX, Y, Z)];
}

uint64 AVoxelVolume::GetObstacleChunk_Y(int32 ChunkY, int32 X, int32 Z) const
{
	const int32 ChunksAlongY = MapSize.Y / static_cast<int32>(CHUNK_SIZE);

	if (ChunkY < 0 || ChunkY >= ChunksAlongY || X < 0 || X >= MapSize.X || Z < 0 || Z >= MapSize.Z)
	{
		return ~0ULL;
	}
	return ObstacleBitmasks_Y[GetFlatIndex_Y(ChunkY, X, Z)];
}

uint64 AVoxelVolume::GetObstacleChunk_Z(int32 ChunkZ, int32 X, int32 Y) const
{
	const int32 ChunksAlongZ = MapSize.Z / static_cast<int32>(CHUNK_SIZE);

	if (ChunkZ < 0 || ChunkZ >= ChunksAlongZ || X < 0 || X >= MapSize.X || Y < 0 || Y >= MapSize.Y)
	{
		return ~0ULL;
	}
	return ObstacleBitmasks_Z[GetFlatIndex_Z(ChunkZ, X, Y)];
}

void AVoxelVolume::InitializeMap(const FIntVector& InMapSize)
{
	MapSize = InMapSize;

	// 비트마스크 최적화를 위해 모든 축의 크기가 CHUNK_SIZE의 배수여야 함
	check(MapSize.X % CHUNK_SIZE == 0);
	check(MapSize.Y % CHUNK_SIZE == 0);
	check(MapSize.Z % CHUNK_SIZE == 0);

	// X축 우선 배열 초기화
	const int32 ChunksAlongX = MapSize.X / CHUNK_SIZE;
	const int32 TotalChunksX = ChunksAlongX * MapSize.Y * MapSize.Z;
	ObstacleBitmasks_X.Init(0, TotalChunksX);

	// Y축 우선 배열 초기화
	const int32 ChunksAlongY = MapSize.Y / CHUNK_SIZE;
	const int32 TotalChunksY = ChunksAlongY * MapSize.X * MapSize.Z;
	ObstacleBitmasks_Y.Init(0, TotalChunksY);

	// Z축 우선 배열 초기화
	const int32 ChunksAlongZ = MapSize.Z / CHUNK_SIZE;
	const int32 TotalChunksZ = ChunksAlongZ * MapSize.X * MapSize.Y;
	ObstacleBitmasks_Z.Init(0, TotalChunksZ);
}

void AVoxelVolume::BuildStaticObstacles()
{
	FCollisionObjectQueryParams StaticObjectQueryParams;
	StaticObjectQueryParams.AddObjectTypesToQuery(ECC_WorldStatic); // 정적 장애물 검색

	TArray<FOverlapResult> OverlapResults;
	if (!GetOverlappingComponentsInVolume(StaticObjectQueryParams, OverlapResults))
	{
		UE_LOG(LogTemp, Warning, TEXT("BuildStaticObstacles: GetOverlappingComponentsInVolume 실패 또는 결과 없음"));
		return; // 오버랩 쿼리 실패 또는 결과 없음
	}

	for (const FOverlapResult& Result : OverlapResults)
	{
		AActor* OverlappedActor = Result.GetActor();
		UPrimitiveComponent* OverlappedComponent = Result.GetComponent();

		if (OverlappedActor == nullptr || OverlappedComponent == nullptr)
		{
			continue;;
		}
		if (OverlappedComponent->IsQueryCollisionEnabled() == false)
		{
			continue;
		}

		if (OverlappedActor->GetComponentByClass<UDynamicObstacle>())
		{
			continue;
		}

		FVector ObstacleOrigin, ObstacleExtent;
		OverlappedComponent->Bounds.GetBox().GetCenterAndExtents(ObstacleOrigin, ObstacleExtent);
		const FBox ObstacleBounds(ObstacleOrigin - ObstacleExtent, ObstacleOrigin + ObstacleExtent);

		// 장애물 바운딩 박스가 차지하는 복셀 인덱스 계산
		FIntVector MinVoxelIndex = WorldToVoxelIndex(ObstacleBounds.Min);
		
		const float Epsilon = KINDA_SMALL_NUMBER;
		FVector AdjustedMaxBound = ObstacleBounds.Max - FVector(Epsilon, Epsilon, Epsilon);
		FIntVector MaxVoxelIndex = WorldToVoxelIndex(AdjustedMaxBound);

		if (ClampVoxelIndicesToVolume(MinVoxelIndex, MaxVoxelIndex) == false)
		{
			continue; // 클램핑 후 유효한 범위가 없다면 다음 컴포넌트로 넘어감
		}
		for (int32 z = MinVoxelIndex.Z; z <= MaxVoxelIndex.Z; ++z)
		{
			for (int32 y = MinVoxelIndex.Y; y <= MaxVoxelIndex.Y; ++y)
			{
				for (int32 x = MinVoxelIndex.X; x <= MaxVoxelIndex.X; ++x)
				{
					FIntVector CurrentVoxelIndex(x, y, z);
					if (CheckVoxelOverlap(CurrentVoxelIndex, OverlappedComponent))
					{
						SetStaticObstacle(CurrentVoxelIndex, true);
					}
				}
			}
		}
	}
}

void AVoxelVolume::UpdateDynamicObstacles()
{
    if (GetWorld() == nullptr)
    {
        return;
    }

    // 1. 이전 프레임의 동적 장애물 복셀 상태 복원
    for (const TPair<FIntVector, bool>& Pair : DynamicObstacleOverriddenVoxels)
    {
        const FIntVector& Index = Pair.Key; // const 참조로 변경
        const bool bWasOriginallyObstacle = Pair.Value;
        if (IsIndexValid(Index))
        {
            SetStaticObstacle(Index, bWasOriginallyObstacle);
        }
    }
    DynamicObstacleOverriddenVoxels.Empty();

    FCollisionObjectQueryParams DynamicObjectQueryParams;
    
    DynamicObjectQueryParams.AddObjectTypesToQuery(ECC_Pawn);
    DynamicObjectQueryParams.AddObjectTypesToQuery(ECC_WorldDynamic);
    DynamicObjectQueryParams.AddObjectTypesToQuery(ECC_PhysicsBody);
    TArray<FOverlapResult> OverlapResults;
    if (!GetOverlappingComponentsInVolume(DynamicObjectQueryParams, OverlapResults))
    {
        UE_LOG(LogTemp, Error, TEXT("UpdateDynamicObstacles: No dynamic components found in volume via GetOverlappingComponentsInVolume."));
      
    }

    for (const FOverlapResult& Result : OverlapResults)
    {
        AActor* OverlappedActor = Result.GetActor();
        UPrimitiveComponent* OverlappedComponent = Result.GetComponent(); // 오버랩된 특정 프리미티브 컴포넌트

        if (OverlappedActor == nullptr || OverlappedComponent == nullptr)
        {
            continue;
        }

        // 이 액터가 UDynamicObstacle 컴포넌트를 가지고 있는지 확인
        if (OverlappedActor->GetComponentByClass<UDynamicObstacle>() == nullptr)
        {
            continue; // UDynamicObstacle 컴포넌트가 없으면 이 액터/컴포넌트는 동적 장애물로 처리하지 않음
        }

    	if (OverlappedComponent->IsQueryCollisionEnabled() == false)
        {
            continue;
        }
            
        FVector ObstacleCompOrigin, ObstacleCompExtent;
        OverlappedComponent->Bounds.GetBox().GetCenterAndExtents(ObstacleCompOrigin, ObstacleCompExtent);
        FBox ObstacleCompBounds(ObstacleCompOrigin - ObstacleCompExtent, ObstacleCompOrigin + ObstacleCompExtent);
    	
        FIntVector MinVoxelIndex = WorldToVoxelIndex(ObstacleCompBounds.Min);
        
        // "한 줄 더 생기는" 문제 해결을 위해 Max 좌표에서 작은 값을 뺌
        const float Epsilon = KINDA_SMALL_NUMBER; // 1.e-4f
        FVector AdjustedMaxBound = ObstacleCompBounds.Max - FVector(Epsilon, Epsilon, Epsilon);
        FIntVector MaxVoxelIndex = WorldToVoxelIndex(AdjustedMaxBound);

        // ClampVoxelIndicesToVolume 헬퍼 함수 사용
        if (!ClampVoxelIndicesToVolume(MinVoxelIndex, MaxVoxelIndex))
        {
        	UE_LOG(LogTemp, Error, TEXT("UpdateDynamicObstacles: No IsQueryCollisionEnabled"));
            continue;
        }

        for (int32 z = MinVoxelIndex.Z; z <= MaxVoxelIndex.Z; ++z)
        {
            for (int32 y = MinVoxelIndex.Y; y <= MaxVoxelIndex.Y; ++y)
            {
                for (int32 x = MinVoxelIndex.X; x <= MaxVoxelIndex.X; ++x)
                {
                    FIntVector CurrentVoxelIndex(x, y, z);

                    // 이미 이번 프레임에서 다른 동적 장애물 컴포넌트에 의해 이 복셀이 처리되었다면 건너뜀
                    // (여러 동적 장애물이 겹칠 때, 하나의 상태만 저장해도 되는 경우에 대한 최적화)
                    if (DynamicObstacleOverriddenVoxels.Contains(CurrentVoxelIndex))
                    {
					 	UE_LOG(LogTemp, Verbose, TEXT("DynamicObstacleOverriddenVoxels"));
                        continue;
                    }
                    
                    // CheckVoxelOverlap 헬퍼 함수 사용
                    if (CheckVoxelOverlap(CurrentVoxelIndex, OverlappedComponent))
                    {
                        bool bWasOriginallyObstacle = !IsVoxelWalkable(CurrentVoxelIndex);
                        DynamicObstacleOverriddenVoxels.Add(CurrentVoxelIndex, bWasOriginallyObstacle);
                        SetStaticObstacle(CurrentVoxelIndex, true);
                    }
                }
            }
        }
    }
}

void AVoxelVolume::SetStaticObstacle(const FIntVector& Index, bool bIsObstacle)
{
	if (!IsIndexValid(Index)) return;

	// 1. X축 비트마스크 업데이트
	{
		const uint32 ChunkX = Index.X / CHUNK_SIZE;
		const uint32 BitPos = Index.X % CHUNK_SIZE;
		const uint32 ChunkArrIndex = GetFlatIndex_X(ChunkX, Index.Y, Index.Z);
		if (bIsObstacle)
			ObstacleBitmasks_X[ChunkArrIndex] |= (1ULL << BitPos);
		else
			ObstacleBitmasks_X[ChunkArrIndex] &= ~(1ULL << BitPos);
	}

	// 2. Y축 비트마스크 업데이트
	{
		const int32 ChunkY = Index.Y / CHUNK_SIZE;
		const int32 BitPos = Index.Y % CHUNK_SIZE;
		const int32 ChunkArrIndex = GetFlatIndex_Y(ChunkY, Index.X, Index.Z);
		if (bIsObstacle)
			ObstacleBitmasks_Y[ChunkArrIndex] |= (1ULL << BitPos);
		else
			ObstacleBitmasks_Y[ChunkArrIndex] &= ~(1ULL << BitPos);
	}

	// 3. Z축 비트마스크 업데이트
	{
		const int32 ChunkZ = Index.Z / CHUNK_SIZE;
		const int32 BitPos = Index.Z % CHUNK_SIZE;
		const int32 ChunkArrIndex = GetFlatIndex_Z(ChunkZ, Index.X, Index.Y);
		if (bIsObstacle)
			ObstacleBitmasks_Z[ChunkArrIndex] |= (1ULL << BitPos);
		else
			ObstacleBitmasks_Z[ChunkArrIndex] &= ~(1ULL << BitPos);
	}
}


bool AVoxelVolume::CheckVoxelOverlap(const FIntVector& VoxelIndex, UPrimitiveComponent* ComponentToTest) const
{
	if (ComponentToTest == nullptr)
	{
		return false;
	}
	if (ComponentToTest->IsQueryCollisionEnabled() == false)
	{
		return false;
	}
	if (IsIndexValid(VoxelIndex) == false)
	{
		return false;
	}

	const FVector VoxelCenter = VoxelIndexToWorld(VoxelIndex);
	const FVector VoxelHalfExtent = FVector(VoxelExtent * 0.5f);

	return ComponentToTest->OverlapComponent(
		VoxelCenter,
		FQuat::Identity,
		FCollisionShape::MakeBox(VoxelHalfExtent)
	);
}

bool AVoxelVolume::ClampVoxelIndicesToVolume(FIntVector& MinVoxelIndex, FIntVector& MaxVoxelIndex) const
{
	MinVoxelIndex.X = FMath::Max(MinVoxelIndex.X, 0);
	MinVoxelIndex.Y = FMath::Max(MinVoxelIndex.Y, 0);
	MinVoxelIndex.Z = FMath::Max(MinVoxelIndex.Z, 0);

	MaxVoxelIndex.X = FMath::Min(MaxVoxelIndex.X, MapSize.X - 1);
	MaxVoxelIndex.Y = FMath::Min(MaxVoxelIndex.Y, MapSize.Y - 1);
	MaxVoxelIndex.Z = FMath::Min(MaxVoxelIndex.Z, MapSize.Z - 1);

	return (MaxVoxelIndex.X >= MinVoxelIndex.X &&
		MaxVoxelIndex.Y >= MinVoxelIndex.Y &&
		MaxVoxelIndex.Z >= MinVoxelIndex.Z);
}

bool AVoxelVolume::GetOverlappingComponentsInVolume(const FCollisionObjectQueryParams& ObjectQueryParams,
                                                    TArray<FOverlapResult>& OutResults) const
{
	OutResults.Empty();
	UWorld* World = GetWorld();
	if (World == nullptr)
	{
		return false;
	}

	const FVector ActorLocation = GetActorLocation();
	//const FQuat ActorRotation = GetActorQuat();
	const FVector TotalVolumeDimensions = FVector(MapSize.X, MapSize.Y, MapSize.Z) * VoxelExtent;
	const FVector VolumeHalfExtent = TotalVolumeDimensions * 0.5f;

	const FVector VolumeLocalCenter = VolumeHalfExtent;
	//const FVector QueryWorldOrigin = ActorLocation + ActorRotation.RotateVector(VolumeLocalCenter);

	FCollisionShape VolumeQueryShape = FCollisionShape::MakeBox(VolumeHalfExtent);
	FCollisionQueryParams CollisionQueryParams;
	CollisionQueryParams.AddIgnoredActor(this);
	
	FVector BoxExtent = VolumeQueryShape.GetBox(); // 박스의 HalfExtent를 가져옵니다.
	return World->OverlapMultiByObjectType(OutResults,
	                                       VolumeLocalCenter,
	                                       FQuat::Identity,
	                                       ObjectQueryParams, // 인자로 받은 오브젝트 쿼리 파라미터 사용
	                                       VolumeQueryShape,
	                                       CollisionQueryParams);
}

bool AVoxelVolume::IsIndexValid(const FIntVector& Index) const
{
	return Index.X >= 0 && Index.X < MapSize.X &&
		Index.Y >= 0 && Index.Y < MapSize.Y &&
		Index.Z >= 0 && Index.Z < MapSize.Z;
}

FVector AVoxelVolume::VoxelIndexToWorld(const FIntVector& VoxelIndex) const
{
	const FVector LocalCenter = FVector(VoxelIndex) * VoxelExtent + (VoxelExtent * 0.5f);
	return GetActorTransform().TransformPosition(LocalCenter);
}

FIntVector AVoxelVolume::WorldToVoxelIndex(const FVector& WorldPosition) const
{
	if (VoxelExtent <= 0.f)
	{
		UE_LOG(LogTemp, Error, TEXT("AVoxelVolume::WorldToVoxelIndex - VoxelExtent is zero or negative."));
		return FIntVector(EForceInit::ForceInitToZero);
	}

	const FVector LocalPosition = GetActorTransform().InverseTransformPosition(WorldPosition);
	const int32 IndexX = FMath::FloorToInt(LocalPosition.X / VoxelExtent);
	const int32 IndexY = FMath::FloorToInt(LocalPosition.Y / VoxelExtent);
	const int32 IndexZ = FMath::FloorToInt(LocalPosition.Z / VoxelExtent);

	return FIntVector(IndexX, IndexY, IndexZ);
}
void AVoxelVolume::DrawDebugVoxelObstacles() const
{
	UWorld* World = GetWorld();
	if (!World || MapSize.IsZero() || VoxelExtent <= 0.f) // MapSize.IsZero()는 FIntVector에 해당 멤버가 없으므로, MapSize.X == 0 등으로 확인
	{
		return;
	}
	if (MapSize.X == 0 && MapSize.Y == 0 && MapSize.Z == 0) // MapSize가 초기화되지 않았으면 반환
	{
		return;
	}


	const FVector SingleVoxelHalfExtent(VoxelExtent * 0.5f);

	for (int32 z = 0; z < MapSize.Z; ++z)
	{
		for (int32 y = 0; y < MapSize.Y; ++y)
		{
			for (int32 x = 0; x < MapSize.X; ++x)
			{
				const FIntVector CurrentVoxelIndex(x, y, z);
				if (IsVoxelWalkable(CurrentVoxelIndex) == false) // 장애물인 복셀만 그립니다.
				{
					const FVector VoxelWorldCenter = VoxelIndexToWorld(CurrentVoxelIndex);
					DrawDebugBox(
						World,
						VoxelWorldCenter,
						SingleVoxelHalfExtent,
						GetActorQuat(), // 복셀 볼륨의 회전을 적용
						DebugObstacleColor,
						false, // Persistent Lines: false로 하면 매 프레임 그려야 함 (Tick에서 호출 시 적합)
						-1.0f, // LifeTime: -1은 한 프레임만. Tick에서 매번 그릴 것이므로. 0.0f도 가능.
						0,     // Depth Priority
						DebugObstacleThickness
					);
				}
			}
		}
	}
}