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
		UE_LOG(LogTemp, Error,
		       TEXT(
			       "UpdateDynamicObstacles: No dynamic components found in volume via GetOverlappingComponentsInVolume."
		       ));
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

#include "Async/Async.h"

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
	if (!World || (MapSize.X == 0 && MapSize.Y == 0 && MapSize.Z == 0) || VoxelExtent <= 0.f)
	{
		return;
	}
	// 람다에서 사용할 멤버 변수들을 값으로 캡처합니다.
	const FIntVector CurrentMapSize = MapSize;
	const float CurrentVoxelExtent = VoxelExtent;
	const FColor CurrentDebugObstacleColor = DebugObstacleColor;
	const float CurrentDebugObstacleThickness = DebugObstacleThickness;

	// 액터와 월드의 약한 포인터를 캡처합니다.
	TWeakObjectPtr<const AVoxelVolume> WeakThis(this);
	TWeakObjectPtr<UWorld> WeakWorld(World);

	// 비동기 작업을 백그라운드 스레드에서 실행합니다.
	AsyncTask(ENamedThreads::AnyBackgroundThreadNormalTask,
	          [WeakThis, WeakWorld, CurrentMapSize, CurrentVoxelExtent, CurrentDebugObstacleColor,
		          CurrentDebugObstacleThickness]()
	          {
		          if (!WeakThis.IsValid()) // VoxelVolume 액터가 유효한지 확인
		          {
			          return;
		          }
		          TArray<FIntVector> ObstacleIndices;
		          ObstacleIndices.Reserve(CurrentMapSize.X * CurrentMapSize.Y);

		          for (int32 z = 0; z < CurrentMapSize.Z; ++z)
		          {
			          for (int32 y = 0; y < CurrentMapSize.Y; ++y)
			          {
				          
				          const int32 ChunksAlongX = CurrentMapSize.X / CHUNK_SIZE;
				          for (int32 ChunkX_idx = 0; ChunkX_idx < ChunksAlongX; ++ChunkX_idx)
				          {
					          const AVoxelVolume* StrongThis = WeakThis.Get();
					          uint64 ObstacleChunkBits = StrongThis->GetObstacleChunk_X(ChunkX_idx, y, z);

					          if (ObstacleChunkBits == 0)
					          {
						          continue;
					          }

					      
					          for (int32 BitPos = 0; BitPos < CHUNK_SIZE; ++BitPos)
					          {
						          if ((ObstacleChunkBits & (1ULL << BitPos)) != 0) // BitPos 번째 비트가 1이면 장애물
						          {
							          const int32 x = ChunkX_idx * CHUNK_SIZE + BitPos;
							          ObstacleIndices.Add(FIntVector(x, y, z));
						          }
					          }
				          }
			          }
		          }

		          if (ObstacleIndices.Num() == 0)
		          {
			          return; // 그릴 장애물이 없으면 종료
		          }

		          // 2. 게임 스레드: 수집된 인덱스를 기반으로 실제 그리기 작업 예약
		          // 게임 스레드 람다 내부
		          AsyncTask(ENamedThreads::GameThread,
		                    [WeakThis, WeakWorld, ObstacleIndices, CurrentVoxelExtent, CurrentDebugObstacleColor,
			                    CurrentDebugObstacleThickness]()
		                    {
			                    if (!WeakWorld.IsValid() || !WeakThis.IsValid()) return;
			                    UWorld* StrongWorld = WeakWorld.Get();
			                    const AVoxelVolume* StrongThis = WeakThis.Get();

			                    APlayerController* PC = StrongWorld->GetFirstPlayerController();
			                    if (!PC) return;

			                    FVector CameraLocation;
			                    FRotator CameraRotation;
			                    PC->GetPlayerViewPoint(CameraLocation, CameraRotation);
			                    FVector CameraForward = CameraRotation.Vector();

			                    const FVector SingleVoxelHalfExtent(CurrentVoxelExtent * 0.5f);
			                    const FQuat ActorQuat = StrongThis->GetActorQuat();

			                    // 레이캐스트를 위한 콜리전 파라미터 설정
			                    FCollisionQueryParams CollisionParams;
			                    CollisionParams.AddIgnoredActor(PC->GetPawn()); // 플레이어 폰은 무시 (선택적)
			                    CollisionParams.AddIgnoredActor(StrongThis); // VoxelVolume 자체도 무시

			                    const float MinDist = CurrentVoxelExtent * 8.5f;
			                    for (const FIntVector& VoxelIndex : ObstacleIndices)
			                    {
				                    const FVector VoxelWorldCenter = StrongThis->VoxelIndexToWorld(VoxelIndex);

				                    // 1. 거리 기반 필터링 (필수)
				                    float DistSq = FVector::DistSquared(VoxelWorldCenter, CameraLocation);

				                    if (DistSq < FMath::Square(MinDist))
				                    {
					                    FVector Center = StrongThis->VoxelIndexToWorld(VoxelIndex);

					                    DrawDebugBox(StrongWorld, Center, SingleVoxelHalfExtent, ActorQuat,
					                                 CurrentDebugObstacleColor, false, -1.f, 0,
					                                 CurrentDebugObstacleThickness);
					                    continue;
				                    }

				                    FVector DirToVoxel = (VoxelWorldCenter - CameraLocation);
				                    float ActualDistanceToVoxel = DirToVoxel.Size(); // 실제 거리
				                    DirToVoxel.Normalize();
				                    float FOV = PC->PlayerCameraManager->GetFOVAngle();
				                    float CosHalfFov = FMath::Cos(FMath::DegreesToRadians(FOV * 0.5f));

				                    if (FVector::DotProduct(CameraForward, DirToVoxel) < CosHalfFov)
				                    {
					                    continue;
				                    }


				                    // 3. 레이캐스트 기반 오클루전 테스트
				                    FHitResult HitResult;
				                    FVector RayStart = CameraLocation; // + DirToVoxel * (CurrentVoxelExtent * 0.1f);
				                    FVector RayEnd = VoxelWorldCenter;

				                    bool bHit = StrongWorld->LineTraceSingleByChannel(
					                    HitResult,
					                    RayStart, // 시작점
					                    RayEnd, // 끝점
					                    ECC_Visibility, // 가시성 채널 사용 (또는 ECC_WorldStatic 등)
					                    CollisionParams
				                    );

				                    if (bHit)
				                    {
					                    // 무언가에 부딪혔다면
					                    // 그 물체가 복셀보다 카메라에 더 가까운지 확인
					                    // (VoxelWorldCenter - HitResult.ImpactPoint).IsNearlyZero(VoxelExtent * 0.1f) 등으로 복셀 자체 충돌 허용 가능
					                    if ((HitResult.ImpactPoint - CameraLocation).SizeSquared() < (
						                    ActualDistanceToVoxel - SingleVoxelHalfExtent.X) * (ActualDistanceToVoxel -
						                    SingleVoxelHalfExtent.X) * 0.9f) // 복셀 표면보다 약간 안쪽과 비교
					                    {
						                    // 복셀에 도달하기 "전에" 다른 물체에 막혔음
						                    continue;
					                    }
					                    // 만약 복셀 자체가 콜리전이 있다면, HitResult.GetActor() == StrongThis 이고 HitResult.Item (복셀인덱스) 등으로 구분해야하는데,
					                    // DrawDebugBox는 콜리전이 없으므로 이 방식은 어려움.
					                    // 여기서는 단순히 "복셀에 도달하기 전에 다른 것에 막히면 컬링"
				                    }


				                    DrawDebugBox(
					                    StrongWorld,
					                    VoxelWorldCenter,
					                    SingleVoxelHalfExtent,
					                    ActorQuat,
					                    CurrentDebugObstacleColor,
					                    false,
					                    -1.0f,
					                    0,
					                    CurrentDebugObstacleThickness
				                    );
			                    }
		                    });
	          });
}
