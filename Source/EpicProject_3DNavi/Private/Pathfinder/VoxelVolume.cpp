// Fill out your copyright notice in the Description page of Project Settings.


#include "Pathfinder/VoxelVolume.h"

#include "EpicProject_3DNavi/Voxel/DynamicObstacle.h"

#include "Engine/LocalPlayer.h"
#include "GameFramework/PlayerController.h"
#include "SceneView.h"
#include "Kismet/GameplayStatics.h"

bool ProjectWorldToScreen_ThreadSafe(const FVector& WorldLocation, const FMatrix& ViewProjectionMatrix,
                                     FVector2D& OutScreenLocation)
{
	const FVector4 ClipSpacePos = ViewProjectionMatrix.TransformFVector4(FVector4(WorldLocation, 1.0f));

	if (ClipSpacePos.W <= 0.0f)
	{
		return false; // 카메라 뒤에 있음
	}

	const FVector2D NormalizedScreenPos = FVector2D(ClipSpacePos.X / ClipSpacePos.W, ClipSpacePos.Y / ClipSpacePos.W);

	// NDC(-1 to 1) to Screen(0 to 1)
	OutScreenLocation.X = (NormalizedScreenPos.X / 2.0f) + 0.5f;
	OutScreenLocation.Y = 1.0f - ((NormalizedScreenPos.Y / 2.0f) + 0.5f);

	return true;
}


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

void AVoxelVolume::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	Super::EndPlay(EndPlayReason);
	if (CullingTaskFuture.IsValid())
	{
		CullingTaskFuture.Wait();
	}
}

void AVoxelVolume::Tick(float DeltaTime)
{
	UpdateDynamicObstacles();

	// 1. 이전 프레임의 비동기 작업이 완료되었는지 확인합니다.
	if (CullingTaskFuture.IsValid() && CullingTaskFuture.IsReady())
	{
		// 2. 완료되었다면, 계산 결과를 가져와 멤버 변수를 업데이트합니다.
		// Get()은 결과를 한 번만 가져올 수 있습니다.
		TSharedPtr<FDebugBoxCullingResult> NewResult = CullingTaskFuture.Get();
		if (NewResult.IsValid())
		{
			LastCompletedCullingResult = NewResult;
		}
		CullingTaskFuture.Reset(); // Future를 리셋하여 다음 작업을 받을 준비를 합니다.
	}

	// 3. 현재 실행 중인 작업이 없다면, 새로운 비동기 작업을 시작합니다.
	if (!CullingTaskFuture.IsValid())
	{
		StartAsyncCullingTask();
	}

	// 4. 실제 디버그 그리기는 매 프레임 호출됩니다.
	// 이 함수는 이제 계산을 하지 않고, LastCompletedCullingResult에 담긴 '지난 프레임의 결과'를 그리기만 합니다.
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
						StaticObstacleVoxels.Add(CurrentVoxelIndex);
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

	// --- 1. 현재 프레임의 동적 장애물 상태 계산 ---
	// 이번 프레임에 각 복셀을 점유한 장애물의 수를 계산하여 TMap에 저장합니다.
	TMap<FIntVector, int32> CurrentFrameVoxelCounts;

	// 물리 쿼리를 위한 파라미터 설정
	FCollisionObjectQueryParams DynamicObjectQueryParams;
	DynamicObjectQueryParams.AddObjectTypesToQuery(ECC_Pawn);
	DynamicObjectQueryParams.AddObjectTypesToQuery(ECC_WorldDynamic);
	DynamicObjectQueryParams.AddObjectTypesToQuery(ECC_PhysicsBody);

	// 볼륨 내의 모든 동적 장애물 컴포넌트를 찾습니다.
	TArray<FOverlapResult> OverlapResults;
	if (GetOverlappingComponentsInVolume(DynamicObjectQueryParams, OverlapResults))
	{
		for (const FOverlapResult& Result : OverlapResults)
		{
			AActor* OverlappedActor = Result.GetActor();
			UPrimitiveComponent* OverlappedComponent = Result.GetComponent();

			// 유효성 검사: UDynamicObstacle 컴포넌트가 있는지, 콜리전이 활성화되어 있는지 확인
			if (!OverlappedActor || !OverlappedComponent || !OverlappedComponent->IsQueryCollisionEnabled() || !
				OverlappedActor->GetComponentByClass<UDynamicObstacle>())
			{
				continue;
			}

			// 장애물의 바운딩 박스 계산
			const FBox ObstacleCompBounds = OverlappedComponent->Bounds.GetBox();
			FIntVector MinVoxelIndex = WorldToVoxelIndex(ObstacleCompBounds.Min);
			FIntVector MaxVoxelIndex = WorldToVoxelIndex(ObstacleCompBounds.Max - FVector(KINDA_SMALL_NUMBER));

			if (!ClampVoxelIndicesToVolume(MinVoxelIndex, MaxVoxelIndex))
			{
				continue;
			}

			// 장애물이 점유하는 모든 복셀에 대해...
			for (int32 z = MinVoxelIndex.Z; z <= MaxVoxelIndex.Z; ++z)
			{
				for (int32 y = MinVoxelIndex.Y; y <= MaxVoxelIndex.Y; ++y)
				{
					for (int32 x = MinVoxelIndex.X; x <= MaxVoxelIndex.X; ++x)
					{
						const FIntVector CurrentVoxelIndex(x, y, z);
						if (CheckVoxelOverlap(CurrentVoxelIndex, OverlappedComponent))
						{
							// 해당 복셀의 카운트를 1 증가시킵니다.
							// 키가 없으면 0으로 자동 추가된 후 1이 됩니다.
							CurrentFrameVoxelCounts.FindOrAdd(CurrentVoxelIndex, 0)++;
						}
					}
				}
			}
		}
	}

	// --- 2. 이전 프레임과 현재 프레임을 비교하여 '실제로 변경된' 복셀만 업데이트 ---

	// 2-1. '사라진' 장애물 처리: 지난 프레임에는 있었지만, 현재 프레임에는 없는 복셀
	for (const auto& Pair : DynamicObstacleVoxelCounts)
	{
		const FIntVector& Index = Pair.Key;
		const int32 PreviousCount = Pair.Value;

		// 현재 프레임의 카운트를 가져옵니다. 키가 없으면 0을 반환합니다.
		const int32 CurrentCount = CurrentFrameVoxelCounts.FindRef(Index);

		// 이전에는 장애물이었는데(PreviousCount > 0), 이제는 아니다(CurrentCount == 0) -> 장애물 상태를 제거
		if (PreviousCount > 0 && CurrentCount == 0)
		{
			SetStaticObstacle(Index, false);
		}
	}

	// 2-2. '새로 생긴' 장애물 처리: 지난 프레임에는 없었지만, 현재 프레임에는 있는 복셀
	for (const auto& Pair : CurrentFrameVoxelCounts)
	{
		const FIntVector& Index = Pair.Key;

		// 지난 프레임의 맵에 이 키가 없었다면, 이 복셀은 완전히 새로운 동적 장애물입니다.
		if (!DynamicObstacleVoxelCounts.Contains(Index))
		{
			// 장애물 상태로 설정합니다.
			SetStaticObstacle(Index, true);
		}
	}

	// --- 3. 다음 프레임을 위해 현재 상태를 저장 ---
	// MoveTemp를 사용하여 불필요한 데이터 복사를 피하고 소유권을 이전합니다.
	DynamicObstacleVoxelCounts = MoveTemp(CurrentFrameVoxelCounts);
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

bool AVoxelVolume::IsVoxelVisible(const UWorld* World, const FVector& ViewLocation, const FVector& VoxelWorldPos,
                                  const FVector& VoxelBoxExtent,
                                  const FCollisionQueryParams& OcclusionParams) const
{
	// --- 1단계: 빠른 경로 (Fast Path) ---
	// 대부분의 복셀은 완전히 보이거나 완전히 가려져 있으므로, 가장 먼저 중심점을 검사합니다.
	// 이 검사만으로도 대다수의 경우를 빠르게 처리할 수 있습니다.
	FHitResult HitResult;
	if (!World->LineTraceSingleByObjectType(
		HitResult,
		ViewLocation,
		VoxelWorldPos,
		FCollisionObjectQueryParams(ECC_WorldStatic), // 'WorldStatic' 타입의 오브젝트만 장애물로 간주
		OcclusionParams
	))
	{
		return true;
	}

	// --- 2단계: 정확한 경로 (Accurate Path) ---
	for (int32 i = 0; i < 8; ++i)
	{
		const FVector CornerDirection(
			(i & 1) ? 1.f : -1.f,
			(i & 2) ? 1.f : -1.f,
			(i & 4) ? 1.f : -1.f
		);

		// 복셀 중심에서 꼭짓점까지의 최종 월드 좌표를 계산합니다.
		const FVector CornerWorldPos = VoxelWorldPos + CornerDirection * VoxelBoxExtent;

		// 각 꼭짓점으로 라인 트레이스를 수행합니다.
		if (!World->LineTraceSingleByObjectType(
			HitResult,
			ViewLocation,
			CornerWorldPos,
			FCollisionObjectQueryParams(ECC_WorldStatic),
			OcclusionParams
		))
		{
			return true;
		}
	}

	return false;
}

void AVoxelVolume::StartAsyncCullingTask()
{
	UWorld* World = GetWorld();
	if (IsValid(World) == false) return;
	APlayerController* PC = UGameplayStatics::GetPlayerController(World, 0);
	if (!PC) return;


	// --- 1. 게임 스레드에서 모든 필요한 데이터를 준비합니다. ---
	FAsyncCullingTaskParams Params;

	// 카메라의 View-Projection Matrix를 가져옵니다. 이것이 ProjectWorldToScreen의 핵심입니다.
	FSceneViewProjectionData ProjectionData;
	if (PC->GetLocalPlayer()->GetProjectionData(PC->GetLocalPlayer()->ViewportClient->Viewport, ProjectionData))
	{
		Params.ViewProjectionMatrix = ProjectionData.ComputeViewProjectionMatrix();
	}
	else
	{
		// 데이터를 가져올 수 없으면 작업을 시작하지 않습니다.
		return;
	}

	// 복셀 데이터 복사
	Params.StaticVoxels = StaticObstacleVoxels;
	DynamicObstacleVoxelCounts.GetKeys(Params.DynamicVoxels);

	// 액터 및 복셀 정보 복사
	Params.ActorToWorldTransform = GetActorTransform();
	Params.VoxelExtent = VoxelExtent;

	int32 ViewportSizeX, ViewportSizeY;
	PC->GetViewportSize(ViewportSizeX, ViewportSizeY);

	// 복셀 데이터도 복사합니다.
	TSet<FIntVector> StaticVoxelsCopy = StaticObstacleVoxels;
	TArray<FIntVector> DynamicVoxelsCopy;
	DynamicObstacleVoxelCounts.GetKeys(DynamicVoxelsCopy);

	FVector VoxelSizeCopy = FVector(VoxelExtent * 0.5);
	const FTransform ActorTransformCopy = GetActorTransform();

	FCollisionQueryParams OcclusionParams;
	OcclusionParams.bTraceComplex = false;
	OcclusionParams.AddIgnoredActor(this);
	
	if (PC->GetPawn())
	{
		OcclusionParams.AddIgnoredActor(PC->GetPawn());
	}

	// --- 비동기 작업(Task)을 스레드 풀(ThreadPool)에 보냅니다. ---
	CullingTaskFuture = Async(EAsyncExecution::ThreadPool,
	                          [Params, ViewportSizeX, ViewportSizeY]() -> TSharedPtr<FDebugBoxCullingResult>
	                          {
		                          TSharedPtr<FDebugBoxCullingResult> Result = MakeShared<FDebugBoxCullingResult>();
		                          const FVector VoxelBoxHalfExtent = FVector(Params.VoxelExtent * 0.5f);

		                          // --- 정적 복셀 계산 ---
		                          for (const FIntVector& VoxelIndex : Params.StaticVoxels)
		                          {
			                          const FVector VoxelCenterLocal = FVector(VoxelIndex) * Params.VoxelExtent +
				                          VoxelBoxHalfExtent;
			                          const FVector VoxelWorldPos = Params.ActorToWorldTransform.TransformPosition(
				                          VoxelCenterLocal);

			                          FVector2D ScreenLocationRatio; // 0~1 사이의 비율 값
			                          if (ProjectWorldToScreen_ThreadSafe(
				                          VoxelWorldPos, Params.ViewProjectionMatrix, ScreenLocationRatio))
			                          {
				                          // 화면 비율을 실제 픽셀 좌표로 변환하여 화면 내에 있는지 확인
				                          const float ScreenX = ScreenLocationRatio.X * ViewportSizeX;
				                          const float ScreenY = ScreenLocationRatio.Y * ViewportSizeY;

				                          if (ScreenX >= 0 && ScreenX <= ViewportSizeX && ScreenY >= 0 && ScreenY <=
					                          ViewportSizeY)
				                          {
					                          // 참고: IsVoxelVisible (라인 트레이스)는 여기서 호출할 수 없습니다.
					                          // 스레드 안전한 Occlusion Culling은 훨씬 복잡한 기술이 필요합니다.
					                          // 일단은 Frustum Culling (화면 안에 보이는지)만 수행합니다.
					                          Result->VisibleStaticVoxels.Add(VoxelWorldPos);
				                          }
			                          }
		                          }
		                          for (const FIntVector& VoxelIndex : Params.DynamicVoxels)
		                          {
			                          const FVector VoxelCenterLocal = FVector(VoxelIndex) * Params.VoxelExtent +
				                          VoxelBoxHalfExtent;
			                          const FVector VoxelWorldPos = Params.ActorToWorldTransform.TransformPosition(
				                          VoxelCenterLocal);

			                          FVector2D ScreenLocationRatio;
			                          if (ProjectWorldToScreen_ThreadSafe(
				                          VoxelWorldPos, Params.ViewProjectionMatrix, ScreenLocationRatio))
			                          {
				                          const float ScreenX = ScreenLocationRatio.X * ViewportSizeX;
				                          const float ScreenY = ScreenLocationRatio.Y * ViewportSizeY;

				                          if (ScreenX >= 0 && ScreenX <= ViewportSizeX && ScreenY >= 0 && ScreenY <=
					                          ViewportSizeY)
				                          {
					                          Result->VisibleDynamicVoxels.Add(VoxelWorldPos);
				                          }
			                          }
		                          }

		                          return Result;
	                          });
}

void AVoxelVolume::DrawDebugVoxelObstacles() const
{
	if (LastCompletedCullingResult.IsValid())
	{
		UWorld* World = GetWorld();
		const FVector VoxelBoxExtent = FVector(VoxelExtent * 0.5);;

		// 계산된 정적 복셀 그리기
		for (const FVector& WorldPos : LastCompletedCullingResult->VisibleStaticVoxels)
		{
			DrawDebugBox(World, WorldPos, VoxelBoxExtent, FColor::Red, false, -1.f, 0, 1.0f);
		}

		// 계산된 동적 복셀 그리기
		for (const FVector& WorldPos : LastCompletedCullingResult->VisibleDynamicVoxels)
		{
			DrawDebugBox(World, WorldPos, VoxelBoxExtent, FColor::Yellow, false, -1.f, 0, 2.0f);
		}
	}
}
