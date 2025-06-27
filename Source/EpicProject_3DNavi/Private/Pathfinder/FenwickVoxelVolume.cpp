// Fill out your copyright notice in the Description page of Project Settings.

#include "Pathfinder/FenwickVoxelVolume.h"

#include "EpicProject_3DNavi/Voxel/DynamicObstacle.h"
#include "Kismet/GameplayStatics.h"


// Sets default values
AFenwickVoxelVolume::AFenwickVoxelVolume()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void AFenwickVoxelVolume::BeginPlay()
{
	Super::BeginPlay();

	FenwickTreeSize = MapSize + FIntVector(1, 1, 1);

	const int32 TotalSize = FenwickTreeSize.X * FenwickTreeSize.Y * FenwickTreeSize.Z;

	FenwickTree.Empty(TotalSize);
	FenwickTree.AddZeroed(TotalSize);

	BuildStaticObstacles();
}


// Called every frame
void AFenwickVoxelVolume::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	
}

void AFenwickVoxelVolume::BuildStaticObstacles()
{
	FCollisionObjectQueryParams StaticObjectQueryParams;
	StaticObjectQueryParams.AddObjectTypesToQuery(ECC_WorldStatic); // 정적 장애물 검색
	StaticObstacleVoxels.Empty();
	TArray<FOverlapResult> OverlapResults;
	if (!GetOverlappingComponentsInVolume(StaticObjectQueryParams, OverlapResults))
	{
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
						StaticObstacleVoxels.Add(CurrentVoxelIndex);
						FenwUpdate(CurrentVoxelIndex, 1);
					}
				}
			}
		}
	}
}

bool AFenwickVoxelVolume::IsRegionWalkable(const FIntVector& VoxelIndex, int32 AgentRadiusInVoxels)
{
	if (!IsVoxelWalkable(VoxelIndex)) return false;

	if (AgentRadiusInVoxels < 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("IsRegionWalkable FAILED: AgentRadiusInVoxels (%d) is negative."),
		       AgentRadiusInVoxels);
		return false;
	}

	const FIntVector MinCorner = VoxelIndex - FIntVector(AgentRadiusInVoxels);
	const FIntVector MaxCorner = VoxelIndex + FIntVector(AgentRadiusInVoxels);


	if (MinCorner.X < 0 || MinCorner.Y < 0 || MinCorner.Z <= 0 ||
		MaxCorner.X >= MapSize.X || MaxCorner.Y >= MapSize.Y || MaxCorner.Z >= MapSize.Z)
	{
		return false;
	}

	const int32 ObstacleCount = FenwRangeSum(MinCorner, MaxCorner);
	return ObstacleCount == 0;
}

bool AFenwickVoxelVolume::IsPathSegmentWalkable(const FIntVector& FromStartIndex, const FIntVector& ToStartIndex,
	int32 AgentRadiusInVoxels) const
{
	// 에이전트가 From에서 To로 이동할 때 쓸고 지나가는 전체 영역(Swept Volume)의 바운딩 박스를 계산합니다.
	
	// From 지점의 에이전트 바운딩 박스
	const FIntVector FromMin = FromStartIndex - FIntVector(AgentRadiusInVoxels);
	const FIntVector FromMax = FromStartIndex + FIntVector(AgentRadiusInVoxels);

	// To 지점의 에이전트 바운딩 박스
	const FIntVector ToMin = ToStartIndex - FIntVector(AgentRadiusInVoxels);
	const FIntVector ToMax = ToStartIndex + FIntVector(AgentRadiusInVoxels);

	// 두 바운딩 박스를 모두 포함하는 최종 Swept Volume의 바운딩 박스
	const FIntVector SweptMin(FMath::Min(FromMin.X, ToMin.X), FMath::Min(FromMin.Y, ToMin.Y), FMath::Min(FromMin.Z, ToMin.Z));
	const FIntVector SweptMax(FMath::Max(FromMax.X, ToMax.X), FMath::Max(FromMax.Y, ToMax.Y), FMath::Max(FromMax.Z, ToMax.Z));

	// 이 거대한 영역에 장애물이 하나라도 있으면 이동 불가.
	// FenwRangeSum을 사용하여 단 한 번의 쿼리로 매우 빠르게 검사합니다.
	if (FenwRangeSum(SweptMin, SweptMax) > 0)
	{
		return false;
	}

	return true;
}

void AFenwickVoxelVolume::UpdateDynamicObstacles()
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
			if (!OverlappedActor || !OverlappedComponent || !OverlappedComponent->IsQueryCollisionEnabled() || !OverlappedActor->GetComponentByClass<UDynamicObstacle>())
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
							CurrentFrameVoxelCounts.FindOrAdd(CurrentVoxelIndex, 0)++;
						}
					}
				}
			}
		}
	}

	// --- 2. 이전 프레임과 현재 프레임을 비교하여 '실제로 변경된' 복셀만 업데이트 ---

	// 2-1. '카운트가 변경'되었거나 '사라진' 장애물 처리
	for (const auto& Pair : DynamicObstacleVoxelCounts)
	{
		const FIntVector& Index = Pair.Key;
		const int32 PreviousCount = Pair.Value;
		
		// 현재 프레임의 카운트를 가져옵니다. 키가 없으면 0을 반환합니다.
		const int32 CurrentCount = CurrentFrameVoxelCounts.FindRef(Index);

		// 카운트에 변화가 있을 때만 업데이트를 수행합니다. (핵심 최적화)
		if (CurrentCount != PreviousCount)
		{
			const int32 Delta = CurrentCount - PreviousCount;
			FenwUpdate(Index, Delta);

			// 이전에는 장애물이었는데(PreviousCount > 0), 이제는 아니다(CurrentCount == 0) -> 장애물 상태를 제거
			if (PreviousCount > 0 && CurrentCount == 0)
			{
				SetStaticObstacle(Index, false);
			}
		}
	}

	// 2-2. '새로 생긴' 장애물 처리: 지난 프레임에는 없었지만, 현재 프레임에는 있는 복셀
	for (const auto& Pair : CurrentFrameVoxelCounts)
	{
		const FIntVector& Index = Pair.Key;
		const int32 CurrentCount = Pair.Value;
		
		// 지난 프레임의 맵에 이 키가 없었다면, 이 복셀은 완전히 새로운 동적 장애물입니다.
		if (!DynamicObstacleVoxelCounts.Contains(Index))
		{
			// 펜윅 트리에 새로운 카운트만큼 더해주고, 장애물 상태로 설정합니다.
			FenwUpdate(Index, CurrentCount);
			SetStaticObstacle(Index, true);
		}
	}

	// --- 3. 다음 프레임을 위해 현재 상태를 저장 ---
	DynamicObstacleVoxelCounts = MoveTemp(CurrentFrameVoxelCounts);
}

void AFenwickVoxelVolume::FenwUpdate(const FIntVector& Index, int32 Delta)
{
	for (int32 z = Index.Z + 1; z < FenwickTreeSize.Z; z += z & -z)
	{
		for (int32 y = Index.Y + 1; y < FenwickTreeSize.Y; y += y & -y)
		{
			for (int32 x = Index.X + 1; x < FenwickTreeSize.X; x += x & -x)
			{
				// 현재 위치(x,y,z)의 1차원 배열 인덱스를 계산하여 값을 업데이트합니다.
				FenwickTree[GetFenwickFlatIndex(x, y, z)] += Delta;
			}
		}
	}
}

// AFenwickVoxelVolume.cpp - 올바른 코드
int32 AFenwickVoxelVolume::FenwQuery(const FIntVector& Index) const
{
	int32 Sum = 0;
	// Stride 계산은 그대로 유지합니다.
	const int32 ZStride = FenwickTreeSize.Y * FenwickTreeSize.X;
	const int32 YStride = FenwickTreeSize.X;

	for (int32 z = Index.Z + 1; z > 0; z -= z & -z)
	{
		const int32 ZOffset = z * ZStride;
		for (int32 y = Index.Y + 1; y > 0; y -= y & -y)
		{
			const int32 YOffset = y * YStride;
			for (int32 x = Index.X + 1; x > 0; x -= x & -x)
			{
				Sum += FenwickTree[ZOffset + YOffset + x];
			}
		}
	}
	return Sum;
}

int32 AFenwickVoxelVolume::FenwRangeSum(const FIntVector& MinCorner, const FIntVector& MaxCorner) const
{
	// 펜윅 트리는 1-based index를 사용하므로, 쿼리 좌표도 조정합니다.
	// MinCorner(x,y,z)는 쿼리 시 x,y,z가 되고, MaxCorner는 x+1, y+1, z+1이 됩니다.
	const int32 x0 = MinCorner.X;
	const int32 y0 = MinCorner.Y;
	const int32 z0 = MinCorner.Z;
	const int32 x1 = MaxCorner.X + 1;
	const int32 y1 = MaxCorner.Y + 1;
	const int32 z1 = MaxCorner.Z + 1;

	int32 TotalSum = 0;

	// Z1에 대한 2D Range Sum 계산
	for (int32 z = z1; z > 0; z -= z & -z)
	{
		int32 YSum = 0;
		// Y1에 대한 1D Range Sum
		for (int32 y = y1; y > 0; y -= y & -y)
		{
			// X1 - X0
			for (int32 x = x1; x > 0; x -= x & -x) { YSum += FenwickTree[GetFenwickFlatIndex(x, y, z)]; }
			for (int32 x = x0; x > 0; x -= x & -x) { YSum -= FenwickTree[GetFenwickFlatIndex(x, y, z)]; }
		}
		// Y0에 대한 1D Range Sum
		for (int32 y = y0; y > 0; y -= y & -y)
		{
			// -(X1 - X0)
			for (int32 x = x1; x > 0; x -= x & -x) { YSum -= FenwickTree[GetFenwickFlatIndex(x, y, z)]; }
			for (int32 x = x0; x > 0; x -= x & -x) { YSum += FenwickTree[GetFenwickFlatIndex(x, y, z)]; }
		}
		TotalSum += YSum;
	}

	// Z0에 대한 2D Range Sum 계산 (전체 부호가 반대)
	for (int32 z = z0; z > 0; z -= z & -z)
	{
		int32 YSum = 0;
		// Y1에 대한 1D Range Sum
		for (int32 y = y1; y > 0; y -= y & -y)
		{
			// X1 - X0
			for (int32 x = x1; x > 0; x -= x & -x) { YSum += FenwickTree[GetFenwickFlatIndex(x, y, z)]; }
			for (int32 x = x0; x > 0; x -= x & -x) { YSum -= FenwickTree[GetFenwickFlatIndex(x, y, z)]; }
		}
		// Y0에 대한 1D Range Sum
		for (int32 y = y0; y > 0; y -= y & -y)
		{
			// -(X1 - X0)
			for (int32 x = x1; x > 0; x -= x & -x) { YSum -= FenwickTree[GetFenwickFlatIndex(x, y, z)]; }
			for (int32 x = x0; x > 0; x -= x & -x) { YSum += FenwickTree[GetFenwickFlatIndex(x, y, z)]; }
		}
		TotalSum -= YSum; // Z0에 대한 결과는 빼줍니다.
	}

	return TotalSum;
}


int32 AFenwickVoxelVolume::GetFenwickFlatIndex(int32 x, int32 y, int32 z) const
{
	return (z * FenwickTreeSize.Y * FenwickTreeSize.X) + (y * FenwickTreeSize.X) + x;
}
