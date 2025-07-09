// Fill out your copyright notice in the Description page of Project Settings.


#include "UniformGridVolume.h"

#include "Components/InstancedStaticMeshComponent.h"

// Sets default values
AUniformGridVolume::AUniformGridVolume()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	if (!GetRootComponent())
	{
		SetRootComponent(CreateDefaultSubobject<USceneComponent>(TEXT("DefaultSceneRoot")));
	}

	// 볼륨 범위 시각화용 Static Mesh 컴포넌트 생성
	VolumeBoundsVisualizer = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("VolumeBoundsVisualizer"));
	static ConstructorHelpers::FObjectFinder<UStaticMesh> DebugMeshFinder(
		TEXT("StaticMesh'/Game/PathfinderPlugin/Debug/SM_DebugVoxel.SM_DebugVoxel'"));
	if (DebugMeshFinder.Succeeded())
	{
		VolumeBoundsVisualizer->SetStaticMesh(DebugMeshFinder.Object);
	}

	VolumeBoundsVisualizer->SetupAttachment(GetRootComponent());
	VolumeBoundsVisualizer->SetCollisionEnabled(ECollisionEnabled::NoCollision);

	// Set as default
	MapSize = FIntVector(64, 64, 64);
	VoxelExtent = 100.f;
}

// Called when the game starts or when spawned
void AUniformGridVolume::BeginPlay()
{
	Super::BeginPlay();
}

void AUniformGridVolume::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);

	if (VolumeBoundsVisualizer)
	{
		const FVector TotalVolumeDimensions = FVector(MapSize) * VoxelExtent;

		// 2. 메시의 위치를 볼륨의 정중앙으로 설정합니다.
		// 액터 위치가 볼륨의 (0,0,0) 모서리이므로, 중심은 전체 크기의 절반만큼 이동해야 합니다.
		const FVector LocalCenterOffset = TotalVolumeDimensions * 0.5f;
		VolumeBoundsVisualizer->SetRelativeLocation(LocalCenterOffset);

		// 3. 메시의 크기를 조절합니다.
		if (TotalVolumeDimensions.IsNearlyZero() || VoxelExtent <= 0.f)
		{
			VolumeBoundsVisualizer->SetRelativeScale3D(FVector::ZeroVector);
		}
		else
		{
			const FVector Scale = TotalVolumeDimensions / 100.0f;
			VolumeBoundsVisualizer->SetRelativeScale3D(Scale);
		}
	}
	UpdateDebugObstacleVisualization();
}

// Called every frame
void AUniformGridVolume::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

FVector AUniformGridVolume::GetLocationFromIndex(
	const FIntVector& VoxelIndex) const
{
	return Super::GetLocationFromIndex(VoxelIndex);
}

FIntVector AUniformGridVolume::GetIndexFromLocation(
	const FVector& WorldPosition) const
{
	return Super::GetIndexFromLocation(WorldPosition);
}

FIntVector AUniformGridVolume::WorldToVoxelIndex(const FVector& WorldPosition) const
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

FVector AUniformGridVolume::VoxelIndexToWorld(const FIntVector& VoxelIndex) const
{
	const FVector LocalCenter = FVector(VoxelIndex) * VoxelExtent + (VoxelExtent * 0.5f);
	return GetActorTransform().TransformPosition(LocalCenter);
}

bool AUniformGridVolume::IsIndexValid(const FIntVector& Index) const
{
	return Index.X >= 0 && Index.X < MapSize.X &&
		Index.Y >= 0 && Index.Y < MapSize.Y &&
		Index.Z >= 0 && Index.Z < MapSize.Z;
}

bool AUniformGridVolume::ClampVoxelIndicesToVolume(FIntVector& MinVoxelIndex, FIntVector& MaxVoxelIndex) const
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

bool AUniformGridVolume::GetOverlappingComponentsInVolume(const FCollisionObjectQueryParams& ObjectQueryParams,
                                                          TArray<FOverlapResult>& OutResults) const
{
	OutResults.Empty();
	UWorld* World = GetWorld();
	if (World == nullptr)
	{
		return false;
	}

	const FTransform ActorTransform = GetActorTransform();
	const FVector TotalVolumeDimensions = FVector(MapSize) * VoxelExtent;
	const FVector VolumeHalfExtent = TotalVolumeDimensions * 0.5f;

	// 볼륨의 로컬 중심 좌표를 월드 좌표로 변환하여 쿼리 원점을 계산
	const FVector VolumeLocalCenter = VolumeHalfExtent;
	const FVector QueryWorldOrigin = ActorTransform.TransformPosition(VolumeLocalCenter);
	const FQuat QueryWorldRotation = ActorTransform.GetRotation();

	FCollisionShape VolumeQueryShape = FCollisionShape::MakeBox(VolumeHalfExtent);
	FCollisionQueryParams CollisionQueryParams;
	CollisionQueryParams.AddIgnoredActor(this);

	return World->OverlapMultiByObjectType(OutResults,
	                                       QueryWorldOrigin,
	                                       QueryWorldRotation,
	                                       ObjectQueryParams,
	                                       VolumeQueryShape,
	                                       CollisionQueryParams);
}

bool AUniformGridVolume::CheckVoxelOverlap(const FIntVector& VoxelIndex, UPrimitiveComponent* ComponentToTest) const
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
		GetActorQuat(),
		FCollisionShape::MakeBox(VoxelHalfExtent)
	);
}

bool AUniformGridVolume::IsPathSegmentWalkable(const FIntVector& FromStartIndex, const FIntVector& ToStartIndex,
                                               int32 AgentRadiusInVoxels) const
{
	return false;
}

void AUniformGridVolume::UpdateDebugObstacleVisualization()
{
	if (bIsUpdatingVisualization)
	{
		return;
	}

	if (!bDrawDebugBox || !DebugObstaclesComponent || !DebugObstaclesComponent->GetStaticMesh())
	{
		// 바로 GameThread 에서 지우기
		if (DebugObstaclesComponent)
		{
			DebugObstaclesComponent->ClearInstances();
		}
		if (VolumeBoundsVisualizer)
		{
			VolumeBoundsVisualizer->SetVisibility(false);
		}
		return;
	}

	if (VolumeBoundsVisualizer)
	{
		VolumeBoundsVisualizer->SetVisibility(true);
	}

	bIsUpdatingVisualization = true;

	// 1) 컴포넌트 월드 트랜스폼 스냅샷
	const FTransform CompXform = DebugObstaclesComponent->GetComponentTransform();
	TWeakObjectPtr<AUniformGridVolume> WeakThis(this);

	// 2) 백그라운드에서 인덱스 수집 + 월드→로컬 변환
	Async(EAsyncExecution::ThreadPool, [WeakThis, CompXform]()
	{
		if (!WeakThis.IsValid()) return;

		TSet<FIntVector> RequiredIndices;
		TArray<FIntVector> ObstacleIndices;
		WeakThis->GetDebugObstacleVoxelIndices(ObstacleIndices);

		for (const FIntVector& Index : ObstacleIndices)
		{
			RequiredIndices.Add(Index);
		}

		// 3) 게임 스레드에서 실제 Clear/Add 호출
		AsyncTask(ENamedThreads::GameThread, [WeakThis, RequiredIndices = MoveTemp(RequiredIndices)]() mutable
		{
			if (!WeakThis.IsValid() || !WeakThis->DebugObstaclesComponent)
			{
				if (WeakThis.IsValid()) WeakThis->bIsUpdatingVisualization = false;
				return;
			}

			auto Comp = WeakThis->DebugObstaclesComponent;
			const float Scale = WeakThis->VoxelExtent / 100.0f;

			// 2. 제거할 인스턴스 계산 (이전에는 있었지만 지금은 없는 것)
			TSet<FIntVector> IndicesToRemove = WeakThis->DisplayedVoxelIndices.Difference(RequiredIndices);

			// 3. 추가할 인스턴스 계산 (이전에는 없었지만 지금은 있는 것)
			TSet<FIntVector> IndicesToAdd = RequiredIndices.Difference(WeakThis->DisplayedVoxelIndices);

			if (IndicesToRemove.Num() > 0)
			{
				TArray<int32> InstanceIndicesToRemove;
				// TMap을 사용해 복셀 인덱스로 인스턴스 인덱스를 빠르게 찾을 수 있다면 더 효율적입니다.
				// 여기서는 간단하게 전체를 순회합니다.
				for (int32 i = Comp->GetInstanceCount() - 1; i >= 0; --i)
				{
					FTransform InstanceTransform;
					Comp->GetInstanceTransform(i, InstanceTransform, /*bWorldSpace=*/ false);

					// 월드 위치로 변환 후 인덱스 계산 (정확성을 위해)
					FVector WorldPos = Comp->GetComponentTransform().TransformPosition(InstanceTransform.GetLocation());
					FIntVector VoxelIndex = WeakThis->WorldToVoxelIndex(WorldPos);

					if (IndicesToRemove.Contains(VoxelIndex))
					{
						Comp->RemoveInstance(i);
					}
				}
			}

			if (IndicesToAdd.Num() > 0)
			{
				TArray<FTransform> NewTransforms;
				NewTransforms.Reserve(IndicesToAdd.Num());
				for (const FIntVector& Idx : IndicesToAdd)
				{
					FVector WorldPos = WeakThis->VoxelIndexToWorld(Idx);
					FVector LocalPos = Comp->GetComponentTransform().InverseTransformPosition(WorldPos);
					NewTransforms.Emplace(FRotator::ZeroRotator, LocalPos, FVector(Scale));
				}
				Comp->AddInstances(NewTransforms, /*bWorldSpace=*/ false);
			}
			// 6. 현재 표시된 인덱스 상태를 최신으로 업데이트
			WeakThis->DisplayedVoxelIndices = MoveTemp(RequiredIndices);

			// 모든 작업이 완료된 후 플래그를 false로 설정
			WeakThis->bIsUpdatingVisualization = false;
		});
	});
}
