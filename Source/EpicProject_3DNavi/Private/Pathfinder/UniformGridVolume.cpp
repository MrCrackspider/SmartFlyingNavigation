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

bool AUniformGridVolume::GetOverlappingComponentsInVolume(const FCollisionObjectQueryParams& ObjectQueryParams,TArray<FOverlapResult>& OutResults) const
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
	if (!bDrawDebugObstacles || !DebugObstaclesComponent || !DebugObstaclesComponent->GetStaticMesh())
	{
		if (DebugObstaclesComponent)
		{
			DebugObstaclesComponent->ClearInstances();
		}
		return;
	}

	TArray<FIntVector> ObstacleIndices;
	GetDebugObstacleVoxelIndices(ObstacleIndices);

	TArray<FTransform> InstanceTransforms;
	InstanceTransforms.Reserve(ObstacleIndices.Num());
	const FVector Scale = FVector(VoxelExtent / 100.0f);

	// 컴포넌트의 월드 트랜스폼을 가져옵니다.
	const FTransform ComponentWorldTransform = DebugObstaclesComponent->GetComponentTransform();

	for (const FIntVector& GlobalIndex : ObstacleIndices)
	{
		// 1. VoxelIndexToWorld를 사용해 복셀의 '월드' 위치를 정확하게 계산합니다.
		//    이 함수는 액터의 현재 위치와 회전을 모두 반영합니다.
		const FVector VoxelWorldCenter = VoxelIndexToWorld(GlobalIndex);

		// 2. AddInstances는 '로컬' 공간의 트랜스폼을 요구합니다.
		//    따라서 계산된 월드 위치를 컴포넌트의 로컬 공간으로 변환해줍니다.
		const FVector VoxelComponentLocalCenter = ComponentWorldTransform.InverseTransformPosition(VoxelWorldCenter);

		// 3. 변환된 로컬 위치를 사용해 트랜스폼을 만듭니다.
		InstanceTransforms.Emplace(FRotator::ZeroRotator, VoxelComponentLocalCenter, Scale);
	}

	DebugObstaclesComponent->ClearInstances();
	if (InstanceTransforms.Num() > 0)
	{
		// 이제 이 트랜스폼들은 항상 올바른 위치를 가리킵니다.
		DebugObstaclesComponent->AddInstances(InstanceTransforms, false);
	}
}




