// Fill out your copyright notice in the Description page of Project Settings.


#include "UniformGridVolume.h"

// Sets default values
AUniformGridVolume::AUniformGridVolume()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void AUniformGridVolume::BeginPlay()
{
	Super::BeginPlay();
	
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
		FQuat::Identity,
		FCollisionShape::MakeBox(VoxelHalfExtent)
	);
}





