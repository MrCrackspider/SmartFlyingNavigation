// Fill out your copyright notice in the Description page of Project Settings.


#include "Pathfinder/NavVolume/UniformGridVolume.h"

// Sets default values
// AUniformGridVolume::AUniformGridVolume()
// {
// 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
// 	PrimaryActorTick.bCanEverTick = true;
//
// 	// 2) Pivot Component (이게 최종 피벗)
// 	PivotComponent = CreateDefaultSubobject<USceneComponent>(TEXT("PivotComponent"));
// 	PivotComponent->SetupAttachment(RootComponent);
//
// 	// 3) BoxComponent 는 PivotComponent 의 자식으로
// 	VolumeBox = CreateDefaultSubobject<UBoxComponent>(TEXT("SelectionVolume"));
// 	VolumeBox->SetupAttachment(PivotComponent);
//
// 	// 에디터 클릭용 세팅
// 	VolumeBox->SetHiddenInGame(true);
// 	VolumeBox->bIsEditorOnly = true;
// 	VolumeBox->SetCollisionEnabled(ECollisionEnabled::NoCollision);
// 	VolumeBox->SetCollisionResponseToAllChannels(ECR_Ignore);
// 	VolumeBox->SetCollisionResponseToChannel(ECC_Visibility, ECR_Block);
//
// 	MapSize = FIntVector(128, 128, 128);
// 	VoxelExtent = 100.f;
// }
//
// // Called when the game starts or when spawned
// void AUniformGridVolume::BeginPlay()
// {
// 	Super::BeginPlay();
// }
//
// void AUniformGridVolume::OnConstruction(const FTransform& Transform)
// {
// 	Super::OnConstruction(Transform);
//
//
// 	if (!GetActorScale3D().Equals(FVector::OneVector, 0.001f))
// 	{
// 		// 즉시 (1,1,1)로 되돌립니다.
// 		SetActorScale3D(FVector::OneVector);
//
// 		// 사용자에게 왜 스케일이 되돌아갔는지 알려주면 더 좋습니다.
// 		UE_LOG(LogTemp, Warning, TEXT("AUniformGridVolume: 스케일은 직접 조절할 수 없습니다. 'MapSize'와 'VoxelExtent' 프로퍼티를 사용하세요."));
// 	}
// 	
// 	FVector TotalSize  = FVector(MapSize) * VoxelExtent;  
// 	FVector HalfExtent = TotalSize * 0.5f;
//
// 	// +X 축(정면)에서 볼 때 가장 가까운 ‘앞면 Front’ 좌표 = +HalfExtent.X
// 	// 왼쪽은 –Y, 아래는 –Z 이므로…
// 	FVector CustomPivotOffset = FVector(
// 		HalfExtent.X,   // Front
// 		HalfExtent.Y,   // Left
// 		HalfExtent.Z    // Bottom
// 	);
//
// 	// 1) PivotComponent 를 볼륨의 ‘앞면 좌측하단’으로 이동
// 	PivotComponent->SetRelativeLocation(CustomPivotOffset);
//
// 	// 2) SelectionVolume 은 PivotComponent 기준 중앙에 박스(extent)만 지정
// 	VolumeBox->SetBoxExtent(HalfExtent);
// 	VolumeBox->SetLineThickness(DebugLineWidth * 10);
// 	VolumeBox->SetRelativeLocation(FVector::ZeroVector);
// 	
// 	UpdateDebugObstacleVisualization();
//
//
// }
//
// // Called every frame
// void AUniformGridVolume::Tick(float DeltaTime)
// {
// 	Super::Tick(DeltaTime);
// }
//
// FVector AUniformGridVolume::GetLocationFromIndex(
// 	const FIntVector& VoxelIndex) const
// {
// 	return Super::GetLocationFromIndex(VoxelIndex);
// }
//
// FIntVector AUniformGridVolume::GetIndexFromLocation(
// 	const FVector& WorldPosition) const
// {
// 	return Super::GetIndexFromLocation(WorldPosition);
// }
//
// FIntVector AUniformGridVolume::WorldToVoxelIndex(const FVector& WorldPosition) const
// {
// 	if (VoxelExtent <= 0.f)
// 	{
// 		UE_LOG(LogTemp, Error, TEXT("AVoxelVolume::WorldToVoxelIndex - VoxelExtent is zero or negative."));
// 		return FIntVector(EForceInit::ForceInitToZero);
// 	}
//
// 	const FVector LocalPosition = GetActorTransform().InverseTransformPosition(WorldPosition);
// 	const int32 IndexX = FMath::FloorToInt(LocalPosition.X / VoxelExtent);
// 	const int32 IndexY = FMath::FloorToInt(LocalPosition.Y / VoxelExtent);
// 	const int32 IndexZ = FMath::FloorToInt(LocalPosition.Z / VoxelExtent);
//
// 	return FIntVector(IndexX, IndexY, IndexZ);
// }
//
// FVector AUniformGridVolume::VoxelIndexToWorld(const FIntVector& VoxelIndex) const
// {
// 	const FVector LocalCenter = FVector(VoxelIndex) * VoxelExtent + (VoxelExtent * 0.5f);
// 	return GetActorTransform().TransformPosition(LocalCenter);
// }
//
//
// bool AUniformGridVolume::IsPathSegmentWalkable(const FIntVector& FromStartIndex, const FIntVector& ToStartIndex,
//                                                int32 AgentRadiusInVoxels) const
// {
// 	return false;
// }
//
AUniformGridVolume::AUniformGridVolume()
{
}
