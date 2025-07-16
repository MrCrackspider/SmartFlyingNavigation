// Fill out your copyright notice in the Description page of Project Settings.


#include "VoxelVolumeBase.h"

#include "Components/BoxComponent.h"
#include "Components/InstancedStaticMeshComponent.h"


AVoxelVolumeBase::AVoxelVolumeBase()
{
	PrimaryActorTick.bCanEverTick = true;

	PivotComponent = CreateDefaultSubobject<USceneComponent>(TEXT("RootComponent"));
	SetRootComponent(PivotComponent);

	VolumeBox = CreateDefaultSubobject<UBoxComponent>(TEXT("SelectionVolume"));
	VolumeBox->SetupAttachment(PivotComponent);
	VolumeBox->SetHiddenInGame(true);
	VolumeBox->bIsEditorOnly = true;
	VolumeBox->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	VolumeBox->SetCollisionResponseToAllChannels(ECR_Ignore);
	VolumeBox->SetCollisionResponseToChannel(ECC_Visibility, ECR_Block);
	
	DebugSolidNodesComponent = CreateDefaultSubobject<UInstancedStaticMeshComponent>(TEXT("DebugSolidNodesComponent"));
	DebugSolidNodesComponent->SetupAttachment(PivotComponent);
	
	DebugEmptyNodesComponent = CreateDefaultSubobject<UInstancedStaticMeshComponent>(TEXT("DebugEmptyNodesComponent"));
	DebugEmptyNodesComponent->SetupAttachment(PivotComponent);

	DebugMixedNodesComponent = CreateDefaultSubobject<UInstancedStaticMeshComponent>(TEXT("DebugMixedNodesComponent"));
	DebugMixedNodesComponent->SetupAttachment(PivotComponent);
	
	
	MapSize = FIntVector(128, 128, 128);
	VoxelExtent = 100.f;
	bDrawDebugBox = true;
}

void AVoxelVolumeBase::BeginPlay()
{
	Super::BeginPlay();
	if (bDrawDebugBox)
	{
		GetWorldTimerManager().SetTimer(DebugUpdateTimer, this, &AVoxelVolumeBase::UpdateDebugObstacleVisualization,
		                                0.8f, true, 0.5f);
	}
}

void AVoxelVolumeBase::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);

	static const FSoftObjectPath DebugMeshPath(
		TEXT("/Script/Engine.StaticMesh'/Navigation3D/PathfinderPlugin/Debug/SM_DebugObstacle.SM_DebugObstacle'"));
	DebugMeshAsset = DebugMeshPath;
#if WITH_EDITORONLY_DATA

	if (UStaticMesh* LoadedMesh = DebugMeshAsset.LoadSynchronous())
	{
		// 3개의 컴포넌트에 모두 동일한 메시 할당
		DebugSolidNodesComponent->SetStaticMesh(LoadedMesh);
		DebugEmptyNodesComponent->SetStaticMesh(LoadedMesh);
		DebugMixedNodesComponent->SetStaticMesh(LoadedMesh);
		
		// 충돌 비활성화
		DebugSolidNodesComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		DebugEmptyNodesComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		DebugMixedNodesComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	
		// 머티리얼 할당 (할당된 경우)
		if (SolidNodeMaterial) DebugSolidNodesComponent->SetMaterial(0, SolidNodeMaterial);
		if (EmptyNodeMaterial) DebugEmptyNodesComponent->SetMaterial(0, EmptyNodeMaterial);
		if (MixedNodeMaterial) DebugMixedNodesComponent->SetMaterial(0, MixedNodeMaterial);
	}
#endif
	if (!GetActorScale3D().Equals(FVector::OneVector, 0.001f))
	{
		SetActorScale3D(FVector::OneVector);
		UE_LOG(LogTemp, Warning,
		       TEXT(
			       "VoxelVolume: The scale cannot be adjusted directly. Please use the 'MapSize' and 'VoxelExtent' properties."
		       ));
	}

	FVector TotalSize = FVector(MapSize) * VoxelExtent;
	FVector HalfExtent = TotalSize * 0.5f;

	// 이제 RootComponent가 Pivot이므로, Box의 상대 위치는 HalfExtent가 되어야 전체 볼륨이 양수 좌표에 위치하게 됩니다.
	VolumeBox->SetBoxExtent(HalfExtent);
	VolumeBox->SetRelativeLocation(HalfExtent);
	VolumeBox->SetLineThickness(DebugLineWidth);
}

void AVoxelVolumeBase::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

void AVoxelVolumeBase::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	GetWorldTimerManager().ClearTimer(DebugUpdateTimer);

	if (DebugSolidNodesComponent)
	{
		DebugSolidNodesComponent->ClearInstances();
	}
	if (DebugEmptyNodesComponent)
	{
		DebugEmptyNodesComponent->ClearInstances();
	}
	if (DebugMixedNodesComponent)
	{
		DebugMixedNodesComponent->ClearInstances();
	}
    
	Super::EndPlay(EndPlayReason);
}


FVector AVoxelVolumeBase::VoxelIndexToWorld(const FIntVector& VoxelIndex) const
{
	const FVector LocalCenter = FVector(VoxelIndex) * VoxelExtent + (VoxelExtent * 0.5f);
	return GetActorTransform().TransformPosition(LocalCenter);
}

FIntVector AVoxelVolumeBase::WorldToVoxelIndex(const FVector& WorldPosition) const
{
	if (VoxelExtent <= 0.f) return FIntVector::ZeroValue;
	const FVector LocalPosition = GetActorTransform().InverseTransformPosition(WorldPosition);
	return FIntVector(
		FMath::FloorToInt(LocalPosition.X / VoxelExtent),
		FMath::FloorToInt(LocalPosition.Y / VoxelExtent),
		FMath::FloorToInt(LocalPosition.Z / VoxelExtent)
	);
}

bool AVoxelVolumeBase::IsVoxelWalkable(const FIntVector& VoxelIndex) const
{
	return false;
}

void AVoxelVolumeBase::RegisterDynamicObstacle(UDynamicObstacleComponent* ObstacleComponent)
{
}

void AVoxelVolumeBase::UnregisterDynamicObstacle(UDynamicObstacleComponent* ObstacleComponent)
{
}


bool AVoxelVolumeBase::IsIndexValid(const FIntVector& Index) const
{
	return Index.X >= 0 && Index.X < MapSize.X &&
		Index.Y >= 0 && Index.Y < MapSize.Y &&
		Index.Z >= 0 && Index.Z < MapSize.Z;
}


bool AVoxelVolumeBase::ClampVoxelIndicesToVolume(FIntVector& MinVoxelIndex, FIntVector& MaxVoxelIndex) const
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

bool AVoxelVolumeBase::ClampVoxelIndicesToVolume(const FIntVector& InMinVoxelIndex, const FIntVector& InMaxVoxelIndex,
                                                 FIntVector& OutClampedMin, FIntVector& OutClampedMax) const
{
	OutClampedMin.X = FMath::Max(InMinVoxelIndex.X, 0);
	OutClampedMin.Y = FMath::Max(InMinVoxelIndex.Y, 0);
	OutClampedMin.Z = FMath::Max(InMinVoxelIndex.Z, 0);

	OutClampedMax.X = FMath::Min(InMaxVoxelIndex.X, MapSize.X - 1);
	OutClampedMax.Y = FMath::Min(InMaxVoxelIndex.Y, MapSize.Y - 1);
	OutClampedMax.Z = FMath::Min(InMaxVoxelIndex.Z, MapSize.Z - 1);

	return (OutClampedMax.X >= OutClampedMin.X &&
		OutClampedMax.Y >= OutClampedMin.Y &&
		OutClampedMax.Z >= OutClampedMin.Z);
}


bool AVoxelVolumeBase::GetOverlappingComponentsInVolume(const FCollisionObjectQueryParams& ObjectQueryParams,
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


bool AVoxelVolumeBase::CheckVoxelOverlap(const FIntVector& VoxelIndex, UPrimitiveComponent* ComponentToTest) const
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

	return ComponentToTest->OverlapComponent(VoxelCenter,
	                                         GetActorQuat(),
	                                         FCollisionShape::MakeBox(VoxelHalfExtent));
}

#if WITH_EDITOR
void AVoxelVolumeBase::BakeStaticData()
{

}
#endif

void AVoxelVolumeBase::RegisterStreamingAgent(AActor* Agent)
{
}

void AVoxelVolumeBase::UnregisterStreamingAgent(AActor* Agent)
{
}

void AVoxelVolumeBase::GetDebugObstacleVoxelIndices(TArray<FIntVector>& OutObstacleIndices) const
{
}


void AVoxelVolumeBase::UpdateDebugObstacleVisualization()
{
	if (!bDrawDebugBox || !GetWorld())
	{
		return;
	}
	
	if (bIsUpdatingVisualization)
	{
		return;
	}

	if (!bDrawDebugBox)
	{
		DebugSolidNodesComponent->ClearInstances();
		DebugEmptyNodesComponent->ClearInstances();
		DebugMixedNodesComponent->ClearInstances();
	}
}
