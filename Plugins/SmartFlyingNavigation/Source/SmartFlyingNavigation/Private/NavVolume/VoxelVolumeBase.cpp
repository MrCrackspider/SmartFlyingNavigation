// Copyright © 2025 TeamHamDev. All Rights Reserved.

#include "NavVolume/VoxelVolumeBase.h"
#include "Components/BoxComponent.h"
#include "TimerManager.h"
#include "Engine/World.h"
#include "Components/HierarchicalInstancedStaticMeshComponent.h"

//~======================================================================
// Constructor
//~======================================================================

AVoxelVolumeBase::AVoxelVolumeBase()
{
	// Allow this actor to be ticked every frame.
	PrimaryActorTick.bCanEverTick = true;

	bIsSpatiallyLoaded = true;

	// Create the volume box component for editor visualization.
	VolumeBox = CreateDefaultSubobject<UBoxComponent>(TEXT("SelectionVolume"));
	RootComponent = VolumeBox;
	VolumeBox->SetHiddenInGame(true);
	//VolumeBox->bIsEditorOnly = true;
	VolumeBox->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	VolumeBox->SetCollisionResponseToAllChannels(ECR_Ignore);
	VolumeBox->SetCollisionResponseToChannel(ECC_Visibility, ECR_Block);
	
#if WITH_EDITOR
	// Create HISM components for debug visualization.
	DebugSolidNodesComponent = CreateDefaultSubobject<UHierarchicalInstancedStaticMeshComponent>(TEXT("DebugSolidNodesComponent"));
	DebugSolidNodesComponent->SetupAttachment(VolumeBox);
	
	DebugMixedNodesComponent = CreateDefaultSubobject<UHierarchicalInstancedStaticMeshComponent>(TEXT("DebugMixedNodesComponent"));
	DebugMixedNodesComponent->SetupAttachment(VolumeBox);

	// Set default paths for debug materials.
	DebugSolidNodeMaterialPath = TEXT("/Script/Engine.MaterialInstanceConstant'/SmartFlyingNavigation/Debug/Material/MI_NavDebugSolidVoxel.MI_NavDebugSolidVoxel'");
	DebugMixedNodeMaterialPath = TEXT("/Script/Engine.MaterialInstanceConstant'/SmartFlyingNavigation/Debug/Material/MI_NavDebugMixedVoxel.MI_NavDebugMixedVoxel'");
	DebugLineWidth = 400.0f;
#endif
	
	// Set default volume properties.
	MapSize = FIntVector(256, 256, 256);
	VoxelExtent = 100.f;
	bDrawDebugBox = true;
}

//~======================================================================
// AActor Overrides
//~======================================================================

void AVoxelVolumeBase::BeginPlay()
{
	Super::BeginPlay();
	
	UE_LOG(LogTemp, Warning, TEXT("CHUNK LOADED: %s at %s"), *GetActorLabel(), *GetActorLocation().ToString());

	// If debug drawing is enabled, set a timer to update the visualization.
	if (bDrawDebugBox)
	{
		GetWorldTimerManager().SetTimer(DebugUpdateTimer, this, &AVoxelVolumeBase::UpdateDebugObstacleVisualization,
			0.8f, false, 0.5f);
	}
}

void AVoxelVolumeBase::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);

#if WITH_EDITOR
	// Set the path for the debug static mesh.
	static const FSoftObjectPath DebugMeshPath(
		TEXT("/Script/Engine.StaticMesh'/SmartFlyingNavigation/Debug/SM_DebugVoxel.SM_DebugVoxel'"));
	DebugMeshAsset = DebugMeshPath;
	
	// Load debug materials from their paths.
	UObject* LoadedObject = DebugSolidNodeMaterialPath.TryLoad();
	if (LoadedObject)
	{
		SolidNodeMaterial = Cast<UMaterialInstance>(LoadedObject);
	}
	
	LoadedObject = DebugMixedNodeMaterialPath.TryLoad();
	if (LoadedObject)
	{
		MixedNodeMaterial = Cast<UMaterialInstance>(LoadedObject);
	}

	// Load the debug mesh and apply it to the HISM components.
	if (UStaticMesh* LoadedMesh = DebugMeshAsset.LoadSynchronous())
	{
		// Assign the same mesh to all 3 components.
		DebugSolidNodesComponent->SetStaticMesh(LoadedMesh);
		DebugMixedNodesComponent->SetStaticMesh(LoadedMesh);

		// Disable collision for debug meshes.
		DebugSolidNodesComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		DebugMixedNodesComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);

		// Assign materials (if they were loaded successfully).
		if (SolidNodeMaterial) DebugSolidNodesComponent->SetMaterial(0, SolidNodeMaterial);
		if (MixedNodeMaterial) DebugMixedNodesComponent->SetMaterial(0, MixedNodeMaterial);
	}
	VolumeBox->SetLineThickness(DebugLineWidth);
	
#endif

	// Prevent direct scaling of the actor and log a warning.
	if (!GetActorScale3D().Equals(FVector::OneVector, 0.001f))
	{
		SetActorScale3D(FVector::OneVector);
	}

	// Calculate the total size and half-extent of the volume.
	const FVector TotalSize = FVector(MapSize) * VoxelExtent;
	const FVector HalfExtent = TotalSize * 0.5f;

	// Since the RootComponent is now the pivot, the box's relative location should be HalfExtent
	// to position the entire volume in positive coordinate space.
	VolumeBox->SetBoxExtent(HalfExtent);
	//VolumeBox->SetRelativeLocation(HalfExtent);
	
}

void AVoxelVolumeBase::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

void AVoxelVolumeBase::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	FString ReasonString;
	switch (EndPlayReason)
	{
	case EEndPlayReason::Destroyed: ReasonString = TEXT("Destroyed"); break;
	case EEndPlayReason::LevelTransition: ReasonString = TEXT("Level Transition"); break;
	case EEndPlayReason::RemovedFromWorld: ReasonString = TEXT("Removed from World (Streaming Out)"); break; // 우리가 원하는 이유
	case EEndPlayReason::Quit: ReasonString = TEXT("Quit"); break;
	default: ReasonString = TEXT("Unknown"); break;
	}
	UE_LOG(LogTemp, Error, TEXT("CHUNK UNLOADED: %s (Reason: %s)"), *GetActorLabel(), *ReasonString);

	// Clear the debug update timer.
	GetWorldTimerManager().ClearTimer(DebugUpdateTimer);
#if WITH_EDITOR 
	// Clear all instances from the debug mesh components.
	if (DebugSolidNodesComponent)
	{
		DebugSolidNodesComponent->ClearInstances();
	}
	if (DebugMixedNodesComponent)
	{
		DebugMixedNodesComponent->ClearInstances();
	}
#endif
	Super::EndPlay(EndPlayReason);
}

//~======================================================================
// Voxel Conversion and Queries
//~======================================================================

FVector AVoxelVolumeBase::VoxelIndexToWorld(const FIntVector& VoxelIndex) const
{
	const FIntVector HalfMapSize = MapSize / 2;
	const FVector VoxelLocalPosition = (FVector(VoxelIndex - HalfMapSize) * VoxelExtent) + FVector(VoxelExtent * 0.5f);

	// 2. 이 로컬 좌표를 월드 좌표로 변환합니다.
	return GetActorTransform().TransformPosition(VoxelLocalPosition);
}

FIntVector AVoxelVolumeBase::WorldToVoxelIndex(const FVector& WorldPosition) const
{
	if (VoxelExtent <= 0.f) return FIntVector::ZeroValue;

	// 1. 월드 좌표를 볼륨의 로컬 좌표로 변환합니다.
	const FVector LocalPosition = GetActorTransform().InverseTransformPosition(WorldPosition);

	// 2. 로컬 좌표를 복셀 인덱스로 변환하고, HalfMapSize를 더해 배열 인덱스로 만듭니다.
	const FIntVector HalfMapSize = MapSize / 2;
	return FIntVector(
		FMath::FloorToInt(LocalPosition.X / VoxelExtent) + HalfMapSize.X,
		FMath::FloorToInt(LocalPosition.Y / VoxelExtent) + HalfMapSize.Y,
		FMath::FloorToInt(LocalPosition.Z / VoxelExtent) + HalfMapSize.Z
	);
}

bool AVoxelVolumeBase::IsIndexValid(const FIntVector& Index) const
{
	// Check if the index is within the volume's bounds.
	return Index.X >= 0 && Index.X < MapSize.X &&
		Index.Y >= 0 && Index.Y < MapSize.Y &&
		Index.Z >= 0 && Index.Z < MapSize.Z;
}

bool AVoxelVolumeBase::ClampVoxelIndicesToVolume(const FIntVector& InMinVoxelIndex, const FIntVector& InMaxVoxelIndex,
	FIntVector& OutClampedMin, FIntVector& OutClampedMax) const
{
	// Clamp the min and max indices to the valid range of the volume.
	OutClampedMin.X = FMath::Max(InMinVoxelIndex.X, 0);
	OutClampedMin.Y = FMath::Max(InMinVoxelIndex.Y, 0);
	OutClampedMin.Z = FMath::Max(InMinVoxelIndex.Z, 0);

	OutClampedMax.X = FMath::Min(InMaxVoxelIndex.X, MapSize.X - 1);
	OutClampedMax.Y = FMath::Min(InMaxVoxelIndex.Y, MapSize.Y - 1);
	OutClampedMax.Z = FMath::Min(InMaxVoxelIndex.Z, MapSize.Z - 1);

	// Return true if the resulting clamped range is valid.
	return (OutClampedMax.X >= OutClampedMin.X &&
		OutClampedMax.Y >= OutClampedMin.Y &&
		OutClampedMax.Z >= OutClampedMin.Z);
}

//~======================================================================
// Overlap and Collision
//~======================================================================

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

	// The center of the query box in local space.
	const FVector VolumeLocalCenter = VolumeHalfExtent;
	// The center of the query box in world space.
	const FVector QueryWorldOrigin = ActorTransform.TransformPosition(VolumeLocalCenter);
	const FQuat QueryWorldRotation = ActorTransform.GetRotation();

	FCollisionShape VolumeQueryShape = FCollisionShape::MakeBox(VolumeHalfExtent);
	FCollisionQueryParams CollisionQueryParams;
	CollisionQueryParams.AddIgnoredActor(this);

	// Perform the overlap query.
	return World->OverlapMultiByObjectType(OutResults,
		QueryWorldOrigin,
		QueryWorldRotation,
		ObjectQueryParams,
		VolumeQueryShape,
		CollisionQueryParams);
}

bool AVoxelVolumeBase::IsVoxelOverlapping(const FIntVector& VoxelIndex, const FCollisionObjectQueryParams& ObjectQueryParams) const
{
	UWorld* World = GetWorld();
	if (!World)
	{
		return false;
	}

	const FVector VoxelWorldCenter = VoxelIndexToWorld(VoxelIndex);
	const FVector VoxelHalfExtent = FVector(VoxelExtent * 0.5f);
	const FCollisionShape VoxelShape = FCollisionShape::MakeBox(VoxelHalfExtent);

	return World->OverlapAnyTestByObjectType(VoxelWorldCenter, FQuat::Identity, ObjectQueryParams, VoxelShape);
}

// ======================================================================
// Debug Visualization
//~======================================================================

void AVoxelVolumeBase::UpdateDebugObstacleVisualization()
{
#if WITH_EDITORONLY_DATA
	if (!bDrawDebugBox || !GetWorld())
	{
		return;
	}

	// Prevent concurrent updates.
	if (bIsUpdatingVisualization)
	{
		return;
	}

	// If debug drawing is turned off, clear any existing instances.
	if (!bDrawDebugBox)
	{
		DebugSolidNodesComponent->ClearInstances();
		DebugMixedNodesComponent->ClearInstances();
	}
#endif
}