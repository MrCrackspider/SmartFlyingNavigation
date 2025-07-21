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

	// Create and set the root component.
	PivotComponent = CreateDefaultSubobject<USceneComponent>(TEXT("RootComponent"));
	SetRootComponent(PivotComponent);

	// Create the volume box component for editor visualization.
	VolumeBox = CreateDefaultSubobject<UBoxComponent>(TEXT("SelectionVolume"));
	VolumeBox->SetupAttachment(PivotComponent);
	VolumeBox->SetHiddenInGame(true);
	VolumeBox->bIsEditorOnly = true;
	VolumeBox->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	VolumeBox->SetCollisionResponseToAllChannels(ECR_Ignore);
	VolumeBox->SetCollisionResponseToChannel(ECC_Visibility, ECR_Block);
	
#if WITH_EDITOR
	// Create HISM components for debug visualization.
	DebugSolidNodesComponent = CreateDefaultSubobject<UHierarchicalInstancedStaticMeshComponent>(TEXT("DebugSolidNodesComponent"));
	DebugSolidNodesComponent->SetupAttachment(PivotComponent);

	DebugEmptyNodesComponent = CreateDefaultSubobject<UHierarchicalInstancedStaticMeshComponent>(TEXT("DebugEmptyNodesComponent"));
	DebugEmptyNodesComponent->SetupAttachment(PivotComponent);

	DebugMixedNodesComponent = CreateDefaultSubobject<UHierarchicalInstancedStaticMeshComponent>(TEXT("DebugMixedNodesComponent"));
	DebugMixedNodesComponent->SetupAttachment(PivotComponent);

	// Set default paths for debug materials.
	DebugSolidNodeMaterialPath = TEXT("/Script/Engine.MaterialInstanceConstant'/SmartFlyingNavigation/Debug/Material/MI_NavDebugMesh2.MI_NavDebugMesh2'");
	DebugEmptyNodeMaterialPath = TEXT("/Script/Engine.MaterialInstanceConstant'/SmartFlyingNavigation/Debug/Material/MI_NavDebugMesh.MI_NavDebugMesh'");
	DebugMixedNodeMaterialPath = TEXT("/Script/Engine.MaterialInstanceConstant'/SmartFlyingNavigation/Debug/Material/MI_NavDebugMesh3.MI_NavDebugMesh3'");
	DebugLineWidth = 400.0f;
#endif
	
	// Set default volume properties.
	MapSize = FIntVector(128, 128, 128);
	VoxelExtent = 100.f;
	bDrawDebugBox = true;
}

//~======================================================================
// AActor Overrides
//~======================================================================

void AVoxelVolumeBase::BeginPlay()
{
	Super::BeginPlay();

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
	LoadedObject = DebugEmptyNodeMaterialPath.TryLoad();
	if (LoadedObject)
	{
		EmptyNodeMaterial = Cast<UMaterialInstance>(LoadedObject);
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
		DebugEmptyNodesComponent->SetStaticMesh(LoadedMesh);
		DebugMixedNodesComponent->SetStaticMesh(LoadedMesh);

		// Disable collision for debug meshes.
		DebugSolidNodesComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		DebugEmptyNodesComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		DebugMixedNodesComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);

		// Assign materials (if they were loaded successfully).
		if (SolidNodeMaterial) DebugSolidNodesComponent->SetMaterial(0, SolidNodeMaterial);
		if (EmptyNodeMaterial) DebugEmptyNodesComponent->SetMaterial(0, EmptyNodeMaterial);
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
	VolumeBox->SetRelativeLocation(HalfExtent);
	
}

void AVoxelVolumeBase::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

void AVoxelVolumeBase::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	// Clear the debug update timer.
	GetWorldTimerManager().ClearTimer(DebugUpdateTimer);
#if WITH_EDITOR 
	// Clear all instances from the debug mesh components.
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
#endif
	Super::EndPlay(EndPlayReason);
}

//~======================================================================
// Voxel Conversion and Queries
//~======================================================================

FVector AVoxelVolumeBase::VoxelIndexToWorld(const FIntVector& VoxelIndex) const
{
	// Calculate the local center of the voxel and transform it to world space.
	const FVector LocalCenter = FVector(VoxelIndex) * VoxelExtent + (VoxelExtent * 0.5f);
	return GetActorTransform().TransformPosition(LocalCenter);
}

FIntVector AVoxelVolumeBase::WorldToVoxelIndex(const FVector& WorldPosition) const
{
	if (VoxelExtent <= 0.f) return FIntVector::ZeroValue;

	// Transform the world position to local space and calculate the corresponding voxel index.
	const FVector LocalPosition = GetActorTransform().InverseTransformPosition(WorldPosition);
	return FIntVector(
		FMath::FloorToInt(LocalPosition.X / VoxelExtent),
		FMath::FloorToInt(LocalPosition.Y / VoxelExtent),
		FMath::FloorToInt(LocalPosition.Z / VoxelExtent)
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

bool AVoxelVolumeBase::IsVoxelOverlapping(const FIntVector& VoxelIndex, UPrimitiveComponent* ComponentToTest) const
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

	// Check for overlap between the component and the voxel's bounding box.
	return ComponentToTest->OverlapComponent(VoxelCenter,
		GetActorQuat(),
		FCollisionShape::MakeBox(VoxelHalfExtent));
}

//~======================================================================
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
		DebugEmptyNodesComponent->ClearInstances();
		DebugMixedNodesComponent->ClearInstances();
	}
#endif
}