// Copyright © 2025 TeamHamDev. All Rights Reserved.

#include "Component/PathfindingComponent.h"
#include "Components/SplineComponent.h"
#include "Subsystem/PathfindingSubsystem.h"
#include "Subsystem/AvoidanceSubsystem.h"
#include "Strategy/PathfindingStrategy.h"
#include "GameFramework/Actor.h"
#include "TimerManager.h"
#include "Engine/World.h"
#include "DrawDebugHelpers.h"
#include "Components/BoxComponent.h"
#include "Components/SphereComponent.h"
#include "NavVolume/OctreeVoxelVolume.h"
#include "Engine/Engine.h"
#include "Strategy/AStarStrategy.h"
#include "Components/WorldPartitionStreamingSourceComponent.h"
#include "Utility/Parallel/PathfindingRequest.h"

UPathfindingComponent::UPathfindingComponent()
{
	PrimaryComponentTick.bCanEverTick = true;

#if WITH_EDITOR
	bTickInEditor = true;
#endif

	PathfindingStrategyClass = UAStarStrategy::StaticClass();

	PathSplinePtr = CreateDefaultSubobject<USplineComponent>(TEXT("SplineComponent"));

	DebugSphereComponent = CreateDefaultSubobject<USphereComponent>(TEXT("DebugAgentRadiusSphere"));
	DebugSphereComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	DebugSphereComponent->SetCanEverAffectNavigation(false);
	DebugSphereComponent->bHiddenInGame = true;
	DebugSphereComponent->SetSphereRadius(BaseAgentRadius);

	// StreamingSourceComponent = CreateDefaultSubobject<UWorldPartitionStreamingSourceComponent>(TEXT("StreamingSourceComponent"));
	// StreamingSourceComponent->DisableStreamingSource();
}

void UPathfindingComponent::BeginPlay()
{
	Super::BeginPlay();

	UWorld* World = GetWorld();
	if (!ensure(World)) return;

	PathfindingSubsystem = World->GetSubsystem<UPathfindingSubsystem>();
	if (!ensure(PathfindingSubsystem))
	{
		return;
	}

	if (!PathSplinePtr)
	{
		PathSplinePtr->SetAbsolute(true, true, true);
		PathSplinePtr->SetupAttachment(GetOwner()->GetRootComponent());
	}

	AvoidanceSubsystem = World->GetSubsystem<UAvoidanceSubsystem>();
	if (AvoidanceSubsystem)
	{
		AvoidanceSubsystem->RegisterAgent(this);
	}

	if (PathfindingStrategyClass)
	{
		PathfindingStrategy = NewObject<UPathfindingStrategy>(this, PathfindingStrategyClass);

		if (PathfindingStrategy)
		{
			PathfindingStrategy->Initialize();
		}
	}

	// if (StreamingSourceComponent)
	// {
	// 	ActivateStreaming();
	// }

	if (AVoxelVolumeBase* StartingVolume = GetCurrentVolume())
	{
		GoalUpdateThreshold = StartingVolume->VoxelExtent;
	}
	else
	{
		// Set default value
		// TODO: This should ideally be set to the same value as VoxelExtent.
		GoalUpdateThreshold = 100.0f;
	}
}

void UPathfindingComponent::TickComponent(float DeltaTime, ELevelTick TickType,
                                          FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	
	if (!IsValid(AvoidanceSubsystem) || !AvoidanceSubsystem->GetbUseAvoidance())
	{
		UpdateSteering(DeltaTime);
		ApplyMovement(PreferredVelocity, DeltaTime);
	}
	CurrentVelocity = PreferredVelocity;
	UpdateAgentRadiusFromScale();

	// If using Avoidance, the functions below are called by the Avoidance Subsystem, 
	// so they are commented out for now.
	
	/***
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (CurrentState == EPathfindingState::FollowPath && !bIsReachDestination)
	{
		if (FoundPath.IsValidIndex(CurrentPathDestinationIndex) && CurrentPathDestinationIndex > 0)
		{
			const AActor* Owner = GetOwner();
			if (!Owner) return;

			const FVector& TargetWaypoint = FoundPath[CurrentPathDestinationIndex];
			const FVector& PrevWaypoint = FoundPath[CurrentPathDestinationIndex - 1];

			const FVector PathSegmentDirection = (TargetWaypoint - PrevWaypoint).GetSafeNormal();

			const FVector VectorFromTargetToAgent = Owner->GetActorLocation() - TargetWaypoint;

			const float DotProduct = FVector::DotProduct(VectorFromTargetToAgent, PathSegmentDirection);

			if (DotProduct >= 0.0f)
			{
				ProgressNextPoint();

				return;
			}
		}
	}

	if (!IsValid(AvoidanceSubsystem) || !AvoidanceSubsystem->GetbUseAvoidance())
	{
		UpdateSteering(DeltaTime);
		ApplyMovement(PreferredVelocity, DeltaTime);
	}

	UpdateAgentRadiusFromScale();
	 */
}

void UPathfindingComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (AvoidanceSubsystem)
	{
		AvoidanceSubsystem->UnregisterAgent(this);
	}

	if (GetWorld())
	{
		GetWorld()->GetTimerManager().ClearAllTimersForObject(this);
	}

	// if (StreamingSourceComponent)
	// {
	// 	DeactivateStreaming();
	// }

	Super::EndPlay(EndPlayReason);
}

void UPathfindingComponent::OnRegister()
{
	Super::OnRegister();

	if (DebugSphereComponent && GetOwner() && GetOwner()->GetRootComponent())
	{
		DebugSphereComponent->AttachToComponent(GetOwner()->GetRootComponent(),
		                                        FAttachmentTransformRules::SnapToTargetNotIncludingScale);
	}
}

void UPathfindingComponent::OnVolumeBecameReadyForPendingRequest(AOctreeVoxelVolume* RegisteredVolume)
{
	if (!bHasPendingPathRequest)
	{
		if (UOctreeNavSubsystem* NavSubsystem = GetWorld()->GetSubsystem<UOctreeNavSubsystem>())
		{
			NavSubsystem->OnVolumeRegistered.RemoveDynamic(this, &UPathfindingComponent::OnVolumeBecameReadyForPendingRequest);
		}
		return;
	}

	UOctreeNavSubsystem* NavSubsystem = GetWorld()->GetSubsystem<UOctreeNavSubsystem>();
	if (!NavSubsystem) return;

	AOctreeVoxelVolume* CurrentVolume = NavSubsystem->GetVolumeChunkAt(GetOwner()->GetActorLocation());

	if (CurrentVolume && CurrentVolume->IsReadyForNavigation())
	{
		UE_LOG(LogTemp, Log, TEXT("PathfindingComponent: Pending request resumed. Start volume '%s' is now ready."), *CurrentVolume->GetName());

		bHasPendingPathRequest = false;
		NavSubsystem->OnVolumeRegistered.RemoveDynamic(this, &UPathfindingComponent::OnVolumeBecameReadyForPendingRequest);

		RequestPathToLocation(PendingTargetLocation);
	}
}

bool UPathfindingComponent::RequestPathToLocation(const FVector& TargetLocation)
{
	if (bIsReachDestination) return false;

	// --- 1. Validate Start Volume ---
	UOctreeNavSubsystem* NavSubsystem = GetWorld()->GetSubsystem<UOctreeNavSubsystem>();
	if (!NavSubsystem || !PathfindingSubsystem) return false;

	const FVector StartLocation = GetOwner()->GetActorLocation();
	AOctreeVoxelVolume* StartVolume = NavSubsystem->GetVolumeChunkAt(StartLocation);


	// --- 2. If the start volume is not ready, pend the request. ---
	if (!StartVolume || !StartVolume->IsReadyForNavigation())
	{
		UE_LOG(LogTemp, Log, TEXT("Start volume is not ready. Path request is pending."));
		bHasPendingPathRequest = true;
		PendingTargetLocation = TargetLocation;
		NavSubsystem->OnVolumeRegistered.RemoveDynamic(this, &UPathfindingComponent::OnVolumeBecameReadyForPendingRequest);
		NavSubsystem->OnVolumeRegistered.AddDynamic(this, &UPathfindingComponent::OnVolumeBecameReadyForPendingRequest);
		return true; // Request accepted
	}

	// --- 3. Branch for Local/Hierarchical Pathfinding ---
	bHasPendingPathRequest = false; // Clear any pending request flag
	CurrentTargetLocation = TargetLocation;

	return RequestLocalPath_Internal(StartLocation, TargetLocation, StartVolume);
	
	// AOctreeVoxelVolume* EndVolume = NavSubsystem->GetVolumeChunkAt(TargetLocation);
	//
	// --- CASE A: Clear Local Pathfinding ---
	// The start and end volumes are the same, valid, and ready.
	// if (EndVolume && StartVolume == EndVolume)
	// {
	// 	return RequestLocalPath_Internal(StartLocation, TargetLocation, StartVolume);
	// }
	// --- CASE B: Hierarchical Pathfinding ---
	// The end volume is different or is null because it hasn't been streamed in.
	// else
	// {
	// 	UE_LOG(LogTemp, Log, TEXT("Requesting HIERARCHICAL path from '%s' to an external or unloaded volume."), *StartVolume->GetName());
	//
	// 	FHierarchicalPathRequest HRequest;
	// 	HRequest.RequestingComponent = this;
	// 	HRequest.StartLocation = StartLocation;
	// 	HRequest.EndLocation = TargetLocation;
	// 	HRequest.AgentRadius = this->AgentRadius;
	// 	HRequest.Strategy = PathfindingStrategy;
	// 	HRequest.OnHierarchicalPathComplete.BindDynamic(this, &UPathfindingComponent::HandleWorldPathResult);
	//
	// 	// CurrentRequestID = PathfindingSubsystem->RequestHierarchicalPath(this, HRequest);
	// 	CurrentRequestID = PathfindingSubsystem->RequestHierarchicalPath(this, HRequest);
	// 	return CurrentRequestID != -1;
	// }
}

bool UPathfindingComponent::RequestLocalPath_Internal(const FVector& Start, const FVector& End,
                                                      AOctreeVoxelVolume* Volume)
{
	if (!PathfindingStrategy || !Volume)
	{
		return false;
	}

	FPathfindingRequest Request;
	Request.RequestingObject = this;
	Request.StartLocation = Start;
	Request.EndLocation = End;
	Request.AgentRadius = this->AgentRadius;
	Request.Strategy = PathfindingStrategy;
	Request.NodeType = EPathfindingNodeType::Octree;
	Request.OnWorldPathComplete.BindDynamic(this, &UPathfindingComponent::HandleWorldPathResult);

	// Create a data snapshot from the volume
	CreateOctreeDataSnapshot(Volume, Request.NavigationData);

	CurrentTargetLocation = End;
	CurrentRequestID = PathfindingSubsystem->RequestPath(this, Request);

	return CurrentRequestID != -1;
}

void UPathfindingComponent::StartPathfinding()
{
}

void UPathfindingComponent::CreateOctreeDataSnapshot(const AOctreeVoxelVolume* OctreeVolume, FOctreePathfindingData& SnapshotData) const
{
	// --- Step 1: Copy small, trivial member variables directly. ---
	SnapshotData.RootNodeIndex = OctreeVolume->GetRootNodeIndex();
	SnapshotData.MaxOctreeDepth = OctreeVolume->GetMaxOctreeDepth();
	SnapshotData.VolumeBounds = OctreeVolume->VolumeBox->Bounds.GetBox();
	SnapshotData.MapSize = OctreeVolume->GetMapSize();
	SnapshotData.VoxelSize = OctreeVolume->VoxelExtent;

	// --- Step 2: Perform a high-speed bulk memory copy for the static node pool. ---
	const TArray<FOctreeNode>* SourceNodePoolPtr = OctreeVolume->GetNodePoolPtr();
	if (SourceNodePoolPtr && !SourceNodePoolPtr->IsEmpty())
	{
		SnapshotData.StaticNodePoolPtr = SourceNodePoolPtr;
	}

	// --- Step 3: Perform a high-speed bulk memory copy for the dynamic obstacle bitmask. ---
	const TArray<F128BitChunk>& SourceBitmask = OctreeVolume->GetDynamicObstacleBitmask();
	const int32 ChunkCount = SourceBitmask.Num();
	if (ChunkCount > 0)
	{
		// Pre-allocate memory without initializing elements.
		SnapshotData.DynamicObstacleBitmask.SetNumUninitialized(ChunkCount);

		// Copy the entire block of memory.
		FMemory::Memcpy(SnapshotData.DynamicObstacleBitmask.GetData(), SourceBitmask.GetData(),
		                ChunkCount * sizeof(F128BitChunk));
	}
}

bool UPathfindingComponent::RequestPathToTargetActor()
{
	if (!IsValid(TargetActor))
	{
		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Red, TEXT("TargetActor is not valid!"));
		return false;	
	}

	return RequestPathToLocation(TargetActor->GetActorLocation());
}

void UPathfindingComponent::SetTargetActor(AActor* NewTarget)
{
	TargetActor = NewTarget;
}

void UPathfindingComponent::StopMovement()
{
	FoundPath.Empty();
	PreferredVelocity = FVector::ZeroVector;
	CurrentVelocity = FVector::ZeroVector;
	bIsReachDestination = true;
}

AVoxelVolumeBase* UPathfindingComponent::GetCurrentVolume() const
{
	UWorld* World = GetWorld();
	if (!World)
	{
		return nullptr;
	}

	if (UOctreeNavSubsystem* NavSubsystem = World->GetSubsystem<UOctreeNavSubsystem>())
	{
		if (AActor* Owner = GetOwner())
		{
			return NavSubsystem->GetVolumeChunkAt(Owner->GetActorLocation());
		}
	}

	return nullptr;
}

// void UPathfindingComponent::ActivateStreaming()
// {
// 	// Currently called in BeginPlay, but it could also be called the moment pathfinding starts.
// 	// However, we must consider that this might cause issues due to loading times.```
//
// 	if (StreamingSourceComponent)
// 	{
// 		StreamingSourceComponent->EnableStreamingSource();
// 	}
// }
//
// void UPathfindingComponent::DeactivateStreaming()
// {
// 	if (IsValid(StreamingSourceComponent))
// 	{
// 		StreamingSourceComponent->DisableStreamingSource();
// 	}
// }

//======================================================================================
// Avoidance Interface
//======================================================================================

#if WITH_EDITOR

void UPathfindingComponent::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);

	const FName PropertyName = (PropertyChangedEvent.Property != nullptr)
		                           ? PropertyChangedEvent.Property->GetFName()
		                           : NAME_None;

	if (PropertyName == GET_MEMBER_NAME_CHECKED(UPathfindingComponent, BaseAgentRadius))
	{
		UpdateAgentRadiusFromScale(true);
	}
}

#endif

FVector UPathfindingComponent::GetPreferredVelocity() const
{
	if (!GetOwner() || bIsReachDestination)
	{
		return FVector::ZeroVector;
	}

	return PreferredVelocity;
}

void UPathfindingComponent::ApplyMovement(const FVector& FinalVelocity, float DeltaTime)
{
	AActor* Owner = GetOwner();
	if (!Owner || FoundPath.IsEmpty() || bIsReachDestination)
	{
		return;
	}
	
	// 1. Simple Move: Move with the given velocity without prediction or complex calculations.
	const FVector MoveDelta = FinalVelocity * DeltaTime;
	FHitResult HitResult;
	Owner->AddActorWorldOffset(MoveDelta, true, &HitResult);

	AdjustProgressToSpline(MoveDelta);
	if (!MoveDelta.IsNearlyZero())
	{
		const FRotator CurrentRotation = Owner->GetActorRotation();
		const FRotator TargetRotation = MoveDelta.ToOrientationRotator();
		FRotator InterpolatedRotation = FMath::RInterpTo(CurrentRotation, TargetRotation, DeltaTime, 5.f);
    
		Owner->SetActorRotation(InterpolatedRotation);
	}
	
	
	// 2. Physical Collision Handling: Declare an emergency only upon actual impact.
	if (HitResult.bBlockingHit)
	{
		UE_LOG(LogTemp, Warning, TEXT("Movement physically blocked by %s. Invalidating path."), *HitResult.GetActor()->GetName());
		InvalidatePathAndRequestRepath();
		return;
	}

	// 3. [KEY] Arrival Check: Based on the 'current' position after moving, check if we are close enough to the target.
	//    A loop is used to perfectly handle cases where multiple waypoints are passed in a single frame.
	
	while (!bIsReachDestination && FoundPath.IsValidIndex(CurrentPathDestinationIndex) && CurrentPathDestinationIndex > 0)
	{
		const FVector& TargetWaypoint = FoundPath[CurrentPathDestinationIndex];
		const FVector& PrevWaypoint = FoundPath[CurrentPathDestinationIndex - 1];

		// **Vector representing the direction of the current path segment.**
		const FVector PathSegmentDirection = (TargetWaypoint - PrevWaypoint).GetSafeNormal();

		// **Vector from the target waypoint to the agent's current location.**
		const FVector VectorToAgent = Owner->GetActorLocation() - TargetWaypoint;
		
		// **Use the dot product to check if the agent has passed the target waypoint along the path's direction.**
		// **A non-negative result means the agent is at or past the waypoint.**
		if (FVector::DotProduct(VectorToAgent, PathSegmentDirection) >= 0.0f)
		{
			// The target was passed, so proceed to the next waypoint.
			ProgressNextPoint();
		}
		else
		{
			// The target has not been passed yet, so stop checking for this frame.
			break;
		}
	}
}

void UPathfindingComponent::AdjustProgressToSpline(const FVector& MoveDelta)
{
	if (!IsValid(PathSplinePtr) || PathSplinePtr->GetNumberOfSplinePoints() < 2)
	{
		return;
	}
	FVector SplineTangent = PathSplinePtr->GetDirectionAtDistanceAlongSpline(DistanceAlongSpline, ESplineCoordinateSpace::World).GetSafeNormal();
	float ForwardDist = FVector::DotProduct(MoveDelta, SplineTangent);

	if (ForwardDist > 0.f)
	{
		DistanceAlongSpline += ForwardDist;
	}
	
	//DistanceAlongSpline += FinalVelocity.Length();
	
	// const FVector SplineDirection = PathSplinePtr->GetDirectionAtDistanceAlongSpline(
	// 		DistanceAlongSpline, ESplineCoordinateSpace::World);
	// const float ForwardDistance = FVector::DotProduct(FinalVelocity, SplineDirection);
	// DistanceAlongSpline += ForwardDistance;
	// const float SplineLength = PathSplinePtr->GetSplineLength();
	// DistanceAlongSpline = FMath::Clamp(DistanceAlongSpline, 0.f, SplineLength);
}

void UPathfindingComponent::ProgressNextPoint()
{
	// Don't wrap this in an if statement, as partialSplineUpdate is needed even for Linear movement.
	PartialSplinePointUpdate(CurrentPathDestinationIndex);
	
	if (CurrentPathDestinationIndex + 1 >= FoundPath.Num())
	{
		const FVector LastPathPoint = FoundPath.Last();
		const float DistanceToFinalGoalSq = FVector::DistSquared(LastPathPoint, CurrentTargetLocation);

		if (DistanceToFinalGoalSq <= FMath::Square(GoalUpdateThreshold))
		{
			UE_LOG(LogTemp, Log, TEXT("Reached the final destination. Stopping movement."));
			OnDestinationReached.Broadcast();
			StopMovement();
			return;
		}
		else
		{
			UE_LOG(LogTemp, Warning, TEXT("Reached end of a partial path. Requesting next path segment immediately."));

			FoundPath.Empty();
			PreferredVelocity = FVector::ZeroVector;
			CurrentVelocity = FVector::ZeroVector;

			RequestPathToLocation(CurrentTargetLocation);
			return;
		}
	}
	
	DetermineMovementModeForNextPath();

	++CurrentPathDestinationIndex;
}

bool UPathfindingComponent::IsOnSplineLine() const
{
	AActor* MyActor = GetOwner();
	if (!MyActor)
	{
		return false;
	}

	if (!IsValid(PathSplinePtr))
	{
		return false;
	}

	const FVector MyLocation = MyActor->GetActorLocation();
	const FVector SplineLocation = PathSplinePtr->GetLocationAtDistanceAlongSpline(
		DistanceAlongSpline, ESplineCoordinateSpace::World);

	const float MyTolerance = 100.0f;

	if (MyLocation.Equals(SplineLocation, MyTolerance))
	{
		return true;
	}

	return false;
}

void UPathfindingComponent::DetermineMovementModeForNextPath()
{
	if (IsSplinePathSafe())
	{
		SetMovementMode(EPathMovementMode::Spline);
	}
	else
	{
		SetMovementMode(EPathMovementMode::Linear);
	}
}

void UPathfindingComponent::PeriodicRepath()
{
	if (IsValid(TargetActor))
	{
		CurrentTargetLocation = TargetActor->GetActorLocation();
	}
	else
	{
		return;
	}

	RequestPathToLocation(CurrentTargetLocation);
}

void UPathfindingComponent::UpdateSteering(float DeltaTime)
{
	UpdatePreferredVelocity(DeltaTime);
}

void UPathfindingComponent::PartialSplinePointUpdate(int32 StartIndex)
{
	if (PathSplinePtr->GetNumberOfSplinePoints() <= 2)
	{
		return;
	}

	if (StartIndex > 1)
	{
		PathSplinePtr->RemoveSplinePoint(0);
	}
	
	DistanceAlongSpline = PathSplinePtr->GetDistanceAlongSplineAtSplinePoint(1);

	int32 AddIndex = StartIndex + PartialPointSplineNum - 1;

	if (AddIndex >= FoundPath.Num())
	{
		return;
	}

	PathSplinePtr->AddSplinePoint(FoundPath[AddIndex], ESplineCoordinateSpace::World, false);
	PathSplinePtr->UpdateSpline();
}
/**
 * 
void UPathfindingComponent::HandleWorldPathResult(const TArray<FVector>& Path, bool bSuccess)
{
	if (!IsValid(PathSplinePtr))
	{
		UE_LOG(LogTemp, Warning, TEXT("PathSplinePtr is not valid. Cannot set up spline path."));
		return;
	}

	if (!bSuccess || Path.IsEmpty())
	{
		UE_LOG(LogTemp, Warning, TEXT("Pathfinding failed or path is empty."));
		if (bPeriodicRepath)
		{
			GetWorld()->GetTimerManager().SetTimer(RepathCooldownTimer, this, &UPathfindingComponent::PeriodicRepath, RepathCooldown, false);
		}
		return;
	}
	
	if (bSuccess && Path.Num() > 0)
	{
		// if (IsPathObstructed(GetOwner()->GetActorLocation(), Path[0]))
		// {
		// 	UE_LOG(LogTemp, Warning, TEXT("New path is obstructed from actor's current location. Requesting a new path immediately."));
		// 	RequestPathToLocation(CurrentTargetLocation);
		// 	return;
		// }
		
		// *** Case: Path is safe ***
		FoundPath = Path;
		if (FVector::DistSquared(GetOwner()->GetActorLocation(), FoundPath[0]) < FMath::Square(10.0f))
		{
			CurrentPathDestinationIndex = 0;
		}
		else
		{
			FoundPath.Insert(GetOwner()->GetActorLocation(), 0);
			CurrentPathDestinationIndex = 1;
		}

		bIsReachDestination = false;
		CurrentState = EPathfindingState::FollowPath;
		CurrentPathDestinationIndex = 1; // Set to start following from the second point of the path (index 1).
		
		// Keep the existing Spline and debug drawing logic
		PathSplinePtr->ClearSplinePoints();
		for (int i = 0; i < PartialPointSplineNum; i++)
		{
			if (i >= FoundPath.Num()) break;
			PathSplinePtr->AddSplinePoint(FoundPath[i], ESplineCoordinateSpace::World, false);
		}
		PathSplinePtr->UpdateSpline();
		DistanceAlongSpline = 0.f;

	
		DetermineMovementModeForNextPath();
		
		if (bDrawDebugPath)
		{
			DrawDebugPath();
		}
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("Pathfinding failed or path is too short."));
	}

	if (bPeriodicRepath)
	{
		if (UWorld* World = GetWorld())
		{
			World->GetTimerManager().SetTimer(RepathCooldownTimer, this, &UPathfindingComponent::PeriodicRepath,
											  RepathCooldown, false);
		}
	}
}
 */
void UPathfindingComponent::HandleWorldPathResult(const TArray<FVector>& Path, bool bSuccess)
{
	if (!IsValid(PathSplinePtr))
	{
		UE_LOG(LogTemp, Warning, TEXT("PathSplinePtr is not valid. Cannot set up spline path."));
		return;
	}

	if (!bSuccess || Path.IsEmpty())
	{
		UE_LOG(LogTemp, Warning, TEXT("Pathfinding failed or path is empty. Retrying if periodic repath is enabled."));
		if (bPeriodicRepath)
		{
			GetWorld()->GetTimerManager().SetTimer(RepathCooldownTimer, this, &UPathfindingComponent::PeriodicRepath, RepathCooldown, false);
		}
		return;
	}

	// **--- Core Repath Logic: Find the most optimal point on the new path to join ---**
	// **This prevents the agent from turning back to the start of the newly generated path.**
	
	const FVector CurrentLocation = GetOwner()->GetActorLocation();
	TArray<FVector> NewPath;


	int32 FinalStartIndex = -1;

	for (int32 i = 1; i < Path.Num(); ++i)
	{
		if (!IsPathObstructed(CurrentLocation, Path[i]))
		{
			FinalStartIndex = i;
			break;
		}
	}

	NewPath.Add(CurrentLocation);

	if (FinalStartIndex != -1)
	{
		for (int32 i = FinalStartIndex; i < Path.Num(); ++i)
		{
			NewPath.Add(Path[i]);
		}
	}
	else
	{
		NewPath.Append(Path);
	}

	FoundPath = NewPath;

	bIsReachDestination = false;
	CurrentState = EPathfindingState::FollowPath;
	CurrentPathDestinationIndex = 1;


	PathSplinePtr->ClearSplinePoints();
	for (int i = 0; i < PartialPointSplineNum; i++)
	{
		if (i >= FoundPath.Num()) break;
		PathSplinePtr->AddSplinePoint(FoundPath[i], ESplineCoordinateSpace::World, false);
	}
	PathSplinePtr->UpdateSpline();
	DistanceAlongSpline = 0.f;

	DetermineMovementModeForNextPath();

	if (bDrawDebugPath)
	{
		DrawDebugPath();
	}

	if (bPeriodicRepath)
	{
		if (UWorld* World = GetWorld())
		{
			World->GetTimerManager().SetTimer(RepathCooldownTimer, this, &UPathfindingComponent::PeriodicRepath,
											  RepathCooldown, false);
		}
	}
}

void UPathfindingComponent::UpdateAgentRadiusFromScale(bool bForceUpdate)
{
	AActor* Owner = GetOwner();
	if (!Owner) return;

	const FVector CurrentScale = Owner->GetActorScale3D();

	if (!bForceUpdate && CurrentScale == LastKnownScale && BaseAgentRadius == LastKnownBaseRadius)
		return;

	LastKnownScale = CurrentScale;
	LastKnownBaseRadius = BaseAgentRadius;

	const float MaxScale = CurrentScale.GetMax();
	AgentRadius = BaseAgentRadius * MaxScale;

	if (DebugSphereComponent)
	{
		DebugSphereComponent->SetSphereRadius(AgentRadius);
	}
}

//======================================================================================
// Internal Logic
//======================================================================================

void UPathfindingComponent::HandlePathfindingResult()
{
	if (bPeriodicRepath && !bIsReachDestination)
	{
		if (UWorld* World = GetWorld())
		{
			World->GetTimerManager().SetTimer(RepathCooldownTimer, this, &UPathfindingComponent::PeriodicRepath,
			                                  RepathCooldown, false);
		}
	}
}

void UPathfindingComponent::InvalidatePathAndRequestRepath()
{
	if (CurrentState == EPathfindingState::PathInvalidated) return;

	UE_LOG(LogTemp, Warning, TEXT("Path Invalidated! Stopping movement and scheduling a repath..."));

	if (UWorld* World = GetWorld())
	{
		World->GetTimerManager().ClearTimer(RepathCooldownTimer);
	}

	RequestPathToLocation(CurrentTargetLocation);
	// End Test
}

// 
bool UPathfindingComponent::IsPathObstructed(const FVector& Start, const FVector& End, bool bIsPortalApproach) const
{
	AOctreeVoxelVolume* OctreeVolume = Cast<AOctreeVoxelVolume>(GetCurrentVolume());
	if (!OctreeVolume)
	{
		return true;
	}

	if (AgentRadius <= 0.f)
	{
		return false;
	}

	const FVector PathVector = End - Start;
	const float TotalDistance = PathVector.Size();

	const float StepSize = AgentRadius * 0.5f;

	if (TotalDistance < StepSize)
	{
		return OctreeVolume->IsLocationInBlockedVoxel(End);
	}

	const FVector Direction = PathVector.GetSafeNormal();
	
	for (float Distance = 0; Distance < TotalDistance; Distance += StepSize)
	{
		const FVector CenterPosition = Start + Direction * Distance;
		
		if (bIsPortalApproach)
		{
			if (OctreeVolume->IsLocationInBlockedVoxel(CenterPosition))
			{
				return true;
			}
		}
		else
		{
			// Normal path: Perform offset tests considering the agent radius as before.
			const FVector TestOffsets[] = {
				FVector::ZeroVector,
				FVector(0, AgentRadius, 0),  // Left
				FVector(0, -AgentRadius, 0), // Right
				FVector(0, 0, AgentRadius),  // Up
				FVector(0, 0, -AgentRadius)
			};

			for (const FVector& Offset : TestOffsets)
			{
				if (OctreeVolume->IsLocationInBlockedVoxel(CenterPosition + Offset))
				{
					return true;
				}
			}
		}
	}

	if (OctreeVolume->IsLocationInBlockedVoxel(End))
	{
		return true;
	}

	return false;
}

void UPathfindingComponent::DrawDebugPath()
{
	UWorld* World = GetWorld();
	if (!World || FoundPath.Num() < 2)
	{
		return;
	}

	const float Duration = 1.f;
	const float LineThickness = 5.0f;
	const float BoxThickness = AgentRadius * 0.7;
	const FVector BoxHalfSize(AgentRadius * 0.05);

	const FLinearColor PathColor(0.5f, 0.0f, 1.0f);
	const FLinearColor PathPointColor(.5f, 0.5f, 1.0f);

	for (int32 i = 0; i < FoundPath.Num() - 1; ++i)
	{
		float T = static_cast<float>(i) / (FoundPath.Num() - 2);

		DrawDebugLine(
			World,
			FoundPath[i],
			FoundPath[i + 1],
			PathColor.ToFColor(true),
			false,
			Duration,
			0,
			LineThickness
		);
	}

	for (int32 i = 0; i < FoundPath.Num(); ++i)
	{
		float T = static_cast<float>(i) / (FoundPath.Num() - 1);

		DrawDebugBox(
			World,
			FoundPath[i],
			BoxHalfSize,
			PathPointColor.ToFColor(true),
			false,
			Duration,
			0,
			BoxThickness
		);
	}
}

void UPathfindingComponent::UpdatePreferredVelocity(float DeltaTime)
{
	if (GetMovementMode() == EPathMovementMode::Spline)
	{
		UpdateSplinePreferredVelocity(DeltaTime);
	}
	else
	{
		UpdateLinearPreferredVelocity(DeltaTime);
	}
}

void UPathfindingComponent::UpdateLinearPreferredVelocity(float DeltaTime)
{
	AActor* Actor = GetOwner();
	if (!Actor || CurrentPathDestinationIndex < 0 || CurrentPathDestinationIndex >= FoundPath.Num())
	{
		return;
	}

	const FVector CurrentLocation = Actor->GetActorLocation();
	const FVector TargetPoint = FoundPath[CurrentPathDestinationIndex];

	const FVector Direction = (TargetPoint - CurrentLocation).GetSafeNormal();
	PreferredVelocity = Direction * MovementSpeed;
}

void UPathfindingComponent::UpdateSplinePreferredVelocity(float DeltaTime)
{
	AActor* Owner = GetOwner();
	if (!Owner || !IsValid(PathSplinePtr) || PathSplinePtr->GetNumberOfSplinePoints() < 2)
	{
		PreferredVelocity = FVector::ZeroVector;
		return;
	}

	const FVector CurrentLocation = Owner->GetActorLocation();

	float ClosestInputKey = PathSplinePtr->FindInputKeyClosestToWorldLocation(CurrentLocation);

	float CurrentDistanceOnSpline = PathSplinePtr->GetDistanceAlongSplineAtSplineInputKey(ClosestInputKey);

	float LookAheadDistance = CurrentDistanceOnSpline + 50.0f;

	const FVector SplineDirection = PathSplinePtr->GetDirectionAtDistanceAlongSpline(LookAheadDistance, ESplineCoordinateSpace::World);

	PreferredVelocity = SplineDirection * MovementSpeed;
}

bool UPathfindingComponent::IsSplinePathSafe() const
{
	// Using this breaks HPA* (Hierarchical Pathfinding A*).
	AOctreeVoxelVolume* VoxelVolume = Cast<AOctreeVoxelVolume>(GetCurrentVolume());
	if (!IsValid(PathSplinePtr) || !IsValid(VoxelVolume))
	{
		return true;
	}

	const float SplineLength = PathSplinePtr->GetSplineLength();

	if (SplineLength < SafetyCheckStepSize)
	{
		if (SplineLength > KINDA_SMALL_NUMBER && VoxelVolume->IsLocationInBlockedVoxel(
			PathSplinePtr->GetLocationAtDistanceAlongSpline(SplineLength, ESplineCoordinateSpace::World)))
		{
			UE_LOG(LogTemp, Warning, TEXT("Spline path endpoint is inside a blocked voxel. Path is unsafe."));
			return false;
		}
		return true;
	}

	// Begin Test
	const float CurrentDistance = PathSplinePtr->GetDistanceAlongSplineAtSplinePoint(CurrentPathDestinationIndex - 1);
	const float NextDistance = PathSplinePtr->GetDistanceAlongSplineAtSplinePoint(CurrentPathDestinationIndex);
	
	for (float Distance = CurrentDistance; Distance <= NextDistance; Distance += SafetyCheckStepSize)
	{
		const FVector CenterLocation = PathSplinePtr->GetLocationAtDistanceAlongSpline(
			Distance, ESplineCoordinateSpace::World);
	
		const TArray<FVector> TestOffsets = {
			FVector::ZeroVector,
			FVector(AgentRadius, 0.f, 0.f),
			FVector(-AgentRadius, 0.f, 0.f),
			FVector(0.f, AgentRadius, 0.f),
			FVector(0.f, -AgentRadius, 0.f),
			FVector(0.f, 0.f, AgentRadius),
			FVector(0.f, 0.f, -AgentRadius)
		};
	
		for (const FVector& Offset : TestOffsets)
		{
			const FVector TestLocation = CenterLocation + Offset;
	
			if (VoxelVolume->IsLocationInBlockedVoxel(TestLocation))
			{
				return false;
			}
		} 
	}
	// End Test

	return true;
}
