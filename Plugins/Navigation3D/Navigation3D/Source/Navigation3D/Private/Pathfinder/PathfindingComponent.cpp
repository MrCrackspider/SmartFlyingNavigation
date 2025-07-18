#include "Pathfinder/PathfindingComponent.h"
#include "Components/SplineComponent.h"
#include "Pathfinder/Subsystem/PathfindingSubsystem.h"
#include "Pathfinder/Subsystem/AvoidanceSubsystem.h" 
#include "Pathfinder/Strategy/PathfindingStrategy.h"
#include "GameFramework/Actor.h"
#include "Engine/World.h"
#include "DrawDebugHelpers.h"
#include "Components/BoxComponent.h"
#include "Components/SphereComponent.h"
#include "Pathfinder/NavVolume/OctreeVoxelVolume.h"
#include "Engine/Engine.h" 
#include "Pathfinder/Strategy/AStarStrategy.h"

UPathfindingComponent::UPathfindingComponent()
{
	PrimaryComponentTick.bCanEverTick = true;

#if WITH_EDITOR
	bTickInEditor = true;
#endif
	
	PathSplinePtr = CreateDefaultSubobject<USplineComponent>(TEXT("Spline"));

	PathfindingStrategyClass = UAStarStrategy::StaticClass();

	DebugSphereComponent = CreateDefaultSubobject<USphereComponent>(TEXT("DebugAgentRadiusSphere"));

	DebugSphereComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	DebugSphereComponent->SetCanEverAffectNavigation(false);
	DebugSphereComponent->bHiddenInGame = true;
	DebugSphereComponent->SetSphereRadius(BaseAgentRadius);
	
}

void UPathfindingComponent::BeginPlay()
{
	Super::BeginPlay();

	UWorld* World = GetWorld();
	if (!ensure(World)) return;
	
	PathSubsystem = GEngine->GetEngineSubsystem<UPathfindingSubsystem>();
	AvoidanceSubsystem = World->GetSubsystem<UAvoidanceSubsystem>();

	if (!ensure(PathSubsystem))
	{
		UE_LOG(LogTemp, Error, TEXT("PathfindingComponent: PathfindingSubsystem not found!"));
		return;
	}

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
	else
	{
		UE_LOG(LogTemp, Error, TEXT("PathfindingComponent on Actor '%s': PathfindingStrategyClass is not set!"),
			   *GetOwner()->GetName());
	}
}

void UPathfindingComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// Called to update pathfinding in response to a scale change at runtime.
	UpdateAgentRadiusFromScale(); 
	
	if (CurrentState == EPathfindingState::FollowPath)
	{
		if (!IsPathAheadValid())
		{
			UE_LOG(LogTemp, Warning, TEXT("Dynamic obstacle is on path. Start Repath"));
			InvalidatePathAndRequestRepath();
			return;
		}
	}
	
	if (PathfindingTaskFuture.IsValid())
	{
		HandlePathfindingResult();
	}
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

	Super::EndPlay(EndPlayReason);
}

void UPathfindingComponent::OnRegister()
{
	Super::OnRegister();
	
	if (DebugSphereComponent && GetOwner() && GetOwner()->GetRootComponent())
	{
		DebugSphereComponent->AttachToComponent(GetOwner()->GetRootComponent(), FAttachmentTransformRules::SnapToTargetNotIncludingScale);
	}
}

bool UPathfindingComponent::RequestPathToLocation(const FVector& TargetLocation)
{
	if (PathfindingTaskFuture.IsValid() && !PathfindingTaskFuture.IsReady())
	{
		UE_LOG(LogTemp, Warning, TEXT("RequestPathToLocation: Already requesting a path. New request ignored."));
		return false;
	}

	if (!PathfindingStrategy || !TargetVolume)
	{
		UE_LOG(LogTemp, Error, TEXT("RequestPathToLocation failed: Strategy or TargetVolume is not set."));
		return false;
	}
	
	if (UWorld* World = GetWorld())
	{
		World->GetTimerManager().ClearTimer(RepathCooldownTimer);
	}

	FPathfindingRequest Request;
	Request.RequestingObject = this;
	Request.StartLocation = GetOwner()->GetActorLocation();
	Request.EndLocation = TargetLocation;
	Request.AgentRadius = this->AgentRadius;
	Request.Strategy = PathfindingStrategy;
	bool bRequestDataValid = false;

	if (AOctreeVoxelVolume* OctreeVolume = Cast<AOctreeVoxelVolume>(TargetVolume))
	{
		Request.NodeType = EPathfindingNodeType::Octree;
		Request.OnWorldPathComplete.BindDynamic(this, &UPathfindingComponent::HandleWorldPathResult);

		// --- 3. Create a thread-safe data snapshot using the optimized helper function. ---
		FOctreePathfindingData OctreeSnapshot;
		CreateOctreeDataSnapshot(OctreeVolume, OctreeSnapshot);
        
		// Move the created snapshot into the request payload.
		Request.NavigationData.Emplace<FOctreePathfindingData>(MoveTemp(OctreeSnapshot));
		bRequestDataValid = true;
	}

	// --- 4. Dispatch the request if the data is valid. ---
	if (bRequestDataValid)
	{
		CurrentTargetLocation = TargetLocation;
		FoundPath.Empty();

		CurrentRequestID = PathSubsystem->RequestPath(Request);
		if (CurrentRequestID != -1)
		{
			return true; // Request successfully queued.
		}
	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT("RequestPathToLocation failed: TargetVolume is not a supported type (currently only Octree is implemented here)."));
	}
	
	return false;
}

void UPathfindingComponent::CreateOctreeDataSnapshot(const AOctreeVoxelVolume* OctreeVolume, FOctreePathfindingData& SnapshotData) const
{
	// --- Step 1: Copy small, trivial member variables directly. ---
	SnapshotData.RootNodeIndex = OctreeVolume->GetRootNodeIndex();
	SnapshotData.MaxOctreeDepth = OctreeVolume->GetMaxOctreeDepth();
	SnapshotData.VolumeBounds = OctreeVolume->VolumeBox->Bounds.GetBox();
	SnapshotData.MapSize = OctreeVolume->MapSize;
	SnapshotData.VoxelSize = OctreeVolume->VoxelExtent;

	// --- Step 2: Perform a high-speed bulk memory copy for the static node pool. ---
	const TArray<FOctreeNode>& SourceNodePool = OctreeVolume->GetStaticNodePool();
	const int32 NodeCount = SourceNodePool.Num();
	if (NodeCount > 0)
	{
		// Pre-allocate memory without initializing elements to avoid overhead.
		SnapshotData.StaticNodePool.SetNumUninitialized(NodeCount);
        
		// Copy the entire block of memory from the source array to the destination.
		FMemory::Memcpy(SnapshotData.StaticNodePool.GetData(), SourceNodePool.GetData(), NodeCount * sizeof(FOctreeNode));
	}

	// --- Step 3: Perform a high-speed bulk memory copy for the dynamic obstacle bitmask. ---
	const TArray<F128BitChunk>& SourceBitmask = OctreeVolume->GetDynamicObstacleBitmask();
	const int32 ChunkCount = SourceBitmask.Num();
	if (ChunkCount > 0)
	{
		// Pre-allocate memory without initializing elements.
		SnapshotData.DynamicObstacleBitmask.SetNumUninitialized(ChunkCount);
        
		// Copy the entire block of memory.
		FMemory::Memcpy(SnapshotData.DynamicObstacleBitmask.GetData(), SourceBitmask.GetData(), ChunkCount * sizeof(F128BitChunk));
	}
}

bool UPathfindingComponent::RequestPathToTargetActor()
{
	if (!IsValid(TargetActor))
	{
		// UE_LOG(LogPathfinding, Warning, TEXT("RequestPathToTargetActor failed: TargetActor is not set or invalid."));
		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Red, TEXT("TargetActor is not valid!"));
		return false;
	}

	return RequestPathToLocation(TargetActor->GetActorLocation());
}

void UPathfindingComponent::SetTargetActor(AActor* NewTarget)
{
	TargetActor = NewTarget;
}

//======================================================================================
// Avoidance Interface
//======================================================================================

#if WITH_EDITOR
void UPathfindingComponent::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);

	const FName PropertyName = (PropertyChangedEvent.Property != nullptr) ? PropertyChangedEvent.Property->GetFName() : NAME_None;

	if (PropertyName == GET_MEMBER_NAME_CHECKED(UPathfindingComponent, BaseAgentRadius))
	{
		UpdateAgentRadiusFromScale(true);
	}
}

#endif

FVector UPathfindingComponent::GetPreferredVelocity() const
{
	if (CurrentState != EPathfindingState::FollowPath || !GetOwner() || bIsReachDestination)
	{
		return FVector::ZeroVector;
	}

	return PreferredVelocity;
}

void UPathfindingComponent::ApplyMovement(const FVector& FinalVelocity, float DeltaTime)
{
	AActor* Owner = GetOwner();
	if (!Owner || !PathSplinePtr.IsValid() || FoundPath.IsEmpty()) return;

	// 1. Actual movement (same as before)
	const FVector CurrentLocation = Owner->GetActorLocation();
	CurrentVelocity = FinalVelocity;
	const FVector AdjustedVelocity = FinalVelocity * DeltaTime;
	Owner->SetActorLocation(CurrentLocation + AdjustedVelocity);

	// --- Modified Rotation Logic ---
	if (!FinalVelocity.IsNearlyZero())
	{
		// Get the current rotation value
		const FRotator CurrentRotation = Owner->GetActorRotation();
        
		// Calculate the target rotation (to look in the direction of movement)
		FRotator TargetRotation = FinalVelocity.GetSafeNormal().ToOrientationRotator();

		// [Important] Apply Axis Lock to the target rotation first.
		// For a locked axis, maintain the current rotation value,
		// preventing rotation interpolation on that axis.
		TargetRotation.Roll = RotationAxisLock[0] ? CurrentRotation.Roll : TargetRotation.Roll;
		TargetRotation.Pitch = RotationAxisLock[1] ? CurrentRotation.Pitch : TargetRotation.Pitch;
		TargetRotation.Yaw = RotationAxisLock[2] ? CurrentRotation.Yaw : TargetRotation.Yaw;

		// [Core] Use RInterpTo to smoothly interpolate from the current rotation to the target rotation.
		// This function automatically calculates the shortest path between the two rotations.
		FRotator NewRotation = FMath::RInterpTo(CurrentRotation, TargetRotation, DeltaTime, RotationInterpSpeed);

		// Finally, apply the calculated smooth rotation value.
		Owner->SetActorRotation(NewRotation);
	}
	// --- End of Modified Rotation Logic ---

	AdjustProgressToSpline(AdjustedVelocity);
}

void UPathfindingComponent::AdjustProgressToSpline(FVector FinalVelocity)
{
	if (!PathSplinePtr.IsValid())
	{
		UE_LOG(LogTemp, Warning, TEXT("PathfindingComponent: TargetSpline is not set."));
		return;
	}

	const FVector SplineDirection = PathSplinePtr->GetDirectionAtDistanceAlongSpline(
		DistanceAlongSpline, ESplineCoordinateSpace::World);

	const float ForwardDistance = FVector::DotProduct(FinalVelocity, SplineDirection);
	DistanceAlongSpline += ForwardDistance;
	const float SplineLength = PathSplinePtr->GetSplineLength();
	DistanceAlongSpline = FMath::Clamp(DistanceAlongSpline, 0.f, SplineLength);

	if (!bIsReachDestination)
	{
		const FVector CurrentLocation = GetOwner()->GetActorLocation();
		const FVector TargetPoint = FoundPath[CurrentPathDestinationIndex];

		if (GetMovementMode() == EPathMovementMode::Spline)
		{
			const float TargetPointDistance = PathSplinePtr->GetDistanceAlongSplineAtSplinePoint(1);

			if (DistanceAlongSpline >= TargetPointDistance)
			{
				ProgressNextPoint();
			}
		}
		else
		{
			const float ArrivalThresholdSq = FMath::Square(10.f);
			if (FVector::DistSquared(CurrentLocation, TargetPoint) < ArrivalThresholdSq)
			{
				ProgressNextPoint();
			}
		}
	}
}

void UPathfindingComponent::ProgressNextPoint()
{
	++CurrentPathDestinationIndex;
	DistanceAlongSpline = 0;
	PartialSplinePointUpdate(CurrentPathDestinationIndex);
	
	if (CurrentPathDestinationIndex >= FoundPath.Num())
	{
		OnDestinationReached.Broadcast();
		bIsReachDestination = true;
		return;
	}

	DetermineMovementModeForNextPath();
}

bool UPathfindingComponent::IsOnSplineLine() const
{
	AActor* MyActor = GetOwner();
	if (!MyActor)
	{
		UE_LOG(LogTemp, Warning, TEXT("PathfindingComponent: Actor is not set."));
		return false;
	}

	if (!PathSplinePtr.IsValid())
	{
		UE_LOG(LogTemp, Error, TEXT("PathfindingComponent: PathSplinePtr is not set for %s!"), *MyActor->GetName());
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
		UE_LOG(LogTemp, Warning, TEXT("PeriodicRepath: DestinationActor is no longer valid. Stopping repath requests."));
		return;
	}
	
	RequestPathToLocation(CurrentTargetLocation);
}

void UPathfindingComponent::UpdateSteering(float DeltaTime)
{
	if (CurrentState == EPathfindingState::FollowPath)
	{
		if (!bIsReachDestination)
		{
			UpdatePreferredVelocity(DeltaTime);
		}
	}
}

void UPathfindingComponent::HandleVoxelPathResult(const TArray<FIntVector>& PathIndices, bool bSuccess)
{
	if (bSuccess && PathIndices.Num() > 0)
	{
		UE_LOG(LogTemp, Log, TEXT("Pathfinding successful for %s. State changed to FollowPath."), *GetOwner()->GetName());
	}
	else
	{
		CurrentState = EPathfindingState::Idle;
		UE_LOG(LogTemp, Warning, TEXT("Pathfinding failed for %s. State changed to Idle."), *GetOwner()->GetName());
	}

	if (bPeriodicRepath && !bIsReachDestination)
	{
		if (UWorld* World = GetWorld())
		{
			//Recursive Call PeriodicRepath
			World->GetTimerManager().SetTimer(RepathCooldownTimer, this, &UPathfindingComponent::PeriodicRepath, RepathCooldown, false);
		}
	}
}

void UPathfindingComponent::PartialSplinePointUpdate(int32 StartIndex)
{
	PathSplinePtr->ClearSplinePoints();

	PathSplinePtr->RemoveSplinePoint(0);
	
	int32 AddIndex = StartIndex + PartialPointSplineNum;

	if (AddIndex >= FoundPath.Num())
	{
		UE_LOG(LogTemp, Warning, TEXT("No more Spline Point!"));
		return;
	}
	
	PathSplinePtr->AddSplinePoint(FoundPath[AddIndex], ESplineCoordinateSpace::World, false);

	PathSplinePtr->UpdateSpline();
}

void UPathfindingComponent::HandleWorldPathResult(const TArray<FVector>& Path, bool bSuccess)
{
	if (bSuccess && Path.Num() > 1)
	{
		FoundPath = Path;
		FoundPathIndices.Empty();

		if (bContinuosFollowingWhenReached)
		{
			bIsReachDestination = false;
		}
		
		CurrentState = EPathfindingState::FollowPath;

		if (PathSplinePtr.IsValid())
		{
			PathSplinePtr->ClearSplinePoints();

			for (int i=0;i<3;i++)
			{
				if (i >= FoundPath.Num())
				{
					break;
				}
				
				PathSplinePtr->AddSplinePoint(FoundPath[i], ESplineCoordinateSpace::World, false);
			}
				
			PathSplinePtr->UpdateSpline();

			const FVector CurrentLocation = GetOwner()->GetActorLocation();
			const float ClosestInputKey = PathSplinePtr->FindInputKeyClosestToWorldLocation(CurrentLocation);
			DistanceAlongSpline = PathSplinePtr->GetDistanceAlongSplineAtSplineInputKey(ClosestInputKey);

			int32 ClosestPointIndex = FMath::RoundToInt(ClosestInputKey);
			CurrentPathDestinationIndex = FMath::Clamp(ClosestPointIndex + 1, 1, PartialPointSplineNum - 1);
		}
		else
		{
			UE_LOG(LogTemp, Warning, TEXT("PathSplinePtr is not valid. Cannot set up spline path."));
		}
	}
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("Pathfinding failed or path is too short."));
        FoundPath.Empty();
        FoundPathIndices.Empty();
        CurrentPathDestinationIndex = 0;
        CurrentState = EPathfindingState::Idle;
    }
	
    if (bPeriodicRepath && !bIsReachDestination)
    {
        if (UWorld* World = GetWorld())
        {
            World->GetTimerManager().SetTimer(RepathCooldownTimer, this, &UPathfindingComponent::PeriodicRepath, RepathCooldown, false);
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
	if (PathfindingTaskFuture.IsValid() && PathfindingTaskFuture.IsReady())
	{
		TArray<FIntVector> PathIndices = PathfindingTaskFuture.Get();
		PathfindingTaskFuture.Reset();
	}
	
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

	// Begin Test
	
	RequestPathToLocation(CurrentTargetLocation);

	// End Test
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
	AActor* MyActor = GetOwner();
	if (!MyActor || !PathSplinePtr.IsValid() || PathSplinePtr->GetNumberOfSplinePoints() < 2)
	{
		return;
	}

	const FVector SplineDirection = PathSplinePtr->GetDirectionAtDistanceAlongSpline(
		DistanceAlongSpline, ESplineCoordinateSpace::World);

	PreferredVelocity = SplineDirection * MovementSpeed;
}

bool UPathfindingComponent::IsSplinePathSafe() const
{
	if (!PathSplinePtr.IsValid() || !GetWorld())
	{
		return true;
	}

	const float SplineLength = PathSplinePtr->GetSplineLength();
	if (SplineLength < SafetyCheckStepSize)
	{
		return true;
	}

	FCollisionQueryParams CollisionParams(SCENE_QUERY_STAT(SplineSafetyCheck), false, GetOwner());
	
	FCollisionObjectQueryParams ObjectQueryParams;
	ObjectQueryParams.AddObjectTypesToQuery(ECollisionChannel::ECC_WorldStatic);
	ObjectQueryParams.AddObjectTypesToQuery(ECollisionChannel::ECC_WorldDynamic);
	ObjectQueryParams.AddObjectTypesToQuery(ECollisionChannel::ECC_PhysicsBody);

	const float CurrentDistance = PathSplinePtr->GetDistanceAlongSplineAtSplinePoint(CurrentPathDestinationIndex - 1);
	const float NextDistance = PathSplinePtr->GetDistanceAlongSplineAtSplinePoint(CurrentPathDestinationIndex);

	
	for (float Distance = CurrentDistance; Distance < NextDistance; Distance += SafetyCheckStepSize)
	{
		FVector StartPoint = PathSplinePtr->GetLocationAtDistanceAlongSpline(Distance, ESplineCoordinateSpace::World);
		FVector EndPoint = PathSplinePtr->GetLocationAtDistanceAlongSpline(
			FMath::Min(Distance + SafetyCheckStepSize, SplineLength), ESplineCoordinateSpace::World);

		if (FVector::DistSquared(StartPoint, EndPoint) < KINDA_SMALL_NUMBER)
		{
			UE_LOG(LogTemp, Warning, TEXT("Start End So Close"));
			continue;
		}

		
		if (GetWorld()->SweepTestByObjectType(StartPoint, EndPoint, FQuat::Identity, ObjectQueryParams,
		                                      FCollisionShape::MakeSphere(AgentRadius), CollisionParams))
		{
			UE_LOG(LogTemp, Warning, TEXT("Obstacle detected on spline path."));
			return false;
		}
	}

	return true;
}

bool UPathfindingComponent::IsPathAheadValid() const
{
	AOctreeVoxelVolume* OctreeVoxelVolume = Cast<AOctreeVoxelVolume>(TargetVolume);
	if (!OctreeVoxelVolume || FoundPathIndices.Num() < 2 || CurrentPathDestinationIndex < 0 || CurrentPathDestinationIndex
		>= FoundPathIndices.
		Num() - 1)
	{
		return true;
	}
	
	return true;
}
