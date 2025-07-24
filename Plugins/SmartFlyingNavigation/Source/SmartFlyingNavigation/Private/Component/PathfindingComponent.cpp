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

	if (TargetVolume.Get() != nullptr &&IsValid(TargetVolume.Get()))
	{
		GoalUpdateThreshold = TargetVolume->VoxelExtent;
	}
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
		return;
	}

	if (!PathSplinePtr)
	{
		PathSplinePtr->SetAbsolute(true, true, true);
		PathSplinePtr->SetupAttachment(GetOwner()->GetRootComponent());
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
}

void UPathfindingComponent::TickComponent(float DeltaTime, ELevelTick TickType,
                                          FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// Called to update pathfinding in response to a scale change at runtime.
	UpdateAgentRadiusFromScale();
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
		DebugSphereComponent->AttachToComponent(GetOwner()->GetRootComponent(),
		                                        FAttachmentTransformRules::SnapToTargetNotIncludingScale);
	}
}

bool UPathfindingComponent::RequestPathToLocation(const FVector& TargetLocation)
{
	if (!PathfindingStrategy || !TargetVolume)
	{
		return false;
	}
	
	if (!IsValid(PathSplinePtr))
	{
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
		CreateOctreeDataSnapshot(OctreeVolume, Request.NavigationData);

		bRequestDataValid = true;
	}

	// --- 4. Dispatch the request if the data is valid. ---
	if (bRequestDataValid)
	{
		CurrentTargetLocation = TargetLocation;

		CurrentRequestID = PathSubsystem->RequestPath(Request);
		if (CurrentRequestID != -1)
		{
			return true; // Request successfully queued.
		}
	}

	return false;
}

void UPathfindingComponent::CreateOctreeDataSnapshot(const AOctreeVoxelVolume* OctreeVolume,
                                                     FOctreePathfindingData& SnapshotData) const
{
	// --- Step 1: Copy small, trivial member variables directly. ---
	SnapshotData.RootNodeIndex = OctreeVolume->GetRootNodeIndex();
	SnapshotData.MaxOctreeDepth = OctreeVolume->GetMaxOctreeDepth();
	SnapshotData.VolumeBounds = OctreeVolume->VolumeBox->Bounds.GetBox();
	SnapshotData.MapSize = OctreeVolume->MapSize;
	SnapshotData.VoxelSize = OctreeVolume->VoxelExtent;

	// --- Step 2: Perform a high-speed bulk memory copy for the static node pool. ---
	const TArray<FOctreeNode>* SourceNodePoolPtr  = OctreeVolume->GetNodePoolPtr();
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
	if (CurrentState != EPathfindingState::FollowPath || !GetOwner() || bIsReachDestination)
	{
		return FVector::ZeroVector;
	}

	return PreferredVelocity;
}
 
void UPathfindingComponent::ApplyMovement(const FVector& FinalVelocity, float DeltaTime)
{
	AActor* Owner = GetOwner();
	if (!Owner || !IsValid(PathSplinePtr) || FoundPath.IsEmpty()) return;

	// 1. Actual movement (same as before)
	const FVector CurrentLocation = Owner->GetActorLocation();
	CurrentVelocity = FinalVelocity;
	const FVector AdjustedVelocity = FinalVelocity * DeltaTime;
	SetAppliedVelocity(CurrentVelocity );
	//Owner->SetActorLocation(CurrentLocation + AdjustedVelocity);

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
	if (!IsValid(PathSplinePtr))
	{
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

		FVector TargetPoint;
		if(FoundPath.IsValidIndex(CurrentPathDestinationIndex))
		{
			TargetPoint = FoundPath[CurrentPathDestinationIndex];
		}
		else
		{
			return;
		}

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
	PartialSplinePointUpdate(CurrentPathDestinationIndex);
	++CurrentPathDestinationIndex;

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
	if (!bSuccess || PathIndices.Num() == 0)
	{
		CurrentState = EPathfindingState::Idle;
	}

	if (bPeriodicRepath && !bIsReachDestination)
	{
		if (UWorld* World = GetWorld())
		{
			//Recursive Call PeriodicRepath
			World->GetTimerManager().SetTimer(RepathCooldownTimer, this, &UPathfindingComponent::PeriodicRepath,
			                                  RepathCooldown, false);
		}
	}
}

void UPathfindingComponent::PartialSplinePointUpdate(int32 StartIndex)
{
	if (PathSplinePtr->GetNumberOfSplinePoints() <= 2)
	{
		return;
	}
	
	DistanceAlongSpline = 0;
	
	PathSplinePtr->RemoveSplinePoint(0);

	int32 AddIndex = StartIndex + PartialPointSplineNum - 1;
	
	if (AddIndex >= FoundPath.Num())
	{
		return;
	}
	
	PathSplinePtr->AddSplinePoint(FoundPath[AddIndex], ESplineCoordinateSpace::World, false);
	PathSplinePtr->UpdateSpline();
}

void UPathfindingComponent::HandleWorldPathResult(const TArray<FVector>& Path, bool bSuccess)
{
	if (!IsValid(PathSplinePtr))
	{
		UE_LOG(LogTemp, Warning, TEXT("PathSplinePtr is not valid. Cannot set up spline path."));
		return;
	}
	
	if (bSuccess && Path.Num() > 1)
	{
		FoundPath = Path;

		if (bContinuosFollowingWhenReached)
		{
			bIsReachDestination = false;
		}

		CurrentState = EPathfindingState::FollowPath;
		
		PathSplinePtr->ClearSplinePoints();

		for (int i = 0; i < PartialPointSplineNum; i++)
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
		CurrentPathDestinationIndex = FMath::Clamp(ClosestPointIndex + 1, 0, PartialPointSplineNum - 1);
		
		UWorld* World = GetWorld();
		if (World && bDrawDebugPath) 
		{
			for (int32 i = 0; i < FoundPath.Num() - 1; ++i)
			{
				DrawDebugLine(
					World,
					FoundPath[i],    
					FoundPath[i + 1],
					FColor::Green,   
					true,            
					5.0f,            
					0,               
					5.0f             
				);
			}
			
			for (const FVector& Point : FoundPath)
			{
				DrawDebugSphere(
					World,
					Point,
					15.0f,             
					12,                
					FColor::Cyan,      
					true,
					5.0f
				);
			}
		}
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("Pathfinding failed or path is too short."));
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
	if (!MyActor || !IsValid(PathSplinePtr) || PathSplinePtr->GetNumberOfSplinePoints() < 2)
	{
		return;
	}

	const FVector SplineDirection = PathSplinePtr->GetDirectionAtDistanceAlongSpline(
		DistanceAlongSpline, ESplineCoordinateSpace::World);

	PreferredVelocity = SplineDirection * MovementSpeed;
}

bool UPathfindingComponent::IsSplinePathSafe() const
{
	if (!IsValid(PathSplinePtr) || !GetWorld())
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
