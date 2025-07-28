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

	if (TargetVolume.Get() != nullptr && IsValid(TargetVolume.Get()))
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
	AvoidanceSubsystem = GEngine->GetEngineSubsystem<UAvoidanceSubsystem>();

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

	if (!IsValid(AvoidanceSubsystem) || !AvoidanceSubsystem->GetbUseAvoidance())
	{
		UpdateSteering(DeltaTime);
		ApplyMovement(PreferredVelocity, DeltaTime);
	}
	
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
	if (!Owner || FoundPath.IsEmpty())
	{
		return;
	}

	CurrentVelocity = FinalVelocity;
	FVector RemainingMoveDelta = FinalVelocity * DeltaTime; // Total movement delta for this frame

	// Loop as long as there is remaining movement delta and a valid path to follow
	while (FoundPath.IsValidIndex(CurrentPathDestinationIndex) && !RemainingMoveDelta.IsNearlyZero())
	{
		const FVector CurrentLocation = Owner->GetActorLocation();
		const FVector TargetPoint = FoundPath[CurrentPathDestinationIndex];
		FVector VectorToTarget = TargetPoint - CurrentLocation;
		const float DistToTargetSq = VectorToTarget.SizeSquared();

		FVector MoveThisStep;
		bool bReachedTargetThisStep = false;

		if (RemainingMoveDelta.SizeSquared() > DistToTargetSq)
		{
			// If the movement for this step overshoots the next target point
			MoveThisStep = VectorToTarget;
			RemainingMoveDelta -= VectorToTarget;
			bReachedTargetThisStep = true;
		}
		else
		{
			// If not overshooting the target point
			MoveThisStep = RemainingMoveDelta;
			RemainingMoveDelta = FVector::ZeroVector;
		}

		// --- Core Change: Actually move the actor and check for collision results ---
		FHitResult HitResult;
		Owner->SetActorLocation(CurrentLocation + MoveThisStep, true, &HitResult);

		if (GetMovementMode() == EPathMovementMode::Spline)
		{
			AdjustProgressToSpline(CurrentVelocity); // 실제 적용된 속도(CurrentVelocity)를 기준으로 진행 상황을 기록합니다.
		}
		if (HitResult.bBlockingHit)
		{
			// If something was hit during movement
			UE_LOG(LogTemp, Warning, TEXT("Movement blocked by %s. Invalidating path."), *HitResult.GetActor()->GetName());
			
			// The current path is no longer valid, so stop movement and request a new path.
			InvalidatePathAndRequestRepath(); 
			
			// Cancel all movement for this frame and exit the function.
			return; 
		}

		// If movement was successful and the target was reached, process the next path point.
		if (bReachedTargetThisStep)
		{
			ProgressNextPoint();
		}
		
	}
}

void UPathfindingComponent::AdjustProgressToSpline(const FVector& FinalVelocity)
{
	if(!IsValid(PathSplinePtr) || PathSplinePtr->GetNumberOfSplinePoints() < 2)
	{
	   	PreferredVelocity = FVector::ZeroVector;
	   	return;
	}

	// "굴러가는" 스플라인을 따라가는 가장 단순하고 확실한 방법은,
	// 언제나 그 스플라인의 맨 앞(거리 0)의 방향을 따라가는 것입니다.
	const FVector SplineDirection = PathSplinePtr->GetDirectionAtDistanceAlongSpline(0.0f, ESplineCoordinateSpace::World);

	PreferredVelocity = SplineDirection * MovementSpeed;
}

void UPathfindingComponent::ProgressNextPoint()
{
	if (GetMovementMode() == EPathMovementMode::Spline)
	{
		PartialSplinePointUpdate(CurrentPathDestinationIndex);
	}

	
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
	UpdatePreferredVelocity(DeltaTime);
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

	if (bSuccess && Path.Num() > 0)
	{
		const FVector CurrentActorLocation = GetOwner()->GetActorLocation();
		const FVector NewPathStartLocation = Path[0];

		// Use the helper function to check if the path from the current location to the new path's start is clear.
		if (IsPathObstructed(CurrentActorLocation, NewPathStartLocation))
		{
			// *** Case: Path is not safe ***
			// The received path is dangerous, so discard it and immediately request a new one.
			UE_LOG(LogTemp, Warning, TEXT("New path is obstructed from actor's current location. Requesting a new path immediately."));
			RequestPathToLocation(CurrentTargetLocation);
			return; // Exit the function
		}

		// *** Case: Path is safe ***
		FoundPath = Path;
		
		// To prevent "rushing" to the start, modify the path's starting point to the actor's current location.
		FoundPath[0] = CurrentActorLocation;
		
		// Reset state variables.
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

void UPathfindingComponent::InvalidatePathAndRequestRepath()
{
	if (CurrentState == EPathfindingState::PathInvalidated) return;

	UE_LOG(LogTemp, Warning, TEXT("Path Invalidated! Stopping movement and scheduling a repath..."));

	// Begin Test

	RequestPathToLocation(CurrentTargetLocation);

	// End Test
}

bool UPathfindingComponent::IsPathObstructed(const FVector& Start, const FVector& End) const
{
	AOctreeVoxelVolume* OctreeVolume = Cast<AOctreeVoxelVolume>(TargetVolume.Get());
	if (!OctreeVolume)
	{
		// If there's no volume, treat the path as obstructed for safety.
		UE_LOG(LogTemp, Error, TEXT("IsPathObstructed: TargetVolume is not a valid AOctreeVoxelVolume."));
		return true; 
	}

	const FVector PathVector = End - Start;
	const float TotalDistance = PathVector.Size();
	
	// If the path is very short, just check the endpoint.
	if (TotalDistance < KINDA_SMALL_NUMBER)
	{
		return OctreeVolume->IsLocationInBlockedVoxel(End);
	}

	const FVector Direction = PathVector.GetSafeNormal();
	// The step size should be smaller than the agent's radius to avoid missing small obstacles.
	const float StepSize = AgentRadius * 0.8f; 

	// Check at intervals of StepSize from the start to the end.
	for (float Distance = 0; Distance < TotalDistance; Distance += StepSize)
	{
		const FVector CurrentPosition = Start + Direction * Distance;
		if (OctreeVolume->IsLocationInBlockedVoxel(CurrentPosition))
		{
			// Obstacle found mid-path!
			return true;
		}
	}

	// After the loop, perform one final check at the exact endpoint of the path.
	if (OctreeVolume->IsLocationInBlockedVoxel(End))
	{
		return true;
	}

	// If all checks pass, the path is clear.
	return false;
}

void UPathfindingComponent::DrawDebugPath()
{
	UWorld* World = GetWorld();
	if (!World || FoundPath.Num() < 2)
	{
		return;
	}


	const float Duration = 1.0f;
	const float LineThickness = 40.0f;
	const float BoxThickness = AgentRadius * 0.7;
	const FVector BoxHalfSize(AgentRadius *0.05);

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
	AOctreeVoxelVolume* VoxelVolume = Cast<AOctreeVoxelVolume>(TargetVolume.Get());
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

	for (float Distance = 0.f; Distance <= SplineLength; Distance += SafetyCheckStepSize)
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

	return true;
}
