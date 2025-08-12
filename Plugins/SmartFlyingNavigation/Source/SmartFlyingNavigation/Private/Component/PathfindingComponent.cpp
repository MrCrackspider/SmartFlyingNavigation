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
}

void UPathfindingComponent::TickComponent(float DeltaTime, ELevelTick TickType,
                                          FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	
	if (GetMovementMode() == EPathMovementMode::Spline)
	{
		if (!IsSplinePathSafe())
		{
			SetMovementMode(EPathMovementMode::Linear);
		}
	}
	
	if (!IsValid(AvoidanceSubsystem) || !AvoidanceSubsystem->GetbUseAvoidance())
	{
		UpdateSteering(DeltaTime);
		ApplyMovement(PreferredVelocity, DeltaTime);
	}
	CurrentVelocity = PreferredVelocity;
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
	LastKnownTargetLocation = TargetLocation;
	
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
		FRotator InterpolatedRotation = FMath::RInterpTo(CurrentRotation, TargetRotation, DeltaTime, RotationInterpSpeed);
    
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
	
	while (!bIsReachDestination && FoundPath.IsValidIndex(CurrentPathDestinationIndex))
	{
		const FVector& TargetWaypoint = FoundPath[CurrentPathDestinationIndex];

		const bool bIsAtWaypoint = FVector::DistSquared(Owner->GetActorLocation(), TargetWaypoint) <= FMath::Square(AcceptanceRadius);

		bool bHasPassedWaypoint = false;
		if (CurrentPathDestinationIndex > 0)
		{
			const FVector& PrevWaypoint = FoundPath[CurrentPathDestinationIndex - 1];
			const FVector PathSegmentDirection = (TargetWaypoint - PrevWaypoint).GetSafeNormal();
			if (!PathSegmentDirection.IsNearlyZero())
			{
				const FVector VectorToAgent = Owner->GetActorLocation() - TargetWaypoint;
				bHasPassedWaypoint = FVector::DotProduct(VectorToAgent, PathSegmentDirection) >= 0.0f;
			}
		}

		if (bIsAtWaypoint || bHasPassedWaypoint)
		{
			ProgressNextPoint();
		}
		else
		{
			break;
		}
	}
	
}

void UPathfindingComponent::AdjustProgressToSpline(const FVector& MoveDelta)
{
	if (!IsValid(PathSplinePtr) || PathSplinePtr->GetNumberOfSplinePoints() < 2) return;

	const FVector CurrLoc = GetOwner()->GetActorLocation();
	const float InputKey = PathSplinePtr->FindInputKeyClosestToWorldLocation(CurrLoc);
	const float Len = PathSplinePtr->GetSplineLength();
	float S = PathSplinePtr->GetDistanceAlongSplineAtSplineInputKey(InputKey);
	DistanceAlongSpline = FMath::Clamp(S, 0.f, Len);
}

void UPathfindingComponent::ProgressNextPoint()
{
	PartialSplinePointUpdate(CurrentPathDestinationIndex);
	
	if (CurrentPathDestinationIndex + 1 >= FoundPath.Num())
	{
		const float DistanceToFinalGoalSq = FVector::DistSquared(GetOwner()->GetActorLocation(), CurrentTargetLocation);

		if (DistanceToFinalGoalSq <= FMath::Square(AcceptanceRadius))
		{
			OnDestinationReached.Broadcast();
			StopMovement();
			return;
		}
		else
		{
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
void UPathfindingComponent::HandleWorldPathResult(const TArray<FVector>& Path, bool bSuccess)
{
    if (!IsValid(PathSplinePtr)) return;

    if (!bSuccess || Path.IsEmpty())
    {
        if (bPeriodicRepath)
            GetWorld()->GetTimerManager().SetTimer(RepathCooldownTimer, this, &UPathfindingComponent::PeriodicRepath, RepathCooldown, false);
        return;
    }

    const FVector CurrentLocation = GetOwner()->GetActorLocation();

    int32 ClosestSegIdx = -1;
    float ClosestDistSq = TNumericLimits<float>::Max();
    FVector ClosestPoint = Path[0];
    for (int32 i = 1; i < Path.Num(); ++i)
    {
        const FVector A = Path[i-1];
        const FVector B = Path[i];
        const FVector AB = B - A;
        const float Den = FVector::DotProduct(AB, AB);
        float t = (Den > KINDA_SMALL_NUMBER) ? FVector::DotProduct(CurrentLocation - A, AB) / Den : 0.f;
        t = FMath::Clamp(t, 0.f, 1.f);
        const FVector P = A + AB * t;
        const float d2 = FVector::DistSquared(CurrentLocation, P);
        if (d2 < ClosestDistSq)
        {
            ClosestDistSq = d2;
            ClosestSegIdx = i;
            ClosestPoint = P;
        }
    }

	FVector ForwardDir = CurrentVelocity.IsNearlyZero() ? FVector::ZeroVector : CurrentVelocity.GetSafeNormal();
    if (ForwardDir.IsNearlyZero() && ClosestSegIdx >= 1)
    {
        const FVector A = Path[ClosestSegIdx-1];
        const FVector B = Path[ClosestSegIdx];
        ForwardDir = (B - A).GetSafeNormal();
    }
    if (ForwardDir.IsNearlyZero())
    {
        ForwardDir = (Path.Last() - CurrentLocation).GetSafeNormal();
    }

	int32 JoinIdx = -1;
    const int32 StartIdx = FMath::Max(1, ClosestSegIdx);
    for (int32 j = StartIdx; j < Path.Num(); ++j)
    {
        if (FVector::DotProduct(Path[j] - CurrentLocation, ForwardDir) <= 0.f) continue;
        if (!IsPathObstructed(CurrentLocation, Path[j]))
        {
            JoinIdx = j;
            break;
        }
    }

    TArray<FVector> NewPath;
    NewPath.Add(CurrentLocation);

    if (JoinIdx >= 0)
    {
        for (int32 k = JoinIdx; k < Path.Num(); ++k) NewPath.Add(Path[k]);
    }
    else
    {
        const float Step = FMath::Max(AgentRadius * 0.5f, 10.f);
        FVector DirSeg = ForwardDir;
        if (DirSeg.IsNearlyZero() && ClosestSegIdx >= 1)
            DirSeg = (Path[ClosestSegIdx] - Path[ClosestSegIdx-1]).GetSafeNormal();
        if (DirSeg.IsNearlyZero()) DirSeg = (Path.Last() - Path[0]).GetSafeNormal();

        FVector E = ClosestPoint;
        const float SegLen = (ClosestSegIdx >= 1) ? (Path[ClosestSegIdx] - Path[ClosestSegIdx-1]).Size() : 0.f;
        const int32 MaxIt = FMath::Max(1, (int32)FMath::CeilToInt((SegLen + 1.f) / Step));
        bool bFoundE = false;
        for (int32 it = 1; it <= MaxIt; ++it)
        {
            const FVector Cand = ClosestPoint + DirSeg * (it * Step);
            if (!IsPathObstructed(CurrentLocation, Cand))
            {
                E = Cand; bFoundE = true; break;
            }
        }

        if (bFoundE)
        {
            NewPath.Add(E);
            const int32 AfterIdx = FMath::Clamp(ClosestSegIdx, 1, Path.Num()-1);
            for (int32 k = AfterIdx; k < Path.Num(); ++k) NewPath.Add(Path[k]);
        }
        else
        {
            const int32 FallbackIdx = FMath::Clamp(ClosestSegIdx, 1, Path.Num()-1);
            for (int32 k = FallbackIdx; k < Path.Num(); ++k) NewPath.Add(Path[k]);
        }
    }

   FoundPath = MoveTemp(NewPath);

    bIsReachDestination = false;
    CurrentState = EPathfindingState::FollowPath;
    CurrentPathDestinationIndex = (FoundPath.Num() >= 2) ? 1 : 0;

    PathSplinePtr->ClearSplinePoints();
    for (int i = 0; i < PartialPointSplineNum && i < FoundPath.Num(); ++i)
        PathSplinePtr->AddSplinePoint(FoundPath[i], ESplineCoordinateSpace::World, false);
    PathSplinePtr->UpdateSpline();

	if (AActor* Owner = GetOwner())
	{
		PathSplinePtr->SetLocationAtSplinePoint(0, Owner->GetActorLocation(), ESplineCoordinateSpace::World);
		const float Key0 = PathSplinePtr->FindInputKeyClosestToWorldLocation(Owner->GetActorLocation());
		const float Len  = PathSplinePtr->GetSplineLength();
		float S = PathSplinePtr->GetDistanceAlongSplineAtSplineInputKey(Key0);
		DistanceAlongSpline = FMath::Clamp(S, 0.f, Len);
	}

	const bool bSplineSafe = IsSplinePathSafe();
	SetMovementMode(bSplineSafe ? EPathMovementMode::Spline : EPathMovementMode::Linear);

    DetermineMovementModeForNextPath();

#if WITH_EDITOR
	if (bDrawDebugPath) DrawDebugPath();
#endif
    if (bPeriodicRepath)
        GetWorld()->GetTimerManager().SetTimer(RepathCooldownTimer, this, &UPathfindingComponent::PeriodicRepath, RepathCooldown, false);
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

	if (UWorld* World = GetWorld())
	{
		World->GetTimerManager().ClearTimer(RepathCooldownTimer);
	}

	RequestPathToLocation(CurrentTargetLocation);
	// End Test
}

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

#if WITH_EDITOR
void UPathfindingComponent::DrawDebugPath()
{
	UWorld* World = GetWorld();
	if (!World || FoundPath.Num() < 2)
	{
		return;
	}


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
#endif

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
	{ PreferredVelocity = FVector::ZeroVector; return; }

	if (!IsSplinePathSafe())
	{
		SetMovementMode(EPathMovementMode::Linear);
		UpdateLinearPreferredVelocity(DeltaTime);
		return;
	}

	const float Len = PathSplinePtr->GetSplineLength();
	const float Key = PathSplinePtr->FindInputKeyClosestToWorldLocation(Owner->GetActorLocation());
	float S = PathSplinePtr->GetDistanceAlongSplineAtSplineInputKey(Key);
	S = FMath::Clamp(S, 0.f, Len);
	const float Remain = FMath::Max(0.f, Len - S);

	const float LookBase = FMath::Max(AgentRadius, 50.f);
	const float LookAhead = FMath::Clamp(S + FMath::Min(LookBase, Remain), 0.f, FMath::Max(0.f, Len - KINDA_SMALL_NUMBER));
	const FVector Dir = PathSplinePtr->GetDirectionAtDistanceAlongSpline(LookAhead, ESplineCoordinateSpace::World);

	const float Voxel = Cast<AVoxelVolumeBase>(GetCurrentVolume())->VoxelExtent;
	const float BrakeDist = FMath::Max(2.f * AgentRadius, 2.f * Voxel);

	const float SpeedScale = FMath::Clamp(Remain / BrakeDist, 0.f, 1.f);
	PreferredVelocity = Dir * (MovementSpeed * SpeedScale);
}

bool UPathfindingComponent::IsSplinePathSafe() const
{
	AOctreeVoxelVolume* V = Cast<AOctreeVoxelVolume>(GetCurrentVolume());
	if (!IsValid(PathSplinePtr) || PathSplinePtr->GetNumberOfSplinePoints() < 2) return false;
	if (!IsValid(V) || V->GetRootNodeIndex() == -1) return false;

	const float Len = PathSplinePtr->GetSplineLength();
	if (Len <= KINDA_SMALL_NUMBER) return false;

	const FVector Curr = GetOwner()->GetActorLocation();
	const float Key = PathSplinePtr->FindInputKeyClosestToWorldLocation(Curr);
	float S = PathSplinePtr->GetDistanceAlongSplineAtSplineInputKey(Key);
	S = FMath::Clamp(S, 0.f, Len);

	const float Voxel = Cast<AVoxelVolumeBase>(V)->VoxelExtent;
	const float Step  = FMath::Max(10.f, FMath::Min(AgentRadius * 0.5f, Voxel * 0.5f));
	const float Look  = FMath::Max(AgentRadius * 4.f, Voxel * 2.f);
	const float E     = FMath::Min(S + Look, Len);

	constexpr int32 RingSamples = 8;
	for (float d = S; d <= E; d += Step)
	{
		const FVector C = PathSplinePtr->GetLocationAtDistanceAlongSpline(d, ESplineCoordinateSpace::World);
		const FVector T = PathSplinePtr->GetDirectionAtDistanceAlongSpline(d, ESplineCoordinateSpace::World).GetSafeNormal();

		const FVector Up = FVector::UpVector;
		FVector N = T.Cross(Up); if (N.IsNearlyZero()) N = T.Cross(FVector::RightVector);
		N = N.GetSafeNormal();
		const FVector B = (T ^ N).GetSafeNormal();

		if (V->IsLocationInBlockedVoxel(C)) return false;

		const float R = AgentRadius;
		for (int i = 0; i < RingSamples; ++i)
		{
			const float ang = (2.f * PI * i) / RingSamples;
			const FVector Off = (N * FMath::Cos(ang) + B * FMath::Sin(ang)) * R;
			if (V->IsLocationInBlockedVoxel(C + Off)) return false;
		}
	}
	return true;
}
