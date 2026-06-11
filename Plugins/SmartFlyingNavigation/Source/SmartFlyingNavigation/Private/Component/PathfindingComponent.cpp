// Copyright © 2025 TeamHamDev. All Rights Reserved.

#include "Component/PathfindingComponent.h"
#include "Component/FlyingNavMovementComponent.h"
#include "Components/SplineComponent.h"
#include "Subsystem/PathfindingSubsystem.h"
#include "Subsystem/AvoidanceSubsystem.h"
#include "Subsystem/OctreeNavSubsystem.h"
#include "Strategy/PathfindingStrategy.h"
#include "GameFramework/Actor.h"
#include "TimerManager.h"
#include "Engine/World.h"
#include "DrawDebugHelpers.h"
#include "Components/SphereComponent.h"
#include "NavVolume/OctreeVoxelVolume.h"
#include "Engine/Engine.h"
#include "Strategy/AStarStrategy.h"
#include "Strategy/AirwayGraphStrategy.h"
#include "Utility/Parallel/PathfindingRequest.h"
#include "HAL/IConsoleManager.h"

/**
 * Runtime master switch for path debug drawing (lines/boxes are expensive on mobile).
 * -1 = follow each component's DebugDraw.bDrawDebugPath, 0 = force off, 1 = force on.
 * Toggle on Android via the 4-finger-tap console or:
 *   adb shell am broadcast -a android.intent.action.RUN -e cmd "sfn.DrawPathDebug 1"
 */
static TAutoConsoleVariable<int32> CVarDrawPathDebug(
	TEXT("sfn.DrawPathDebug"),
	-1,
	TEXT("-1=per-component setting, 0=force off, 1=force on."),
	ECVF_Default);

UPathfindingComponent::UPathfindingComponent()
{
	PrimaryComponentTick.bCanEverTick = true;

#if WITH_EDITOR
	bTickInEditor = true;
#endif

#if PLATFORM_ANDROID
	PathfindingStrategyClass = UAirwayGraphStrategy::StaticClass();
	MovementMode = EPathMovementMode::Linear;
#else
	PathfindingStrategyClass = UAStarStrategy::StaticClass();
#endif

	PathSplinePtr = CreateDefaultSubobject<USplineComponent>(TEXT("SplineComponent"));
	PathSplinePtr->SetAbsolute(true, true, true);

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

	if (!ensure(GetPathfindingSubsystem()))
	{
		return;
	}
	bPeriodicRepath = true;

	if (UAvoidanceSubsystem* Avoidance = GetAvoidanceSubsystem())
	{
		Avoidance->RegisterAgent(this);
	}

	if (PathfindingStrategyClass)
	{
		PathfindingStrategy = NewObject<UPathfindingStrategy>(this, PathfindingStrategyClass);

		if (PathfindingStrategy)
		{
			PathfindingStrategy->Initialize();
		}
	}

#if PLATFORM_ANDROID
	if (PathfindingStrategy && PathfindingStrategy->IsA<UAirwayGraphStrategy>())
	{
		FallbackPathfindingStrategy = NewObject<UAStarStrategy>(this, UAStarStrategy::StaticClass());
		if (FallbackPathfindingStrategy)
		{
			FallbackPathfindingStrategy->Initialize();
		}
	}
#endif

	if (!MovementExecutor.GetObject() && GetOwner())
	{
		if (GetOwner()->GetClass()->ImplementsInterface(UFlyingMovementExecutor::StaticClass()))
		{
			MovementExecutor.SetObject(GetOwner());
			MovementExecutor.SetInterface(Cast<IFlyingMovementExecutor>(GetOwner()));
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

	if (GetWorld() && GetWorld()->IsGameWorld())
	{
		const UAvoidanceSubsystem* Avoidance = GetAvoidanceSubsystem();
		if (!Avoidance || !Avoidance->GetbUseAvoidance())
		{
			UpdateSteering(DeltaTime);
			ApplyMovement(PreferredVelocity, DeltaTime);
			CurrentVelocity = PreferredVelocity;
		}

		if (FoundPath.IsEmpty() && !bIsReachDestination && !bIsPathRequestInFlight && !bHasPendingPathRequest
			&& IsValid(TargetActor) && !GetWorld()->GetTimerManager().IsTimerActive(RepathCooldownTimer))
		{
			GetWorld()->GetTimerManager().SetTimer(
				RepathCooldownTimer, this, &UPathfindingComponent::PeriodicRepath, RepathCooldown, false);
		}
	}

	UpdateAgentRadiusFromScale();

#if ENABLE_DRAW_DEBUG
	// Per-tick, single-frame draw: continuous path visualization (works on-device in non-Shipping builds).
	{
		const int32 DrawMode = CVarDrawPathDebug.GetValueOnGameThread();
		const bool bShouldDraw = (DrawMode == 1) || (DrawMode != 0 && DebugDraw.bDrawDebugPath);
		if (bShouldDraw)
		{
			DrawDebugPath();
		}
	}
#endif
}

void UPathfindingComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (UAvoidanceSubsystem* Avoidance = GetAvoidanceSubsystem())
	{
		Avoidance->UnregisterAgent(this);
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

	UOctreeNavSubsystem* NavSubsystem = GetWorld()->GetSubsystem<UOctreeNavSubsystem>();
	if (!NavSubsystem || !GetPathfindingSubsystem()) return false;

	const FVector StartLocation = GetOwner()->GetActorLocation();
	AOctreeVoxelVolume* StartVolume = NavSubsystem->GetVolumeChunkAt(StartLocation);
	LastKnownTargetLocation = TargetLocation;

	if (!StartVolume || !StartVolume->IsReadyForNavigation())
	{
		UE_LOG(LogTemp, Log, TEXT("Start volume is not ready. Path request is pending."));
		bHasPendingPathRequest = true;
		PendingTargetLocation = TargetLocation;
		NavSubsystem->OnVolumeRegistered.RemoveDynamic(this, &UPathfindingComponent::OnVolumeBecameReadyForPendingRequest);
		NavSubsystem->OnVolumeRegistered.AddDynamic(this, &UPathfindingComponent::OnVolumeBecameReadyForPendingRequest);
		return true;
	}

	bHasPendingPathRequest = false;
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

	ActiveVolume = Volume;

	FPathfindingRequest Request;
	Request.RequestingObject = this;
	Request.StartLocation = Start;
	Request.EndLocation = End;
	Request.AgentRadius = this->AgentRadius;
	Request.Strategy = PathfindingStrategy;
	Request.NodeType = EPathfindingNodeType::Octree;
	Request.OnWorldPathComplete.BindDynamic(this, &UPathfindingComponent::HandleWorldPathResult);

	CreateOctreeDataSnapshot(Volume, Request.NavigationData);

#if PLATFORM_ANDROID
	if (!Request.NavigationData.AirwayGraphPtr && PathfindingStrategy && PathfindingStrategy->IsA<UAirwayGraphStrategy>() && FallbackPathfindingStrategy)
	{
		UE_LOG(LogTemp, Warning, TEXT("SmartFlyingNavigation: AirwayGraph data is missing. Falling back to Octree A* on Android. Re-bake the navigation data to use Mobile Lite graph navigation."));
		Request.Strategy = FallbackPathfindingStrategy;
	}
#endif

	CurrentTargetLocation = End;
	UPathfindingSubsystem* Pathfinding = GetPathfindingSubsystem();
	CurrentRequestID = Pathfinding ? Pathfinding->RequestPath(this, Request) : -1;

	if (CurrentRequestID != -1)
	{
		bIsPathRequestInFlight = true;
	}

	return CurrentRequestID != -1;
}

UPathfindingSubsystem* UPathfindingComponent::GetPathfindingSubsystem() const
{
	UWorld* World = GetWorld();
	return World ? World->GetSubsystem<UPathfindingSubsystem>() : nullptr;
}

UAvoidanceSubsystem* UPathfindingComponent::GetAvoidanceSubsystem() const
{
	UWorld* World = GetWorld();
	return World ? World->GetSubsystem<UAvoidanceSubsystem>() : nullptr;
}

void UPathfindingComponent::CreateOctreeDataSnapshot(const AOctreeVoxelVolume* OctreeVolume, FOctreePathfindingData& SnapshotData) const
{
	// The data view already carries an owned snapshot of dynamic obstacle primitives,
	// so it is safe to hand to async pathfinding tasks as-is.
	OctreeVolume->CreatePathfindingDataView(SnapshotData);
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
	CurrentPathCorridor.Reset();
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
	if (!Owner || bIsReachDestination || FinalVelocity.IsNearlyZero())
	{
		return;
	}

	constexpr float MaxDeltaTime = 0.05f;
	DeltaTime = FMath::Min(DeltaTime, MaxDeltaTime);

	const FVector MoveDelta = FinalVelocity * DeltaTime;
	const FVector FacingDirection = MoveDelta.IsNearlyZero() ? FVector::ZeroVector : MoveDelta.GetSafeNormal();

	if (MovementExecutor.GetObject())
	{
		IFlyingMovementExecutor::Execute_ExecuteMovement(
			MovementExecutor.GetObject(), FinalVelocity, DeltaTime, FacingDirection);
	}
	else if (OnMovementRequested.IsBound())
	{
		OnMovementRequested.Broadcast(FinalVelocity, DeltaTime, FacingDirection);
	}
	else if (UFlyingNavMovementComponent* NavMovement = ResolveNavMovementComponent(); NavMovement && NavMovement->IsAbleToMove())
	{
		// Preferred route: the movement component owns the actual motion (sweep + slide)
		// and reports Velocity so physics/animation see real movement.
		NavMovement->SetDesiredVelocity(FinalVelocity);

		if (NavMovement->GetBlockedFrameCount() > 10)
		{
			NavMovement->ResetBlockedFrameCount();
			InvalidatePathAndRequestRepath();
		}
	}
	else
	{
		if (NavMovementComponent.IsValid() && !bWarnedDefaultMovement)
		{
			bWarnedDefaultMovement = true;
			UE_LOG(LogTemp, Warning,
				TEXT("PathfindingComponent on '%s': a FlyingNavMovementComponent exists but cannot move ")
				TEXT("(owner must be a Pawn with a movable root component). Falling back to default movement."),
				*GetNameSafe(GetOwner()));
		}
		// Deprecated kinematic fallback: physics never observes a velocity.
		if (!bWarnedDefaultMovement)
		{
			bWarnedDefaultMovement = true;
			UE_LOG(LogTemp, Warning,
				TEXT("PathfindingComponent on '%s' is using the deprecated AddActorWorldOffset movement fallback. ")
				TEXT("Add a UFlyingNavMovementComponent to the pawn (or implement IFlyingMovementExecutor) for physics-consistent movement."),
				*GetNameSafe(GetOwner()));
		}
		ExecuteDefaultMovement(FinalVelocity, DeltaTime);
	}

	AdjustProgressToSpline(MoveDelta);
	HandleWaypointProgression();
}

UFlyingNavMovementComponent* UPathfindingComponent::ResolveNavMovementComponent()
{
	if (NavMovementComponent.IsValid())
	{
		return NavMovementComponent.Get();
	}

	if (AActor* Owner = GetOwner())
	{
		NavMovementComponent = Owner->FindComponentByClass<UFlyingNavMovementComponent>();
	}

	return NavMovementComponent.Get();
}

void UPathfindingComponent::ExecuteDefaultMovement(const FVector& Velocity, float DeltaTime)
{
	AActor* Owner = GetOwner();
	if (!Owner) return;

	const FVector MoveDelta = Velocity * DeltaTime;
	FHitResult HitResult;
	Owner->AddActorWorldOffset(MoveDelta, true, &HitResult);

	if (!MoveDelta.IsNearlyZero())
	{
		const FRotator CurrentRotation = Owner->GetActorRotation();
		const FRotator TargetRotation = MoveDelta.ToOrientationRotator();
		FRotator InterpolatedRotation = FMath::RInterpTo(CurrentRotation, TargetRotation, DeltaTime, RotationInterpSpeed);
		Owner->SetActorRotation(InterpolatedRotation);
	}

	if (HitResult.bBlockingHit)
	{
		ConsecutiveBlockedFrames++;
		if (ConsecutiveBlockedFrames > 10)
		{
			ConsecutiveBlockedFrames = 0;
			InvalidatePathAndRequestRepath();
		}
	}
	else
	{
		ConsecutiveBlockedFrames = 0;
	}
}

void UPathfindingComponent::HandleWaypointProgression()
{
	AActor* Owner = GetOwner();
	if (!Owner) return;

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
	const float S = PathSplinePtr->GetDistanceAlongSplineAtSplineInputKey(InputKey);
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
	if (!MyActor || !IsValid(PathSplinePtr))
	{
		return false;
	}

	const FVector MyLocation = MyActor->GetActorLocation();
	const FVector SplineLocation = PathSplinePtr->GetLocationAtDistanceAlongSpline(
		DistanceAlongSpline, ESplineCoordinateSpace::World);

	return MyLocation.Equals(SplineLocation, AgentRadius * 2.f);
}

void UPathfindingComponent::DetermineMovementModeForNextPath()
{
	SetMovementMode(IsSplinePathSafe() ? EPathMovementMode::Spline : EPathMovementMode::Linear);
}

void UPathfindingComponent::PeriodicRepath()
{
	if (IsValid(TargetActor))
	{
		CurrentTargetLocation = TargetActor->GetActorLocation();
	}

	if (!CurrentTargetLocation.IsZero())
	{
		RequestPathToLocation(CurrentTargetLocation);
	}
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
	bIsPathRequestInFlight = false;

	if (!IsValid(PathSplinePtr)) return;

	if (!bSuccess || Path.IsEmpty())
	{
		CurrentPathCorridor.Reset();
		if (bPeriodicRepath)
		{
			GetWorld()->GetTimerManager().SetTimer(
				RepathCooldownTimer, this, &UPathfindingComponent::PeriodicRepath, RepathCooldown, false);
		}
		return;
	}

	const FVector CurrentLocation = GetOwner()->GetActorLocation();

	// 현재 위치에서 가장 가까운 경로 세그먼트 탐색
	int32 ClosestSegIdx = -1;
	float ClosestDistSq = TNumericLimits<float>::Max();
	FVector ClosestPoint = Path[0];
	for (int32 i = 1; i < Path.Num(); ++i)
	{
		const FVector A = Path[i - 1];
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

	// 전방 방향 결정
	FVector ForwardDir = CurrentVelocity.IsNearlyZero() ? FVector::ZeroVector : CurrentVelocity.GetSafeNormal();
	if (ForwardDir.IsNearlyZero() && ClosestSegIdx >= 1)
	{
		ForwardDir = (Path[ClosestSegIdx] - Path[ClosestSegIdx - 1]).GetSafeNormal();
	}
	if (ForwardDir.IsNearlyZero())
	{
		ForwardDir = (Path.Last() - CurrentLocation).GetSafeNormal();
	}

	// 전방에 있는 경로 포인트 중 장애물 없이 도달 가능한 지점 탐색
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
		for (int32 k = JoinIdx; k < Path.Num(); ++k)
		{
			NewPath.Add(Path[k]);
		}
	}
	else
	{
		const float Step = FMath::Max(AgentRadius * 0.5f, 10.f);
		FVector DirSeg = ForwardDir;
		if (DirSeg.IsNearlyZero() && ClosestSegIdx >= 1)
		{
			DirSeg = (Path[ClosestSegIdx] - Path[ClosestSegIdx - 1]).GetSafeNormal();
		}
		if (DirSeg.IsNearlyZero())
		{
			DirSeg = (Path.Last() - Path[0]).GetSafeNormal();
		}

		const float SegLen = (ClosestSegIdx >= 1) ? (Path[ClosestSegIdx] - Path[ClosestSegIdx - 1]).Size() : 0.f;
		const int32 MaxIt = FMath::Max(1, FMath::CeilToInt((SegLen + 1.f) / Step));
		FVector E = ClosestPoint;
		bool bFoundE = false;
		for (int32 it = 1; it <= MaxIt; ++it)
		{
			const FVector Cand = ClosestPoint + DirSeg * (it * Step);
			if (!IsPathObstructed(CurrentLocation, Cand))
			{
				E = Cand;
				bFoundE = true;
				break;
			}
		}

		if (bFoundE)
		{
			NewPath.Add(E);
			const int32 AfterIdx = FMath::Clamp(ClosestSegIdx, 1, Path.Num() - 1);
			for (int32 k = AfterIdx; k < Path.Num(); ++k)
			{
				NewPath.Add(Path[k]);
			}
		}
		else
		{
			// 모든 경로 포인트를 순회해서 도달 가능한 지점 탐색
			int32 ReachableIdx = -1;
			for (int32 k = 0; k < Path.Num(); ++k)
			{
				if (!IsPathObstructed(CurrentLocation, Path[k]))
				{
					ReachableIdx = k;
					break;
				}
			}

			if (ReachableIdx >= 0)
			{
				for (int32 k = ReachableIdx; k < Path.Num(); ++k)
				{
					NewPath.Add(Path[k]);
				}
			}
			else
			{
				// 어떤 포인트에도 도달 불가 — sweep 충돌 + repath에 의존
				const int32 FallbackIdx = FMath::Clamp(ClosestSegIdx, 1, Path.Num() - 1);
				for (int32 k = FallbackIdx; k < Path.Num(); ++k)
				{
					NewPath.Add(Path[k]);
				}
			}
		}
	}

	FoundPath = MoveTemp(NewPath);
	RebuildCurrentCorridorFromFoundPath(true);

	bIsReachDestination = false;
	CurrentState = EPathfindingState::FollowPath;
	CurrentPathDestinationIndex = (FoundPath.Num() >= 2) ? 1 : 0;

	PathSplinePtr->ClearSplinePoints();
	for (int32 i = 0; i < PartialPointSplineNum && i < FoundPath.Num(); ++i)
	{
		PathSplinePtr->AddSplinePoint(FoundPath[i], ESplineCoordinateSpace::World, false);
	}
	PathSplinePtr->UpdateSpline();

	if (AActor* Owner = GetOwner())
	{
		PathSplinePtr->SetLocationAtSplinePoint(0, Owner->GetActorLocation(), ESplineCoordinateSpace::World);
		const float Key0 = PathSplinePtr->FindInputKeyClosestToWorldLocation(Owner->GetActorLocation());
		const float Len = PathSplinePtr->GetSplineLength();
		const float S = PathSplinePtr->GetDistanceAlongSplineAtSplineInputKey(Key0);
		DistanceAlongSpline = FMath::Clamp(S, 0.f, Len);
	}

	DetermineMovementModeForNextPath();

	if (bPeriodicRepath)
	{
		GetWorld()->GetTimerManager().SetTimer(
			RepathCooldownTimer, this, &UPathfindingComponent::PeriodicRepath, RepathCooldown, false);
	}
}

void UPathfindingComponent::UpdateAgentRadiusFromScale(bool bForceUpdate)
{
	AActor* Owner = GetOwner();
	if (!Owner) return;

	const FVector CurrentScale = Owner->GetActorScale3D();
	const float PreviousAgentRadius = AgentRadius;

	if (!bForceUpdate && CurrentScale == LastKnownScale && BaseAgentRadius == LastKnownBaseRadius)
		return;

	LastKnownScale = CurrentScale;
	LastKnownBaseRadius = BaseAgentRadius;

	const float MaxScale = CurrentScale.GetMax();
	AgentRadius = BaseAgentRadius * MaxScale;

	if (AvoidanceRadius < AgentRadius)
	{
		AvoidanceRadius = AgentRadius * 1.1f;
	}

	if (DebugSphereComponent)
	{
		DebugSphereComponent->SetSphereRadius(AgentRadius);
	}

	if (!FMath::IsNearlyEqual(PreviousAgentRadius, AgentRadius, KINDA_SMALL_NUMBER) &&
		GetWorld() && GetWorld()->IsGameWorld() && !bIsReachDestination &&
		(FoundPath.Num() > 0 || !CurrentTargetLocation.IsZero() || IsValid(TargetActor)))
	{
		CurrentState = EPathfindingState::PathInvalidated;
		CurrentPathCorridor.bDynamicValidated = false;
		InvalidatePathAndRequestRepath();
	}
}

void UPathfindingComponent::RebuildCurrentCorridorFromFoundPath(bool bDynamicValidated)
{
	CurrentPathCorridor.Reset();
	CurrentPathCorridor.Waypoints = FoundPath;
	CurrentPathCorridor.RequiredAgentRadius = AgentRadius;
	CurrentPathCorridor.bDynamicValidated = bDynamicValidated && FoundPath.Num() >= 2;
}

void UPathfindingComponent::InvalidatePathAndRequestRepath()
{
	if (bIsPathRequestInFlight) return;

	if (UWorld* World = GetWorld())
	{
		if (World->GetTimerManager().IsTimerActive(RepathCooldownTimer)) return;

		const float Cooldown = FMath::Max(RepathCooldown, 0.2f);
		World->GetTimerManager().SetTimer(
			RepathCooldownTimer, this, &UPathfindingComponent::PeriodicRepath, Cooldown, false);
	}
}

bool UPathfindingComponent::IsPathObstructed(const FVector& Start, const FVector& End, bool bIsPortalApproach) const
{
	(void)bIsPortalApproach;

	AOctreeVoxelVolume* OctreeVolume = Cast<AOctreeVoxelVolume>(GetCurrentVolume());
	if (!OctreeVolume)
	{
		return true;
	}

	if (AgentRadius <= 0.f)
	{
		return false;
	}

	return OctreeVolume->IsSegmentBlockedForAgent(Start, End, AgentRadius, true, true);
}

#if ENABLE_DRAW_DEBUG
void UPathfindingComponent::DrawDebugPath()
{
	UWorld* World = GetWorld();
	if (!World || FoundPath.Num() < 2)
	{
		return;
	}

	// Single-frame draws: refreshed every tick from TickComponent.
	for (int32 i = 0; i < FoundPath.Num() - 1; ++i)
	{
		DrawDebugLine(
			World,
			FoundPath[i],
			FoundPath[i + 1],
			DebugDraw.PathColor.ToFColor(true),
			false,
			-1.f,
			0,
			DebugDraw.LineThickness
		);
	}

	for (int32 i = 0; i < FoundPath.Num(); ++i)
	{
		DrawDebugBox(
			World,
			FoundPath[i],
			DebugDraw.BoxHalfSize,
			DebugDraw.PathPointColor.ToFColor(true),
			false,
			-1.f,
			0,
			DebugDraw.BoxThickness
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

	if (bUseWallRepulsion)
	{
		ApplyWallRepulsion();
	}
}

void UPathfindingComponent::ApplyWallRepulsion()
{
	if (PreferredVelocity.IsNearlyZero() || AgentRadius <= 0.f)
	{
		return;
	}

	const AActor* Owner = GetOwner();
	const AOctreeVoxelVolume* Volume = Cast<AOctreeVoxelVolume>(GetCurrentVolume());
	const FNavDistanceField* DistanceField = Volume ? Volume->GetDistanceField() : nullptr;
	if (!Owner || !DistanceField)
	{
		return;
	}

	const FVector Location = Owner->GetActorLocation();
	const float Clearance = DistanceField->SampleClearance(Location);
	const float ActivationRange = AgentRadius * 1.5f;
	if (Clearance >= ActivationRange)
	{
		return;
	}

	const FVector AwayFromWall = DistanceField->SampleGradient(Location);
	if (AwayFromWall.IsNearlyZero())
	{
		return;
	}

	// Blend toward the outward gradient, stronger the closer we are to the wall.
	const float Strength = FMath::Clamp(1.f - Clearance / ActivationRange, 0.f, 1.f);
	const float Speed = PreferredVelocity.Size();
	const FVector NewDirection = (PreferredVelocity.GetSafeNormal() + AwayFromWall * Strength).GetSafeNormal();
	if (!NewDirection.IsNearlyZero())
	{
		PreferredVelocity = NewDirection * Speed;
	}
}

void UPathfindingComponent::UpdateLinearPreferredVelocity(float DeltaTime)
{
	AActor* Actor = GetOwner();
	if (!Actor) return;

	if (CurrentPathDestinationIndex >= 0 && CurrentPathDestinationIndex < FoundPath.Num())
	{
		const FVector CurrentLocation = Actor->GetActorLocation();
		const FVector TargetPoint = FoundPath[CurrentPathDestinationIndex];
		const FVector Direction = (TargetPoint - CurrentLocation).GetSafeNormal();
		PreferredVelocity = Direction * MovementSpeed;
		return;
	}

	if (!CurrentTargetLocation.IsZero() && !bIsReachDestination)
	{
		const FVector CurrentLocation = Actor->GetActorLocation();
		const FVector Direction = (CurrentTargetLocation - CurrentLocation).GetSafeNormal();
		if (!Direction.IsNearlyZero() && !IsPathObstructed(CurrentLocation, CurrentTargetLocation))
		{
			PreferredVelocity = Direction * MovementSpeed * 0.5f;
			return;
		}
	}

	PreferredVelocity = FVector::ZeroVector;
}

void UPathfindingComponent::UpdateSplinePreferredVelocity(float DeltaTime)
{
	AActor* Owner = GetOwner();
	if (!Owner || !IsValid(PathSplinePtr) || PathSplinePtr->GetNumberOfSplinePoints() < 2)
	{
		PreferredVelocity = FVector::ZeroVector;
		return;
	}

	if (!IsSplinePathSafe())
	{
		SetMovementMode(EPathMovementMode::Linear);
		UpdateLinearPreferredVelocity(DeltaTime);
		return;
	}

	AVoxelVolumeBase* Volume = GetCurrentVolume();
	if (!Volume)
	{
		PreferredVelocity = FVector::ZeroVector;
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

	const float Voxel = Volume->VoxelExtent;
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

	const float Voxel = V->VoxelExtent;
	const float Step = FMath::Max(10.f, FMath::Min(AgentRadius * 0.5f, Voxel * 0.5f));
	const float Look = FMath::Max(AgentRadius * 4.f, Voxel * 2.f);
	const float E = FMath::Min(S + Look, Len);

	for (float d = S; d <= E; d += Step)
	{
		const FVector C = PathSplinePtr->GetLocationAtDistanceAlongSpline(d, ESplineCoordinateSpace::World);
		if (V->IsPointBlockedForAgent(C, AgentRadius, true, true))
		{
			return false;
		}
	}
	return true;
}
