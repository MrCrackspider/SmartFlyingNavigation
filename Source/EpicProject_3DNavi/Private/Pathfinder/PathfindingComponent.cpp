#include "Pathfinder/PathfindingComponent.h"
#include "Components/SplineComponent.h"
#include "Pathfinder/FenwickVoxelVolume.h"
#include "Pathfinder/PathfindingSubsystem.h"
#include "Pathfinder/AvoidanceSubsystem.h" // 새로운 회피 서브시스템 헤더
#include "Pathfinder/Strategy/PathfindingStrategy.h"
#include "GameFramework/Actor.h"
#include "Engine/World.h"
#include "DrawDebugHelpers.h"
#include "Pathfinder/PathActor.h"

UPathfindingComponent::UPathfindingComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
}


void UPathfindingComponent::OnPathfindingCompleted(
	const TArray<FIntVector>& PathIndices, bool bSuccess)
{
	if (bSuccess && PathIndices.Num() > 0)
	{
		ProcessNewPath(PathIndices);
		UE_LOG(LogTemp, Log, TEXT("Pathfinding successful. State changed to FollowPath."));
	}
	else
	{
		// 실패 시 '대기' 상태로 복귀
		CurrentState = EPathfindingState::Idle;
		UE_LOG(LogTemp, Warning, TEXT("Pathfinding failed. State changed to Idle."));
	}
}

bool UPathfindingComponent::FindPathToLocation(const FVector& TargetLocation)
{
	if (CurrentState == EPathfindingState::RequestingPath)
	{
		UE_LOG(LogTemp, Warning, TEXT("FindPathToLocation: Already requesting a path. New request ignored."));
		return false;
	}

	if (!PathfindingStrategy || !TargetVolume)
	{
		UE_LOG(LogTemp, Error, TEXT("FindPathToLocation failed: Strategy or TargetVolume is not set."));
		return false;
	}

	AUniformGridVolume* GridVolume = Cast<AUniformGridVolume>(TargetVolume);
	if (!GridVolume)
	{
		UE_LOG(LogTemp, Error, TEXT("FindPathToLocation failed: TargetVolume is not a UniformGridVolume."));
		return false;
	}

	this->PathSubsystem = GetWorld()->GetSubsystem<UPathfindingSubsystem>();
	
	if (!PathSubsystem)
	{
		UE_LOG(LogTemp, Error, TEXT("FindPathToLocation failed: PathfindingSubsystem not found."));
		return false;
	}

	FPathfindingRequest Request;
	Request.RequestingComponent = this;
	Request.Strategy = PathfindingStrategy;
	Request.StartIndex = GridVolume->WorldToVoxelIndex(GetOwner()->GetActorLocation());
	Request.EndIndex = GridVolume->WorldToVoxelIndex(TargetLocation);
	Request.AgentRadiusInVoxels = FMath::RoundToInt(AgentRadius / GridVolume->VoxelExtent);

	Request.OnPathfindingComplete.BindDynamic(this, &UPathfindingComponent::OnPathfindingCompleted);
	
	CurrentState = EPathfindingState::RequestingPath;
	CurrentTargetLocation = TargetLocation;
	FoundPath.Empty();
	FoundPathIndices.Empty();
	CurrentPathIndex = -1;

	const int32 RequestID = PathSubsystem->RequestPath(Request);

	if (RequestID == -1)
	{
		// 요청이 실패하면 즉시 '대기' 상태로 되돌립니다.
		CurrentState = EPathfindingState::Idle;
		return false;
	}

	// 요청이 성공적으로 큐에 들어갔습니다.
	return true;
}

void UPathfindingComponent::UpdateTargetLocation(const FVector& NewTargetLocation)
{
	if (CurrentState != EPathfindingState::FollowPath)
	{
		return;
	}
	if (FVector::DistSquared(CurrentTargetLocation, NewTargetLocation) < FMath::Square(GoalUpdateThreshold))
	{
		return;
	}

	UE_LOG(LogTemp, Log, TEXT("Target has moved significantly. Requesting new path."));

	FindPathToLocation(NewTargetLocation);
}

//======================================================================================
// Avoidance Interface
//======================================================================================

FVector UPathfindingComponent::GetPreferredVelocity() const
{
	if (CurrentState != EPathfindingState::FollowPath || !GetOwner())
	{
		return FVector::ZeroVector;
	}

	const FVector CurrentLocation = GetOwner()->GetActorLocation();
	const FVector Direction = (NextPreferredLocation - CurrentLocation).GetSafeNormal();
	return Direction * MovementSpeed;
}

void UPathfindingComponent::ApplyMovement(const FVector& FinalVelocity, float DeltaTime)
{
	AActor* Owner = GetOwner();
	if (!Owner) return;

	CurrentVelocity = FinalVelocity;
	const FVector CurrentLocation = Owner->GetActorLocation();
	Owner->SetActorLocation(CurrentLocation + FinalVelocity * DeltaTime);

	if (!FinalVelocity.IsNearlyZero())
	{
		Owner->SetActorRotation(FinalVelocity.GetSafeNormal().ToOrientationQuat());
	}
}


void UPathfindingComponent::CheckForPathOptimization()
{
	if (CurrentState != EPathfindingState::FollowPath || !PathfindingStrategy)
	{
		return;
	}
	if (PathfindingStrategy->ShouldRepathForOptimization(GetOwner()->GetActorLocation(),
														 FoundPath,
														 CurrentPathIndex,
														 CurrentTargetLocation, AgentRadius))
	{
		UE_LOG(LogTemp, Log, TEXT("Strategy suggests a shorter path is available. Requesting repath."));

		// 전략이 재탐색이 필요하다고 판단했으므로, 재탐색을 요청합니다.
		FindPathToLocation(CurrentTargetLocation);
	}
}

//======================================================================================
// Engine Overrides
//======================================================================================

void UPathfindingComponent::BeginPlay()
{
	Super::BeginPlay();

	UWorld* World = GetWorld();
	if (!ensure(World)) return;

	PathSubsystem = World->GetSubsystem<UPathfindingSubsystem>();
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

	if (!CurrentPathActor)
	{
		CurrentPathActor = World->SpawnActor<
			APathActor>(GetOwner()->GetActorLocation(), GetOwner()->GetActorRotation());
		if (CurrentPathActor)
		{
			PathSplinePtr = CurrentPathActor->GetSplineComponent();
		}
	}
	
	if (PathfindingStrategyClass)
	{
		PathfindingStrategy = NewObject<UPathfindingStrategy>(this, PathfindingStrategyClass);
		if (PathfindingStrategy)
		{
			PathfindingStrategy->Initialize(TargetVolume);
		}
	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT("PathfindingComponent on Actor '%s': PathfindingStrategyClass is not set!"),
		       *GetOwner()->GetName());
	}
}

void UPathfindingComponent::TickComponent(float DeltaTime, ELevelTick TickType,
                                          FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	switch (CurrentState)
	{
	case EPathfindingState::FollowPath:
		if (IsCurrentPathValid())
		{
			UpdateNextLocation(DeltaTime);
		}
		else
		{
			InvalidatePathAndRequestRepath();
		}
		break;

	case EPathfindingState::RequestingPath:
		//HandlePathfindingResult();
		break;

	case EPathfindingState::PathInvalidated:
	case EPathfindingState::Idle:
	default:
		break;
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


//======================================================================================
// Internal Logic
//======================================================================================

void UPathfindingComponent::ProcessNewPath(const TArray<FIntVector>& PathIndices)
{
	if (PathIndices.Num() < 2)
	{
		UE_LOG(LogTemp, Warning, TEXT("ProcessNewPath: Path not found or too short. Returning to Idle."));
		CurrentState = EPathfindingState::Idle;
		GetWorld()->GetTimerManager().PauseTimer(PathOptimizationTimer);
		return;
	}

	FoundPathIndices = PathIndices;
	FoundPath.Empty();

	AUniformGridVolume* GridVolume = Cast<AUniformGridVolume>(TargetVolume);
	if (!ensure(GridVolume))
	{
		CurrentState = EPathfindingState::Idle;
		return;
	}

	for (const FIntVector& Index : FoundPathIndices)
	{
		FoundPath.Add(GridVolume->VoxelIndexToWorld(Index));
	}

	CurrentPathIndex = 0;
	CurrentState = EPathfindingState::FollowPath;
	GetWorld()->GetTimerManager().UnPauseTimer(PathOptimizationTimer);

	bool bUseLinearFallback = false;
	if (MovementMode == EPathMovementMode::Spline)
	{
		if (PathSplinePtr.IsValid())
		{
			PathSplinePtr->ClearSplinePoints();
			for (const FVector& Point : FoundPath)
			{
				PathSplinePtr->AddSplinePoint(Point, ESplineCoordinateSpace::World, false);
			}
			PathSplinePtr->UpdateSpline();

			if (!bEnableSplineSafetyCheck || IsSplinePathSafe())
			{
				UE_LOG(LogTemp, Log, TEXT("Spline path is safe. Starting spline movement."));
				DistanceAlongSpline = 0.f;
			}
			else
			{
				UE_LOG(LogTemp, Warning, TEXT("Spline path is unsafe! Falling back to linear movement for this path."));
				bUseLinearFallback = true;
			}
		}
		else
		{
			bUseLinearFallback = true;
		}
	}

	if (bDrawDebugPath)
	{
		for (int32 i = 0; i < FoundPath.Num() - 1; ++i)
		{
			DrawDebugLine(GetWorld(), FoundPath[i], FoundPath[i + 1], FColor::Green, false, 10.f, 0, 2.f);
		}
	}
}

bool UPathfindingComponent::IsCurrentPathValid() const
{
	if (MovementMode == EPathMovementMode::Spline)
	{
		return IsSplinePathSafe();
	}
	return IsPathAheadValid();
}

void UPathfindingComponent::InvalidatePathAndRequestRepath()
{
	if (CurrentState == EPathfindingState::PathInvalidated) return;

	UE_LOG(LogTemp, Warning, TEXT("Path Invalidated! Stopping movement and scheduling a repath..."));

	CurrentState = EPathfindingState::PathInvalidated;
	FoundPath.Empty();
	FoundPathIndices.Empty();
	CurrentPathIndex = -1;
	GetWorld()->GetTimerManager().PauseTimer(PathOptimizationTimer);
	if (PathSplinePtr.IsValid())
	{
		PathSplinePtr->ClearSplinePoints();
	}

	if (UWorld* World = GetWorld())
	{
		World->GetTimerManager().SetTimer(RepathCooldownTimer, this, &UPathfindingComponent::OnRepathTimerFired,
		                                  RepathCooldown, false);
	}
}

void UPathfindingComponent::OnRepathTimerFired()
{
	FindPathToLocation(CurrentTargetLocation);
}

void UPathfindingComponent::UpdateNextLocation(float DeltaTime)
{
	// ProcessNewPath에서 결정된 임시 폴백을 고려할 수 있지만, 여기서는 MovementMode를 기준으로 단순하게 처리합니다.
	if (MovementMode == EPathMovementMode::Spline)
	{
		UpdateSplinePreferredLocation(DeltaTime);
	}
	else
	{
		UpdateLinearPreferredLocation(DeltaTime);
	}
}

void UPathfindingComponent::UpdateLinearPreferredLocation(float DeltaTime)
{
	AActor* Actor = GetOwner();
	if (!Actor || CurrentPathIndex < 0 || CurrentPathIndex >= FoundPath.Num())
	{
		CurrentState = EPathfindingState::Idle;
		return;
	}

	const FVector CurrentLocation = Actor->GetActorLocation();
	const FVector TargetPoint = FoundPath[CurrentPathIndex];

	const float ArrivalThresholdSq = FMath::Square(100.f);
	if (FVector::DistSquared(CurrentLocation, TargetPoint) < ArrivalThresholdSq)
	{
		CurrentPathIndex++;
		if (CurrentPathIndex >= FoundPath.Num())
		{
			CurrentState = EPathfindingState::Idle;
			UE_LOG(LogTemp, Log, TEXT("Linear movement finished."));
			return;
		}
	}

	const FVector Direction = (FoundPath[CurrentPathIndex] - CurrentLocation).GetSafeNormal();
	NextPreferredLocation = CurrentLocation + Direction * MovementSpeed * DeltaTime;
}

void UPathfindingComponent::UpdateSplinePreferredLocation(float DeltaTime)
{
	AActor* MyActor = GetOwner();
	if (!MyActor || !PathSplinePtr.IsValid() || PathSplinePtr->GetNumberOfSplinePoints() < 2)
	{
		CurrentState = EPathfindingState::Idle;
		return;
	}

	DistanceAlongSpline += MovementSpeed * DeltaTime;
	const float SplineLength = PathSplinePtr->GetSplineLength();

	if (DistanceAlongSpline >= SplineLength)
	{
		MyActor->SetActorLocation(FoundPath.Last());
		CurrentState = EPathfindingState::Idle;
		UE_LOG(LogTemp, Log, TEXT("Spline movement finished."));
		return;
	}

	NextPreferredLocation = PathSplinePtr->GetLocationAtDistanceAlongSpline(DistanceAlongSpline, ESplineCoordinateSpace::World);
}

bool UPathfindingComponent::IsSplinePathSafe() const
{
	if (!bEnableSplineSafetyCheck || !PathSplinePtr.IsValid() || !GetWorld())
	{
		return true;
	}

	const float SplineLength = PathSplinePtr->GetSplineLength();
	if (SplineLength < SafetyCheckStepSize)
	{
		return true;
	}

	FCollisionQueryParams CollisionParams(SCENE_QUERY_STAT(SplineSafetyCheck), false, GetOwner());
	FCollisionObjectQueryParams ObjectQueryParams(FCollisionObjectQueryParams::AllDynamicObjects);

	for (float Distance = 0; Distance < SplineLength; Distance += SafetyCheckStepSize)
	{
		FVector StartPoint = PathSplinePtr->GetLocationAtDistanceAlongSpline(Distance, ESplineCoordinateSpace::World);
		FVector EndPoint = PathSplinePtr->GetLocationAtDistanceAlongSpline(
			FMath::Min(Distance + SafetyCheckStepSize, SplineLength), ESplineCoordinateSpace::World);

		if (FVector::DistSquared(StartPoint, EndPoint) < KINDA_SMALL_NUMBER)
		{
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
	AFenwickVoxelVolume* FenwickVolume = Cast<AFenwickVoxelVolume>(TargetVolume);
	if (!FenwickVolume || FoundPathIndices.Num() < 2 || CurrentPathIndex < 0 || CurrentPathIndex >= FoundPathIndices.
		Num() - 1)
	{
		return true;
	}

	AUniformGridVolume* GridVolume = Cast<AUniformGridVolume>(TargetVolume);
	if (!GridVolume) return false;

	const int32 AgentRadiusInVoxels = FMath::CeilToInt(AgentRadius / GridVolume->VoxelExtent);
	if (AgentRadiusInVoxels <= 0) return true;

	const int32 LastCheckIndex = FMath::Min(CurrentPathIndex + PathValidationLookAhead, FoundPathIndices.Num() - 2);
	for (int32 i = CurrentPathIndex; i <= LastCheckIndex; ++i)
	{
		if (!FenwickVolume->IsPathSegmentWalkable(FoundPathIndices[i], FoundPathIndices[i + 1], AgentRadiusInVoxels))
		{
			UE_LOG(LogTemp, Warning,
			       TEXT("Path Invalidated: A future path segment (from waypoint %d to %d) is blocked!"), i, i + 1);
			return false;
		}
	}

	return true;
}
