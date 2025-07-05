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
	if (PathfindingTaskFuture.IsValid())
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

	CurrentTargetLocation = TargetLocation;
	// FoundPath.Empty();
	FoundPathIndices.Empty();
	// CurrentPathIndex = -1;

	TWeakObjectPtr<UPathfindingStrategy> StrategyPtr = PathfindingStrategy;
	const FIntVector StartIndex = GridVolume->WorldToVoxelIndex(GetOwner()->GetActorLocation());
	const FIntVector EndIndex = GridVolume->WorldToVoxelIndex(TargetLocation);
	const int32 AgentRadiusInVoxels = FMath::RoundToInt(AgentRadius / GridVolume->VoxelExtent);

	PathfindingTaskFuture = Async(EAsyncExecution::ThreadPool,
	                              [StrategyPtr, StartIndex, EndIndex, AgentRadiusInVoxels]() -> TArray<FIntVector>
	                              {
		                              TArray<FIntVector> PathIndices;
		                              if (StrategyPtr.IsValid())
		                              {
			                              StrategyPtr->FindPathWithStrategy(
				                              StartIndex, EndIndex, AgentRadiusInVoxels, PathIndices);
		                              }
		                              return PathIndices;
	                              });

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

    AdjustProgressToSpline(FinalVelocity);
	
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
		CurrentPathActor = World->SpawnActor<APathActor>(GetOwner()->GetActorLocation(), GetOwner()->GetActorRotation());
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

	if (PathfindingTaskFuture.IsValid()) //thread돌아가는지 확인
	{
		HandlePathfindingResult();
	}

	// const UEnum* EnumPtr = StaticEnum<EPathfindingState>();
	// if (EnumPtr)
	// {
	// 	UE_LOG(LogTemp, Warning, TEXT("CurrentState: %s"), *EnumPtr->GetNameStringByValue((int64)CurrentState));
	// }

	// UE_LOG(LogTemp, Warning, TEXT("NextPreferredLocation: %f %f %f"), NextPreferredLocation.X, NextPreferredLocation.Y, NextPreferredLocation.Z);

	switch (CurrentState)
	{
	case EPathfindingState::FollowPath:
		if (IsPathAheadValid())
		{
			DetermineMovementStrategy();			
			
			UpdateNextLocation(DeltaTime);
		}
		else
		{
			InvalidatePathAndRequestRepath();
		}
		break;
	case EPathfindingState::PathInvalidated:
	case EPathfindingState::Idle:
	default:
		break;
	}
}

void UPathfindingComponent::DetermineMovementStrategy()
{
	if (IsSplinePathSafe())
	{
		MovementMode = EPathMovementMode::Spline;
	}
	else
	{
		MovementMode = EPathMovementMode::Linear;
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

void UPathfindingComponent::HandlePathfindingResult()
{
	if (PathfindingTaskFuture.IsValid() && PathfindingTaskFuture.IsReady())
	{
		TArray<FIntVector> PathIndices = PathfindingTaskFuture.Get();
		PathfindingTaskFuture.Reset();
		ProcessNewPath(PathIndices);
	}
}

void UPathfindingComponent::ProcessNewPath(const TArray<FIntVector>& PathIndices)
{
	if (PathIndices.Num() < 2)
	{
		UE_LOG(LogTemp, Warning, TEXT("ProcessNewPath: Path not found or too short. Returning to Idle."));
		// CurrentState = EPathfindingState::Idle;
		GetWorld()->GetTimerManager().PauseTimer(PathOptimizationTimer);
		return;
	}

	FoundPathIndices = PathIndices;
	FoundPath.Empty();

	AUniformGridVolume* GridVolume = Cast<AUniformGridVolume>(TargetVolume);
	if (!ensure(GridVolume))
	{
		// CurrentState = EPathfindingState::Idle;
		return;
	}

	for (const FIntVector& Index : FoundPathIndices)
	{
		FoundPath.Add(GridVolume->VoxelIndexToWorld(Index));
	}

	//그대로 해도 살짝이상 살짝 dir방향으로 앞쪽에 해야할듯
	FoundPath[0] = GetOwner()->GetActorLocation();
	
	CurrentState = EPathfindingState::FollowPath;
	GetWorld()->GetTimerManager().UnPauseTimer(PathOptimizationTimer);

	bool bUseLinearFallback = false;

	if (PathSplinePtr.IsValid())
	{
		CurrentPathIndex = 0;
		DistanceAlongSpline = 0.f; //Update가 완료되면 Spline 0.f로 하기

		PathSplinePtr->ClearSplinePoints();
		for (const FVector& Point : FoundPath)
		{
			PathSplinePtr->AddSplinePoint(Point, ESplineCoordinateSpace::World, false);
		}
		PathSplinePtr->UpdateSpline();
		
		if (!bEnableSplineSafetyCheck || IsSplinePathSafe())
		{
			UE_LOG(LogTemp, Log, TEXT("Spline path is safe. Starting spline movement."));
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

	if (bDrawDebugPath)
	{
		for (int32 i = 0; i < FoundPath.Num() - 1; ++i)
		{
			DrawDebugLine(GetWorld(), FoundPath[i], FoundPath[i + 1], FColor::Green, false, 10.f, 0, 2.f);
		}
	}
}

void UPathfindingComponent::InvalidatePathAndRequestRepath()
{
	if (CurrentState == EPathfindingState::PathInvalidated) return;

	UE_LOG(LogTemp, Warning, TEXT("Path Invalidated! Stopping movement and scheduling a repath..."));

	// CurrentState = EPathfindingState::PathInvalidated;
	// FoundPath.Empty();
	FoundPathIndices.Empty();
	// CurrentPathIndex = -1;
	GetWorld()->GetTimerManager().PauseTimer(PathOptimizationTimer);
	// if (PathSplinePtr.IsValid())
	// {
	// 	PathSplinePtr->ClearSplinePoints();
	// }

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
		// CurrentState = EPathfindingState::Idle;
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
			// CurrentState = EPathfindingState::Idle;
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
		// CurrentState = EPathfindingState::Idle;
		return;
	}

	// 이거는 지역변수로 바꿔서 기록은 안하고 마지막에 최종 velocity로 내적한 값을 더해줘야함
	float PreferredDistance = DistanceAlongSpline + MovementSpeed * DeltaTime;
	
	// DistanceAlongSpline += MovementSpeed * DeltaTime;
	const float SplineLength = PathSplinePtr->GetSplineLength();

	if (DistanceAlongSpline >= SplineLength)
	{
		UE_LOG(LogTemp, Log, TEXT("Linear movement finished."));
		return;
	}

	NextPreferredLocation = PathSplinePtr->GetLocationAtDistanceAlongSpline(PreferredDistance, ESplineCoordinateSpace::World);
}

void UPathfindingComponent::AdjustProgressToSpline(FVector FinalVelocity)
{
	// 0. 유효성 검사: 따라갈 스플라인이 없으면 아무것도 하지 않습니다.
	if (!PathSplinePtr.IsValid())
	{
		UE_LOG(LogTemp, Warning, TEXT("PathfindingComponent: TargetSpline is not set."));
		return;
	}

	// 1. 현재 진행도(DistanceAlongSpline)에서의 스플라인 방향 벡터를 구합니다.
	// 방향은 정규화(길이가 1인 벡터)되어야 합니다.
	const FVector SplineDirection = PathSplinePtr->GetDirectionAtDistanceAlongSpline(DistanceAlongSpline, ESplineCoordinateSpace::World);
	
	// 3. 내적(Dot Product) 계산
	// 두 벡터를 내적하면, 한 벡터가 다른 벡터 방향으로 얼마나 이동했는지(투영된 길이)를 알 수 있습니다.
	// SplineDirection이 정규화된 벡터이므로, 내적의 결과는 FrameDisplacement 벡터를 SplineDirection에 투영한 길이가 됩니다.
	const float ForwardDistance = FVector::DotProduct(FinalVelocity, SplineDirection);

	// 4. 스플라인 진행도 업데이트
	DistanceAlongSpline += ForwardDistance;

	// 5. 스플라인 길이 초과 방지
	// 계산된 진행도가 스플라인의 전체 길이를 넘지 않도록 제한합니다.
	const float SplineLength = PathSplinePtr->GetSplineLength();
	DistanceAlongSpline = FMath::Clamp(DistanceAlongSpline, 0.f, SplineLength);

	UE_LOG(LogTemp, Warning, TEXT("DistanceAlongSpline: %f"), DistanceAlongSpline);
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
			// UE_LOG(LogTemp, Warning, TEXT("Obstacle detected on spline path."));
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
