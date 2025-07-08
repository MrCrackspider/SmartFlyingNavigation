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
		CurrentState = EPathfindingState::Idle;
		UE_LOG(LogTemp, Warning, TEXT("Pathfinding failed. State changed to Idle."));
	}
	
	if (PeriodicRepathOpt)
	{
		//도착하지 않았으면
		if (!bIsReachDestination)
		{
			if (UWorld* World = GetWorld())
			{
				//재귀적으로 요청
				World->GetTimerManager().SetTimer(RepathCooldownTimer, this, &UPathfindingComponent::PeriodicRepath,
												  RepathCooldown, false);
			}
		}
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

	// 기존에 타이머 걸어둔게 있으면 취소
	if (UWorld* World = GetWorld())
	{
		World->GetTimerManager().ClearTimer(RepathCooldownTimer);
	}
	
	AUniformGridVolume* GridVolume = Cast<AUniformGridVolume>(TargetVolume);
	if (!GridVolume)
	{
		UE_LOG(LogTemp, Error, TEXT("FindPathToLocation failed: TargetVolume is not a UniformGridVolume."));
		return false;
	}
	
	CurrentTargetLocation = TargetLocation;
	FoundPathIndices.Empty();
	
	// 2. "요청서"를 작성합니다.
	FPathfindingRequest Request;
	Request.RequestingComponent = this;
	Request.Strategy = PathfindingStrategy;
	Request.StartIndex = GridVolume->WorldToVoxelIndex(GetOwner()->GetActorLocation());
	Request.EndIndex = GridVolume->WorldToVoxelIndex(TargetLocation);
	Request. AgentRadiusInVoxels = FMath::CeilToInt((AgentRadius / GridVolume->VoxelExtent) - 0.5f);

	//    결과를 받을 콜백 함수를 지정합니다.
	Request.OnPathfindingComplete.BindDynamic(this, &UPathfindingComponent::OnPathfindingCompleted);
	
	CurrentRequestID = PathSubsystem->RequestPath(Request);

	if (CurrentRequestID != -1)
	{
		// 상태 변경 등...
		return true;
	}

	return false;
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
	
	// 1. 실제 이동 및 회전 (기존과 동일)
	const FVector CurrentLocation = Owner->GetActorLocation();

	CurrentVelocity = FinalVelocity;
	
	const FVector AdjustedVelocity = FinalVelocity * DeltaTime;
	
	Owner->SetActorLocation(CurrentLocation + AdjustedVelocity);
	if (!FinalVelocity.IsNearlyZero())
	{
		Owner->SetActorRotation(FinalVelocity.GetSafeNormal().ToOrientationQuat());
	}

	AdjustProgressToSpline(AdjustedVelocity);
}

void UPathfindingComponent::AdjustProgressToSpline(FVector FinalVelocity)
{
	// 0. 유효성 검사: 따라갈 스플라인이 없으면 아무것도 하지 않습니다.
	if (!PathSplinePtr.IsValid())
	{
		UE_LOG(LogTemp, Warning, TEXT("PathfindingComponent: TargetSpline is not set."));
		return;
	}

	const FVector SplineDirection = PathSplinePtr->GetDirectionAtDistanceAlongSpline(DistanceAlongSpline, ESplineCoordinateSpace::World);
	
	const float ForwardDistance = FVector::DotProduct(FinalVelocity, SplineDirection);
	DistanceAlongSpline += ForwardDistance;
	const float SplineLength = PathSplinePtr->GetSplineLength();
	DistanceAlongSpline = FMath::Clamp(DistanceAlongSpline, 0.f, SplineLength);
	// UE_LOG(LogTemp, Warning, TEXT("DistanceAlongSpline: %f"), DistanceAlongSpline);
	
	//DistanceAlongSpline으로 현재 목표포인트인덱스 업데이트
	if (!bIsReachDestination)
	{
		const FVector CurrentLocation = GetOwner()->GetActorLocation();
		const FVector TargetPoint = FoundPath[CurrentPathDestinationIndex];
		
		if (MovementMode == EPathMovementMode::Spline/* && CurrentPathIndex != 0*/)
		{
			const float TargetPointDistance = PathSplinePtr->GetDistanceAlongSplineAtSplinePoint(CurrentPathDestinationIndex);
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
	DistanceAlongSpline = PathSplinePtr->GetDistanceAlongSplineAtSplinePoint(CurrentPathDestinationIndex);
				
	int32 PassedPointIndex = CurrentPathDestinationIndex;
	if (PassedPointIndex >= 0)
	{
		UE_LOG(LogTemp, Display, TEXT("%s: passed point %d"), *GetOwner()->GetName(), PassedPointIndex);
		UE_LOG(LogTemp, Display, TEXT("%s: FoundPath.Num() %d"), *GetOwner()->GetName(), FoundPath.Num());
	}
	
	++CurrentPathDestinationIndex;
	
	if (CurrentPathDestinationIndex >= FoundPath.Num())
	{
		ReachDestination();
		UE_LOG(LogTemp, Log, TEXT("Spline movement finished."));
		return;
	}
	
	DetermineMovementModeForNextPath();
}

bool UPathfindingComponent::IsOnSplineLine()
{
	AActor* MyActor = GetOwner();
	if (!MyActor)
	{
		UE_LOG(LogTemp, Warning, TEXT("PathfindingComponent: Actor is not set."));
		return false;
	}

	const FVector MyLocation = MyActor->GetActorLocation();
	const FVector SplineLocation = PathSplinePtr->GetLocationAtDistanceAlongSpline(DistanceAlongSpline, ESplineCoordinateSpace::World);

	//원래 1이였음 판정 심하게 넉넉하면 1로 줄이기
	const float MyTolerance = 100.0f; 

	if (MyLocation.Equals(SplineLocation, MyTolerance))
	{
		return true;
	}

	UE_LOG(LogTemp, Warning, TEXT("%s is not on Spline line"), *GetOwner()->GetName());
	return false;
}

void UPathfindingComponent::DetermineMovementModeForNextPath()
{
	UE_LOG(LogTemp, Warning, TEXT("Go to Next Point"));
	UE_LOG(LogTemp, Warning, TEXT("From: index_%d Loc_%.2f %.2f %.2f | To: index_%d Loc_%.2f %.2f %.2f"),
		CurrentPathDestinationIndex-1, FoundPath[CurrentPathDestinationIndex-1].X, FoundPath[CurrentPathDestinationIndex-1].Y, FoundPath[CurrentPathDestinationIndex-1].Z,
		CurrentPathDestinationIndex, FoundPath[CurrentPathDestinationIndex].X, FoundPath[CurrentPathDestinationIndex].Y, FoundPath[CurrentPathDestinationIndex].Z);
	
	if (IsSplinePathSafe())
	{
		MovementMode = EPathMovementMode::Spline;
		UE_LOG(LogTemp, Warning, TEXT("%s: Path is safe to next point! Move Spline"), *GetOwner()->GetName());
	}
	else
	{
		MovementMode = EPathMovementMode::Linear;
		UE_LOG(LogTemp, Warning, TEXT("%s: Path is unsafe to next point! Move Linear"), *GetOwner()->GetName());
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
														 CurrentPathDestinationIndex,
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
	if (TargetVolume)
	{
		TargetVolume->RegisterStreamingAgent(GetOwner());
	}
}

void UPathfindingComponent::PeriodicRepath()
{	
	UE_LOG(LogTemp, Warning, TEXT("RePathing"));
	FindPathToLocation(CurrentTargetLocation);
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

void UPathfindingComponent::TickComponent(float DeltaTime, ELevelTick TickType,
                                          FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (!IsPathAheadValid()) //여기가 리패스하는 곳인듯
	{
		UE_LOG(LogTemp, Warning, TEXT("Dynamic obstacle is on path. Start Repath"));
		InvalidatePathAndRequestRepath();
	}
	
	if (PathfindingTaskFuture.IsValid()) //thread돌아가는지 확인
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
	MovementMode = EPathMovementMode::Spline; //움직임모드 설정할 수 있게 할거면 하드하게 설정하지 말고 변수로 대체
	GetWorld()->GetTimerManager().UnPauseTimer(PathOptimizationTimer);

	if (PathSplinePtr.IsValid())
	{
		CurrentPathDestinationIndex = 0;
		DistanceAlongSpline = 0.f; //Update가 완료되면 Spline 0.f로 하기
		bIsReachDestination = false;

		PathSplinePtr->ClearSplinePoints();
		UE_LOG(LogTemp, Warning, TEXT("Create Spline index"));
		for (int i=0;i<FoundPath.Num();i++)
		{
			UE_LOG(LogTemp, Warning, TEXT("index: %d, Location: %.2f %.2f %.2f"), i, FoundPath[i].X, FoundPath[i].Y, FoundPath[i].Z);
			PathSplinePtr->AddSplinePoint(FoundPath[i], ESplineCoordinateSpace::World, false);
		}
		PathSplinePtr->UpdateSpline();
	}
	
	if (bDrawDebugPath)
	{
		for (int32 i = 0; i < FoundPath.Num() - 1; ++i)
		{
			DrawDebugLine(GetWorld(), FoundPath[i], FoundPath[i + 1], FColor::Red, false, 10.f, 0, 2.f);
		}
	}
}

void UPathfindingComponent::InvalidatePathAndRequestRepath()
{
	if (CurrentState == EPathfindingState::PathInvalidated) return;

	UE_LOG(LogTemp, Warning, TEXT("Path Invalidated! Stopping movement and scheduling a repath..."));
	
	FoundPathIndices.Empty();

	GetWorld()->GetTimerManager().PauseTimer(PathOptimizationTimer);
	
	if (UWorld* World = GetWorld())
	{
		//subsystem에서 스케줄링 해줄거니까 repath 시간 둘 필요 없음
		OnRepathTimerFired();
	}
}

void UPathfindingComponent::OnRepathTimerFired()
{
	FindPathToLocation(CurrentTargetLocation);
}

void UPathfindingComponent::UpdatePreferredVelocity(float DeltaTime)
{
	if (MovementMode == EPathMovementMode::Spline)
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
	
	const FVector SplineDirection = PathSplinePtr->GetDirectionAtDistanceAlongSpline(DistanceAlongSpline, ESplineCoordinateSpace::World);
	
	PreferredVelocity = SplineDirection * MovementSpeed;
}

void UPathfindingComponent::ReachDestination()
{
	UE_LOG(LogTemp, Warning, TEXT("Reach the Destination!"));
	// PreferredVelocity = FVector::ZeroVector;
	bIsReachDestination = true;
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

	//현재 포인트부터 다음포인트까지 훑어야 하기 때문에 0~SplineLength가 아니라 현재포인트Distance~ 다음포인트 Distance까지 해야함.

	const float CurrentDistance = PathSplinePtr->GetDistanceAlongSplineAtSplinePoint(CurrentPathDestinationIndex - 1);
	const float NextDistance = PathSplinePtr->GetDistanceAlongSplineAtSplinePoint(CurrentPathDestinationIndex);

	// UE_LOG(LogTemp, Warning, TEXT("CurrentDistance: %f, NextDistance: %f"), CurrentDistance, NextDistance);
	
	for (float Distance = CurrentDistance; Distance < NextDistance; Distance += SafetyCheckStepSize)
	{
		// UE_LOG(LogTemp, Warning, TEXT("Checking... Distance: %f"), Distance);
		
		
		FVector StartPoint = PathSplinePtr->GetLocationAtDistanceAlongSpline(Distance, ESplineCoordinateSpace::World);
		FVector EndPoint = PathSplinePtr->GetLocationAtDistanceAlongSpline(
			FMath::Min(Distance + SafetyCheckStepSize, SplineLength), ESplineCoordinateSpace::World);

		// 디버그 구체를 그립니다.
		// DrawDebugSphere(
		// 	GetWorld(),                  // 구체를 그릴 월드
		// 	StartPoint,             // 구체의 중심 위치: StartPoint
		// 	AgentRadius,            // 구체의 반지름: AgentRadius
		// 	6,                     // 구체의 정밀도 (12 또는 24 정도면 충분합니다)
		// 	FColor::Green,          // 색상: 녹색
		// 	false,                  // 한 프레임만 표시할지 여부
		// 	5.0f,                   // 표시 시간: 5초
		// 	0,                      // 렌더링 우선순위
		// 	2.0f                    // 선의 두께 (추가적인 파라미터)
		// );
		
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
	AFenwickVoxelVolume* FenwickVolume = Cast<AFenwickVoxelVolume>(TargetVolume);
	if (!FenwickVolume || FoundPathIndices.Num() < 2 || CurrentPathDestinationIndex < 0 || CurrentPathDestinationIndex >= FoundPathIndices.
		Num() - 1)
	{
		return true;
	}

	AUniformGridVolume* GridVolume = Cast<AUniformGridVolume>(TargetVolume);
	if (!GridVolume || GridVolume->VoxelExtent == 0) return false;

	const int32 AgentRadiusInVoxels = FMath::CeilToInt((AgentRadius / GridVolume->VoxelExtent) - 0.5f);
	
	const int32 LastCheckIndex = FMath::Min(CurrentPathDestinationIndex + PathValidationLookAhead, FoundPathIndices.Num() - 2);
	for (int32 i = CurrentPathDestinationIndex; i <= LastCheckIndex; ++i)
	{
		if (!FenwickVolume->IsRegionWalkable(FoundPathIndices[i], AgentRadiusInVoxels)) //테스트 필요함.
		//if (!FenwickVolume->IsPathSegmentWalkable(FoundPathIndices[i], FoundPathIndices[i + 1], AgentRadiusInVoxels))
		{
			UE_LOG(LogTemp, Warning,
			       TEXT("Path Invalidated: A future path segment (from waypoint %d to %d) is blocked!"), i, i + 1);
			return false;
		}
	}

	return true;
}
