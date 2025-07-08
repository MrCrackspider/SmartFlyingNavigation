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
	Request.AgentRadiusInVoxels = FMath::CeilToInt((AgentRadius - GridVolume->VoxelExtent / 2) / GridVolume->VoxelExtent);

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
	// UE_LOG(LogTemp, Warning, TEXT("DistanceAlongSpline: %f"), DistanceAlongSpline);
	
	//DistanceAlongSpline으로 현재 목표포인트인덱스 업데이트
	if (!bIsReachDestination)
	{

		const FVector CurrentLocation = GetOwner()->GetActorLocation();
		const FVector TargetPoint = FoundPath[CurrentPathIndex];

		const float ArrivalThresholdSq = FMath::Square(10.f);

		// 스플라인인지 linear인지 분기처리해서 따로 if문 처리할 수 있지만 저걸로 전부 처리 가능할거같아서 일단 이렇게 처리
		// if (DistanceAlongSpline >= TargetPointDistance)
		if (FVector::DistSquared(CurrentLocation, TargetPoint) < ArrivalThresholdSq)
		{
			//직선이 더 빠르기때문에 특정포인트에 도달하면 동기화시켜줘야함
			DistanceAlongSpline = PathSplinePtr->GetDistanceAlongSplineAtSplinePoint(CurrentPathIndex);
			
			int32 PassedPointIndex = CurrentPathIndex - 1;
			if (PassedPointIndex >= 0)
			{
				UE_LOG(LogTemp, Warning, TEXT("passed point %d"), PassedPointIndex);
			}

			//포인트 통과시 실행할 코드 있으면 삽입

			ProgressNextPoint();
		}
	}
}

void UPathfindingComponent::ProgressNextPoint()
{
	CurrentPathIndex++;
	if (CurrentPathIndex >= FoundPath.Num())
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

	const float MyTolerance = 1.0f; 

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
		MovementMode = EPathMovementMode::Spline;
		UE_LOG(LogTemp, Warning, TEXT("Path is safe to next point! Move Spline"));
	}else
	{
		MovementMode = EPathMovementMode::Linear;
		UE_LOG(LogTemp, Warning, TEXT("Path is unsafe to next point! Move Linear"));
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
			if (IsPathAheadValid()) //여기가 리패스하는 곳인듯
			{
				UpdatePreferredVelocity(DeltaTime);
			}
			else
			{
				InvalidatePathAndRequestRepath();
			}
		}
	}
	
	// switch (CurrentState)
	// {
	// case EPathfindingState::FollowPath:
	// 	if (IsPathAheadValid()) //여기가 리패스하는 곳인듯
	// 	{
	// 		UpdatePreferredVelocity(DeltaTime);
	// 	}
	// 	else
	// 	{
	// 		InvalidatePathAndRequestRepath();
	// 	}
	// 	break;
	// case EPathfindingState::PathInvalidated:
	// case EPathfindingState::Idle:
	// default:
	// 	break;
	// }
}

void UPathfindingComponent::TickComponent(float DeltaTime, ELevelTick TickType,
                                          FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// avoidance때문에 경로를 벗어나면 일단 linear하게 움직이도록 땜빵?처리 (기존의방식)
	if (MovementMode == EPathMovementMode::Spline && IsOnSplineLine())
	{
		UE_LOG(LogTemp, Warning, TEXT("Out Of Spline Line. Change MovementMode to Linear"));
		MovementMode = EPathMovementMode::Linear;
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
	GetWorld()->GetTimerManager().UnPauseTimer(PathOptimizationTimer);

	if (PathSplinePtr.IsValid())
	{
		CurrentPathIndex = 0;
		DistanceAlongSpline = 0.f; //Update가 완료되면 Spline 0.f로 하기
		bIsReachDestination = false;

		PathSplinePtr->ClearSplinePoints();
		for (const FVector& Point : FoundPath)
		{
			PathSplinePtr->AddSplinePoint(Point, ESplineCoordinateSpace::World, false);
		}
		PathSplinePtr->UpdateSpline();
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
	if (!Actor || CurrentPathIndex < 0 || CurrentPathIndex >= FoundPath.Num())
	{
		return;
	}

	const FVector CurrentLocation = Actor->GetActorLocation();
	const FVector TargetPoint = FoundPath[CurrentPathIndex];

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

	const float CurrentDistance = PathSplinePtr->GetDistanceAlongSplineAtSplinePoint(CurrentPathIndex - 1);
	const float NextDistance = PathSplinePtr->GetDistanceAlongSplineAtSplinePoint(CurrentPathIndex);

	// UE_LOG(LogTemp, Warning, TEXT("CurrentDistance: %f, NextDistance: %f"), CurrentDistance, NextDistance);
	
	for (float Distance = CurrentDistance; Distance < NextDistance; Distance += SafetyCheckStepSize)
	{
		// UE_LOG(LogTemp, Warning, TEXT("Checking... Distance: %f"), Distance);
		
		
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
		//if (!FenwickVolume->IsRegionWalkable(FoundPathIndices[i], AgentRadiusInVoxels)) 테스트 필요함.
		if (!FenwickVolume->IsPathSegmentWalkable(FoundPathIndices[i], FoundPathIndices[i + 1], AgentRadiusInVoxels))
		{
			UE_LOG(LogTemp, Warning,
			       TEXT("Path Invalidated: A future path segment (from waypoint %d to %d) is blocked!"), i, i + 1);
			return false;
		}
	}

	return true;
}
