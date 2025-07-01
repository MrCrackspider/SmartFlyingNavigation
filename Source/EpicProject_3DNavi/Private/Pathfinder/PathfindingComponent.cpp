#include "Kismet/KismetSystemLibrary.h" // SphereTrace, CapsuleTrace를 위해 필요

#include "Components/SplineComponent.h"
#include "Pathfinder/FenwickVoxelVolume.h"
#include "Pathfinder/PathfindingComponent.h"
#include "Pathfinder/PathfindingSubsystem.h"
#include "Pathfinder/AvoidanceSubsystem.h" // 새로운 회피 서브시스템 헤더
#include "Pathfinder/Strategy/PathfindingStrategy.h"
#include "GameFramework/Actor.h"
#include "Engine/World.h"
#include "DrawDebugHelpers.h"

UPathfindingComponent::UPathfindingComponent()
{
    PrimaryComponentTick.bCanEverTick = true;

    // 변수 초기화
    CurrentPathIndex = -1;
    MovementSpeed = 500.f;
    Radius = 50.f;
    CurrentState = EPathfindingState::Idle;
	DistanceAlongSpline = 0.f;

	if (PathSpline == nullptr)
	{
		PathSpline = CreateDefaultSubobject<USplineComponent>(TEXT("PathSpline"));
	}
}

void UPathfindingComponent::BeginPlay()
{
    Super::BeginPlay();

    UWorld* World = GetWorld();
    if (!World) return;

    // 길찾기 및 회피 서브시스템을 찾아서 저장합니다.
    PathSubsystem = World->GetSubsystem<UPathfindingSubsystem>();
    AvoidanceSubsystem = World->GetSubsystem<UAvoidanceSubsystem>();

    // 회피 서브시스템에 자신을 등록합니다.
    if (AvoidanceSubsystem)
    {
        AvoidanceSubsystem->RegisterAgent(this);
    }

    if (!PathSubsystem)
    {
        UE_LOG(LogTemp, Error, TEXT("PathfindingComponent: PathfindingSubsystem not found!"));
        return;
    }

    // TargetVolume 설정 (PathSubsystem을 통해)
    const TArray<TObjectPtr<AVoxelVolume>>& Volumes = PathSubsystem->GetVoxelVolumes();
    if (Volumes.Num() > 0)
    {
		// FIX-ME: 현재는 첫 번째 볼륨을 타겟으로 설정합니다.
        // 지금 현재 0번째 볼륨을 타겟으로 설정하지만, 추후 볼륨의 위치를 사용해서 적용하도록 변경 예정
        TargetVolume = Volumes[0];
    }

    // 길찾기 전략 초기화
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
        UE_LOG(LogTemp, Error, TEXT("PathfindingComponent on Actor '%s': PathfindingStrategyClass is not set!"), *GetOwner()->GetName());
    }
}

void UPathfindingComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    // 회피 서브시스템에서 자신을 등록 해제합니다.
    if (AvoidanceSubsystem)
    {
        AvoidanceSubsystem->UnregisterAgent(this);
    }

    GetWorld()->GetTimerManager().ClearTimer(RepathCooldownTimer);
    Super::EndPlay(EndPlayReason);
}

void UPathfindingComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	switch (CurrentState)
	{
	case EPathfindingState::FollowSpline:
	case EPathfindingState::FollowLinear:
		// 이동 중일 때, 경로가 막히지 않았는지 계속 확인합니다.
		CheckAndHandlePathInvalidation();
		// HandleMovingState(DeltaTime);
		break;
		
	case EPathfindingState::RequestingPath:
		HandleRequestingPathState();
		break;
		
	case EPathfindingState::PathInvalidated:
	case EPathfindingState::Idle:
	default:
		// 이동 중이 아니거나 재탐색 대기 중일 때는 아무것도 하지 않음
		break;
	}
}

bool UPathfindingComponent::FindPathToLocation(const FVector& TargetLocation)
{
    if (CurrentState == EPathfindingState::RequestingPath || !PathfindingStrategy || !TargetVolume)
    {
        return false;
    }

    CurrentTargetLocation = TargetLocation;
    CurrentState = EPathfindingState::RequestingPath;
    FoundPath.Empty();
    FoundPathIndices.Empty();
    CurrentPathIndex = -1;

    TWeakObjectPtr<UPathfindingStrategy> StrategyPtr = PathfindingStrategy;
    const FIntVector StartIndex = TargetVolume->WorldToVoxelIndex(GetOwner()->GetActorLocation());
    const FIntVector EndIndex = TargetVolume->WorldToVoxelIndex(TargetLocation);
    const float AgentRadius = Radius;
    const int32 AgentRadiusInVoxels = FMath::CeilToInt(AgentRadius / TargetVolume->GetVoxelExtent());

    PathfindingTaskFuture = Async(EAsyncExecution::ThreadPool,
        [StrategyPtr, StartIndex, EndIndex, AgentRadiusInVoxels]() -> TArray<FIntVector>
        {
            TArray<FIntVector> PathIndices;
            if (StrategyPtr.IsValid())
            {
                StrategyPtr->FindPathWithStrategy(StartIndex, EndIndex, AgentRadiusInVoxels, PathIndices);
            }
            return PathIndices;
        });

    return true;
}

void UPathfindingComponent::FollowSplinePath(float DeltaTime)
{
	AActor* MyActor = GetOwner();

	//따라갈 스플라인이 없거나 점이 부족하거나 주인이 없으면 이동 중지
	if (!MyActor || !PathSpline || PathSpline->GetNumberOfSplinePoints() < 2)
	{
		CurrentState = EPathfindingState::Idle;
		return;
	}

	// 1. 이번 프레임에 이동할 거리를 누적합니다.
	DistanceAlongSpline += MovementSpeed * DeltaTime;

	// 2. 스플라인의 총 길이를 가져옵니다.
	const float SplineLength = PathSpline->GetSplineLength();

	// 3. 스플라인의 총 길이를 가져옵니다.
	if (DistanceAlongSpline >= SplineLength)
	{
		// 도착 완료: 캐릭터를 경로의 마지막 지점에 위치시킵니다.
		const FVector FinalDestination = FoundPath.Last();
		MyActor->SetActorLocation(FinalDestination);

		CurrentState = EPathfindingState::Idle; //상태를 대기로 변경
		UE_LOG(LogTemp, Log, TEXT("MovemnetFinished"));
		return;
	}

	// 4. 아직 이동중이라면, 스플라인 상의 새로운 위치와 회전값을 계산
	const FVector NewLocation = PathSpline->GetLocationAtDistanceAlongSpline(DistanceAlongSpline, ESplineCoordinateSpace::World);
	const FRotator NewRotation = PathSpline->GetRotationAtDistanceAlongSpline(DistanceAlongSpline, ESplineCoordinateSpace::World);

	// 5. 캐릭터의 위치와 회전을 부드럽게 업데이트합니다.
	// RInterpTo를 사용하면 더 부드러워짐
	const FRotator SmoothedRotation = FMath::RInterpTo(MyActor->GetActorRotation(), NewRotation, DeltaTime, 10.0f);
	MyActor->SetActorLocationAndRotation(NewLocation, SmoothedRotation);
}

void UPathfindingComponent::CreateSplineFromPath()
{
	if (!PathSpline || FoundPath.Num() < 2)
	{
		//경로가 유효하지 않으면 이동을 중지?
		CurrentState = EPathfindingState::Idle;
		return;
	}

	// 1. 기존 스플라인 정보를 모두 지웁니다.
	PathSpline->ClearSplinePoints();

	// 2. FoundPath의 각 점의 스플라인 포인트로 추가합니다.
	for (const FVector& Point: FoundPath)
	{
		PathSpline->AddSplinePoint(Point, ESplineCoordinateSpace::World, true);
	}

	// 3. 스플라인 내부 데이터를 강제로 업데이트합니다. (길이 계산 등)
	PathSpline->UpdateSpline();

	if (bEnableSplineSafetyCheck && !IsSplinePathSafe())
	{
		UE_LOG(LogTemp, Warning, TEXT("Spline Path is unsafe! Falling back to linear movement"));
		CurrentState = EPathfindingState::FollowLinear;
		CurrentPathIndex = 0;
	}else //안전할 경우 spline사용
	{
		UE_LOG(LogTemp, Log, TEXT("Spline path is safe, Starting spline movement"));
		DistanceAlongSpline = 0.f;
		CurrentState = EPathfindingState::FollowSpline;
	}
	
	// 4. 이동을 시작할 준비를 합니다.
	DistanceAlongSpline = 0.0f; //이동거리 초기화

	UE_LOG(LogTemp, Log, TEXT("Spline created with %d points. Starting Movement"), FoundPath.Num());
}

bool UPathfindingComponent::IsSplinePathSafe()
{
	if (!PathSpline || !GetWorld())
	{
		return false;
	}  

	const float SplineLength = PathSpline->GetSplineLength();
	
	if (SplineLength < SafetyCheckStepSize)
	{
		//경로가 너무 짧으면 그냥 안전하다고 판단
		return true;
	}

	//스플라인을 따라 일정한 간격으로 샘플링
	for (float Distance = 0; Distance<SplineLength; Distance+=SafetyCheckStepSize)
	{
		FVector StartPoint = PathSpline->GetLocationAtDistanceAlongSpline(Distance, ESplineCoordinateSpace::World);
		FVector EndPoint = PathSpline->GetLocationAtDistanceAlongSpline(FMath::Min(Distance + SafetyCheckStepSize, SplineLength), ESplineCoordinateSpace::World);

		// 시작점과 끝점이 거의 같다면 검사 생략
		if (FVector::DistSquared(StartPoint, EndPoint) < 1.0f)
		{
			continue;
		}
		
		FHitResult HitResult;
		// 물리 쿼리를 위한 파라미터 설정
		FCollisionObjectQueryParams DynamicObjectQueryParams;
		DynamicObjectQueryParams.AddObjectTypesToQuery(ECC_WorldStatic);
		DynamicObjectQueryParams.AddObjectTypesToQuery(ECC_WorldDynamic);
		DynamicObjectQueryParams.AddObjectTypesToQuery(ECC_PhysicsBody);

		FCollisionQueryParams CollisionParams;
		CollisionParams.AddIgnoredActor(GetOwner()); //자기는 무시

		TArray<AActor*> IgnoredActors;
		IgnoredActors.Add(GetOwner());

		FVector Extents = FVector(Radius);
		
		bool bHit = GetWorld()->SweepSingleByObjectType(
			HitResult,
			StartPoint,
			EndPoint,
			FQuat::Identity, // Sweep 함수는 FQuat 타입을 받습니다.
			DynamicObjectQueryParams,
			FCollisionShape::MakeBox(Extents), // 박스의 크기를 지정
			CollisionParams
		);

		DrawDebugBox(GetWorld(), HitResult.bBlockingHit ? HitResult.Location : EndPoint, Extents, FQuat::Identity, bHit ? FColor::Red : FColor::Green, false, 2.f, 0, 1.f);

		if (bHit)
		{
			//장애물이 감지되면 즉시 안전하지 않음을 반환
			UE_LOG(LogTemp, Warning, TEXT("Obstacle detected on spline path at location: %s"), *HitResult.Location.ToString());
			return false;
		}
	}

	//샘플구간이 전부 안전하면 안전함을 반환
	return true;
}

FVector UPathfindingComponent::GetPreferredVelocity() const{
    AActor* Owner = GetOwner();
    // 이동 중이 아니거나, 경로가 없거나, 주인이 없으면 멈춥니다.
    if (!Owner || (CurrentState != EPathfindingState::FollowSpline && CurrentState != EPathfindingState::FollowLinear) || CurrentPathIndex < 0 || CurrentPathIndex >= FoundPath.Num())
    {
        return FVector::ZeroVector;
    }
    // 다음 목표 지점을 향하는 방향과 속도를 계산하여 반환합니다.
    const FVector CurrentLocation = Owner->GetActorLocation();
    const FVector TargetPoint = FoundPath[CurrentPathIndex];
    const FVector Direction = (TargetPoint - CurrentLocation).GetSafeNormal();

    return Direction * MovementSpeed;
}

void UPathfindingComponent::ApplyMovement(const FVector& FinalVelocity, float DeltaTime)
{
    AActor* Owner = GetOwner();
    if (!Owner) return;
    CurrentVelocity = FinalVelocity;

    // 1. 회피 시스템이 계산해준 최종 속도로 액터를 이동 및 회전시킵니다.
    const FVector CurrentLocation = Owner->GetActorLocation();
    Owner->SetActorLocation(CurrentLocation + FinalVelocity * DeltaTime);

    if (!FinalVelocity.IsNearlyZero())
    {
        Owner->SetActorRotation(FinalVelocity.GetSafeNormal().ToOrientationQuat());
    }

    // 2. 현재 경로의 진행 상황을 업데이트합니다. (목표 지점 도착 체크)
    if ((CurrentState == EPathfindingState::FollowSpline || CurrentState == EPathfindingState::FollowLinear) && CurrentPathIndex >= 0 && CurrentPathIndex < FoundPath.Num())
    {
        const FVector TargetPoint = FoundPath[CurrentPathIndex];
        const float ArrivalThresholdSq = FMath::Square(100.f);

        if (FVector::DistSquared(CurrentLocation, TargetPoint) < ArrivalThresholdSq)
        {
            CurrentPathIndex++;
            if (CurrentPathIndex >= FoundPath.Num())
            {
                CurrentState = EPathfindingState::Idle;
                CurrentPathIndex = -1;
                UE_LOG(LogTemp, Log, TEXT("Movement Finished."));
            }
        }
    }
}

void UPathfindingComponent::HandleMovingState(float DeltaTime)
{
	// 1. 먼저 경로가 막혔는지 확인하고 처리
	if (CheckAndHandlePathInvalidation())
	{
		return;
	}

	// 2. 경로가 유효하면 계속해서 경로를 따라 이동
	if (CurrentState == EPathfindingState::FollowLinear)
	{
		FollowLinearPath(DeltaTime);
	}
	else if (CurrentState == EPathfindingState::FollowSpline)
	{
		FollowSplinePath(DeltaTime);
	}
}

void UPathfindingComponent::HandleRequestingPathState()
{
    if (PathfindingTaskFuture.IsValid() && PathfindingTaskFuture.IsReady())
    {
        FoundPathIndices = PathfindingTaskFuture.Get();
        FoundPath.Empty();

        if (FoundPathIndices.Num() > 0)
        {
            for (const FIntVector& Index : FoundPathIndices)
            {
                FoundPath.Add(TargetVolume->VoxelIndexToWorld(Index));
            }
            CurrentPathIndex = 0;
        	//TODO: 바꿔야할수도
            CurrentState = EPathfindingState::FollowSpline; // 상태를 '이동 중'으로 변경!

        	CreateSplineFromPath();
        	
            if (bDrawDebugPath)
            {
                for (int32 i = 0; i < FoundPath.Num() - 1; ++i)
                {
                    DrawDebugLine(GetWorld(), FoundPath[i], FoundPath[i + 1], FColor::Green, false, 10.f, 0, 2.f);
                }
            }
        }
        else
        {
            UE_LOG(LogTemp, Warning, TEXT("Async path not found."));
            CurrentState = EPathfindingState::Idle;
        }
        PathfindingTaskFuture.Reset();
    }
}

void UPathfindingComponent::FollowLinearPath(float DeltaTime)
{
	AActor* Actor = GetOwner();
	if (!Actor || CurrentPathIndex < 0 || FoundPath.Num() == 0)
	{
		CurrentState = EPathfindingState::Idle;
		return;
	}

	// 목표 지점에 도착했는지 확인하고 다음 지점으로 넘어가는 로직
	const FVector CurrentLocation = Actor->GetActorLocation();
	const FVector TargetPoint = FoundPath[CurrentPathIndex];
	
	if (FVector::DistSquared(CurrentLocation, TargetPoint) < FMath::Square(100.f))
	{
		CurrentPathIndex++;
		if (CurrentPathIndex >= FoundPath.Num())
		{
			CurrentState = EPathfindingState::Idle; // 이동 완료
			CurrentPathIndex = -1;
			UE_LOG(LogTemp, Log, TEXT("Movement Finished."));
			return;
		}
	}

	// 실제 이동 로직
	if (CurrentPathIndex >= 0 && CurrentPathIndex < FoundPath.Num())
	{
		const FVector Direction = (FoundPath[CurrentPathIndex] - CurrentLocation).GetSafeNormal();
		// 회전도 추가하여 더 자연스럽게 만듭니다.
		const FRotator TargetRotation = Direction.Rotation();
		const FRotator SmoothedRotation = FMath::RInterpTo(Actor->GetActorRotation(), TargetRotation, DeltaTime, 5.0f);
		Actor->SetActorRotation(SmoothedRotation);
		Actor->SetActorLocation(CurrentLocation + Direction * MovementSpeed * DeltaTime); // 원래 방식
	}
}

bool UPathfindingComponent::CheckAndHandlePathInvalidation()
{
    if (!IsPathAheadValid())
    {
        CurrentState = EPathfindingState::PathInvalidated;
        FoundPath.Empty();
        CurrentPathIndex = -1;
        UE_LOG(LogTemp, Warning, TEXT("Path Invalidated! Stopping movement and waiting to repath..."));

        GetWorld()->GetTimerManager().SetTimer(RepathCooldownTimer, this, &UPathfindingComponent::OnRepathTimerFired, RepathCooldown, false);
        return true;
    }
    return false;
}

bool UPathfindingComponent::IsPathAheadValid() const
{
    AFenwickVoxelVolume* FenwickVolume = Cast<AFenwickVoxelVolume>(TargetVolume);
    if (!FenwickVolume || FoundPathIndices.Num() == 0 || CurrentPathIndex < 0)
    {
        return true;
    }

    const int32 AgentRadiusInVoxels = FMath::CeilToInt(Radius / TargetVolume->GetVoxelExtent());
    if (AgentRadiusInVoxels <= 0) return true;

    const int32 LastCheckIndex = FMath::Min(CurrentPathIndex + PathValidationLookAhead, FoundPathIndices.Num() - 1);

    // 현재 위치에서 다음 점프 포인트까지만 검사해도 충분할 수 있습니다.
    if (CurrentPathIndex + 1 <= LastCheckIndex)
    {
        const FIntVector FromIndex = FoundPathIndices[CurrentPathIndex];
        const FIntVector ToIndex = FoundPathIndices[CurrentPathIndex + 1];
        if (!FenwickVolume->IsPathSegmentWalkable(FromIndex, ToIndex, AgentRadiusInVoxels))
        {
            return false;
        }
    }

    return true;
}

void UPathfindingComponent::OnRepathTimerFired()
{
    // 현재 목표 위치로 재탐색 시도
    FindPathToLocation(CurrentTargetLocation);
}