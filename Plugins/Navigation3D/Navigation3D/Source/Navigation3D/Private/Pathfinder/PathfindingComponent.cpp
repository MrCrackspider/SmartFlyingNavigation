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
	// 에디터 뷰포트에서도 TickComponent()가 호출되도록
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

bool UPathfindingComponent::RequestPathToLocation(const FVector& TargetLocation)
{
    // --- 1. 사전 유효성 검사 (기존과 동일) ---
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

    UWorld* World = GetWorld();
    if (World)
    {
        World->GetTimerManager().ClearTimer(RepathCooldownTimer);
    }

    // --- 2. "요청서" 작성 시작 ---
    FPathfindingRequest Request;
    Request.RequestingObject = this;
    Request.StartLocation = GetOwner()->GetActorLocation();
    Request.EndLocation = TargetLocation;
    Request.AgentRadius = this->AgentRadius; // 월드 유닛 에이전트 반경
	Request.Strategy = PathfindingStrategy;
    bool bRequestDataValid = false;
	
    if (AOctreeVoxelVolume* OctreeVolume = Cast<AOctreeVoxelVolume>(TargetVolume))
    {
        // ----- 옥트리 볼륨일 경우 -----
        UE_LOG(LogTemp, Log, TEXT("Requesting path on an Octree Volume."));

        // 요청서에 옥트리용이라고 명시
        Request.NodeType = EPathfindingNodeType::Octree;
    	Request.OnWorldPathComplete.BindDynamic(this, &UPathfindingComponent::HandleWorldPathResult);
 
        // FOctreePathfindingData 스냅샷 생성
        FOctreePathfindingData OctreeData;
        OctreeData.StaticNodePool = OctreeVolume->GetStaticNodePool();
        OctreeData.RootNodeIndex = OctreeVolume->GetRootNodeIndex();
        OctreeData.MaxOctreeDepth = OctreeVolume->GetMaxOctreeDepth();
        OctreeData.VolumeBounds = OctreeVolume->VolumeBox->Bounds.GetBox();
        OctreeData.DynamicObstacleBitmask = OctreeVolume->GetDynamicObstacleBitmask();
        OctreeData.MapSize = OctreeVolume->MapSize;
        OctreeData.VoxelSize = OctreeVolume->VoxelExtent;
       
        // NavigationData에 옥트리 데이터 스냅샷을 넣음
        Request.NavigationData.Emplace<FOctreePathfindingData>(MoveTemp(OctreeData));
        bRequestDataValid = true;
    }

    // --- 4. 유효한 요청 데이터가 만들어졌으면 서브시스템에 전달 ---
    if (bRequestDataValid)
    {
        CurrentTargetLocation = TargetLocation;

    	CurrentRequestID = PathSubsystem->RequestPath(Request);
    	if (CurrentRequestID != -1)
    	{
    		// 상태 변경 등...
    		return true;
    	}
    }
    else
    {
        UE_LOG(LogTemp, Error, TEXT("RequestPathToLocation failed: TargetVolume is not a supported type (Octree)."));
    }

    return false;
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
		UpdateAgentRadiusFromScale(true); // 이 안에서 GetActorScale3D() 사용!
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

	// 1. 실제 이동 (기존과 동일)
	const FVector CurrentLocation = Owner->GetActorLocation();
	CurrentVelocity = FinalVelocity;
	const FVector AdjustedVelocity = FinalVelocity * DeltaTime;
	Owner->SetActorLocation(CurrentLocation + AdjustedVelocity);

	// --- 회전 로직 수정 ---
	if (!FinalVelocity.IsNearlyZero())
	{
		// 현재 회전값 가져오기
		const FRotator CurrentRotation = Owner->GetActorRotation();
        
		// 목표 회전값 계산 (이동 방향을 바라보도록)
		FRotator TargetRotation = FinalVelocity.GetSafeNormal().ToOrientationRotator();

		// [중요] 축 잠금(Axis Lock)을 목표 회전값에 먼저 적용
		// 잠긴 축은 현재 회전값을 그대로 유지하도록 하여, 해당 축으로는 회전 보간이 일어나지 않게 함
		TargetRotation.Roll = RotationAxisLock[0] ? CurrentRotation.Roll : TargetRotation.Roll;
		TargetRotation.Pitch = RotationAxisLock[1] ? CurrentRotation.Pitch : TargetRotation.Pitch;
		TargetRotation.Yaw = RotationAxisLock[2] ? CurrentRotation.Yaw : TargetRotation.Yaw;

		// [핵심] RInterpTo를 사용하여 현재 회전에서 목표 회전으로 부드럽게 보간
		// 이 함수는 두 회전값 사이의 최단 경로를 자동으로 계산해 줍니다.
		FRotator NewRotation = FMath::RInterpTo(CurrentRotation, TargetRotation, DeltaTime, RotationInterpSpeed);

		// 최종적으로 계산된 부드러운 회전값 적용
		Owner->SetActorRotation(NewRotation);
	}
	// --- 회전 로직 수정 끝 ---

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

		if (MovementMode == EPathMovementMode::Spline/* && CurrentPathIndex != 0*/)
		{
			const float TargetPointDistance = PathSplinePtr->GetDistanceAlongSplineAtSplinePoint(
				CurrentPathDestinationIndex);
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
		OnDestinationReached.Broadcast();
		bIsReachDestination = true;
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
	const FVector SplineLocation = PathSplinePtr->GetLocationAtDistanceAlongSpline(
		DistanceAlongSpline, ESplineCoordinateSpace::World);

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
	       CurrentPathDestinationIndex-1, FoundPath[CurrentPathDestinationIndex-1].X,
	       FoundPath[CurrentPathDestinationIndex-1].Y, FoundPath[CurrentPathDestinationIndex-1].Z,
	       CurrentPathDestinationIndex, FoundPath[CurrentPathDestinationIndex].X,
	       FoundPath[CurrentPathDestinationIndex].Y, FoundPath[CurrentPathDestinationIndex].Z);

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
		return;
	
}
//======================================================================================
// Engine Overrides
//======================================================================================

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
	if (TargetVolume)
	{
		TargetVolume->RegisterStreamingAgent(GetOwner());
	}
}
void UPathfindingComponent::OnRegister()
{
	Super::OnRegister();
	
	if (DebugSphereComponent && GetOwner() && GetOwner()->GetRootComponent())
	{
		DebugSphereComponent->AttachToComponent(GetOwner()->GetRootComponent(), FAttachmentTransformRules::SnapToTargetNotIncludingScale);
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

void UPathfindingComponent::TickComponent(float DeltaTime, ELevelTick TickType,
                                          FActorComponentTickFunction* ThisTickFunction)
{
	
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	UpdateAgentRadiusFromScale(); // 인게임에서 Scale 변화에 따른 길찾기를 위해 호출.
	
	if (CurrentState == EPathfindingState::FollowPath)
	{
		if (!IsPathAheadValid())
		{
			UE_LOG(LogTemp, Warning, TEXT("Dynamic obstacle is on path. Start Repath"));
			InvalidatePathAndRequestRepath();
			return;
		}

		CheckForPathOptimization();
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

void UPathfindingComponent::HandleVoxelPathResult(const TArray<FIntVector>& PathIndices, bool bSuccess)
{
	if (bSuccess && PathIndices.Num() > 0)
	{
		ProcessNewPath(PathIndices); // 이 함수는 경로를 스플라인 등으로 변환
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
			World->GetTimerManager().SetTimer(RepathCooldownTimer, this, &UPathfindingComponent::PeriodicRepath, RepathCooldown, false);
		}
	}
}

void UPathfindingComponent::HandleWorldPathResult(const TArray<FVector>& Path, bool bSuccess)
{
	if (bSuccess && Path.Num() > 1)
	{
		FoundPath.Empty();
		FoundPath = Path;
		FoundPathIndices.Empty();

		bIsReachDestination = false;
		CurrentState = EPathfindingState::FollowPath;

		if (PathSplinePtr.IsValid())
		{
			PathSplinePtr->ClearSplinePoints();
			for (const FVector& Point : FoundPath)
			{
				PathSplinePtr->AddSplinePoint(Point, ESplineCoordinateSpace::World, false);
			}
			PathSplinePtr->UpdateSpline();

			const FVector CurrentLocation = GetOwner()->GetActorLocation();
			const float ClosestInputKey = PathSplinePtr->FindInputKeyClosestToWorldLocation(CurrentLocation);
			DistanceAlongSpline = PathSplinePtr->GetDistanceAlongSplineAtSplineInputKey(ClosestInputKey);

			int32 ClosestPointIndex = FMath::RoundToInt(ClosestInputKey);
			CurrentPathDestinationIndex = FMath::Clamp(ClosestPointIndex + 1, 1, FoundPath.Num() - 1);
		}
		else
		{
			UE_LOG(LogTemp, Warning, TEXT("PathSplinePtr is not valid. Cannot set up spline path."));
		}

		GetWorld()->GetTimerManager().UnPauseTimer(PathOptimizationTimer);
	}
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("Pathfinding failed or path is too short."));
        FoundPath.Empty();
        FoundPathIndices.Empty();
        CurrentPathDestinationIndex = 0;
        CurrentState = EPathfindingState::Idle;
        GetWorld()->GetTimerManager().PauseTimer(PathOptimizationTimer);
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
		ProcessNewPath(PathIndices);
	}
	if (bPeriodicRepath)
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

void UPathfindingComponent::ProcessNewPath(const TArray<FIntVector>& PathIndices)
{
	
}

void UPathfindingComponent::InvalidatePathAndRequestRepath()
{
	if (CurrentState == EPathfindingState::PathInvalidated) return;

	UE_LOG(LogTemp, Warning, TEXT("Path Invalidated! Stopping movement and scheduling a repath..."));

	// Begin Test
	//FoundPathIndices.Empty();
	//GetWorld()->GetTimerManager().PauseTimer(PathOptimizationTimer);

	//if (UWorld* World = GetWorld())
	//{
	//	OnRepathTimerFired();
	//}


	RequestPathToLocation(CurrentTargetLocation);

	// End Test
}

void UPathfindingComponent::OnRepathTimerFired()
{
	RequestPathToLocation(CurrentTargetLocation);
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
