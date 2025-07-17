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

#include "Pathfinder/Strategy/JPSPlusStrategy.h"

UPathfindingComponent::UPathfindingComponent()
{
	PrimaryComponentTick.bCanEverTick = true;

	PathSplinePtr = CreateDefaultSubobject<USplineComponent>(TEXT("Spline"));

	PathfindingStrategyClass = UAStarStrategy::StaticClass();

	DebugSphereComponent = CreateDefaultSubobject<USphereComponent>(TEXT("DebugAgentRadiusSphere"));

	DebugSphereComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision); // 충돌 기능은 완전히 끕니다.
	DebugSphereComponent->SetCanEverAffectNavigation(false); // 네비게이션에 영향을 주지 않도록 합니다.
	DebugSphereComponent->bHiddenInGame = true; // 실제 게임에서는 숨깁니다. (플레이 시에는 Tick의 DrawDebugSphere가 그려줌)
	DebugSphereComponent->SetSphereRadius(AgentRadius); // 초기 반지름을 AgentRadius로 설정합니다.
	
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
        OctreeData.AgentRadius = this->AgentRadius;

        // NavigationData에 옥트리 데이터 스냅샷을 넣음
        Request.NavigationData.Emplace<FOctreePathfindingData>(MoveTemp(OctreeData));
        bRequestDataValid = true;
    }

    // --- 4. 유효한 요청 데이터가 만들어졌으면 서브시스템에 전달 ---
    if (bRequestDataValid)
    {
        CurrentTargetLocation = TargetLocation;
        FoundPath.Empty(); // 경로 초기화

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

	// 변경된 프로퍼티의 이름을 가져옵니다.
	const FName PropertyName = (PropertyChangedEvent.Property != nullptr) ? PropertyChangedEvent.Property->GetFName() : NAME_None;

	// 변경된 프로퍼티가 'AgentRadius'가 맞는지 확인합니다.
	if (PropertyName == GET_MEMBER_NAME_CHECKED(UPathfindingComponent, AgentRadius))
	{
		if (DebugSphereComponent)
		{
			// DebugSphereComponent의 반지름을 새로운 AgentRadius 값으로 업데이트합니다.
			DebugSphereComponent->SetSphereRadius(AgentRadius);
		}
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

	// 1) Volume, AgentVoxel, GoalVoxel 계산
	AUniformGridVolume* Volume = Cast<AUniformGridVolume>(TargetVolume);
	if (!Volume) return;
	const FIntVector CurrentAgentVoxel =
		Volume->WorldToVoxelIndex(GetOwner()->GetActorLocation());
	const FIntVector GoalVoxel =
		Volume->WorldToVoxelIndex(CurrentTargetLocation);

	// 2) FoundPathIndices 는 이미 voxel 인덱스 배열이니 그대로 사용
	const TArray<FIntVector>& VoxPath = FoundPathIndices;
	
	if (PathfindingStrategy->ShouldRepathForOptimization(
			CurrentAgentVoxel,
			VoxPath,
			CurrentPathDestinationIndex,
			GoalVoxel,
			AgentRadius))
	{
		UE_LOG(LogTemp, Log, TEXT("Strategy suggests a shorter path → Requesting repath"));
		RequestPathToLocation(CurrentTargetLocation);
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
	
	if (bShowAgentRadius && GetOwner())
	{
		DrawDebugSphere(
			GetWorld(),
			GetOwner()->GetActorLocation(),
			AgentRadius,
			24,
			FColor::Green,
			false,
			-1.f,
			0,
			1.f 
		);
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
        
        FoundPath = Path; 
        FoundPathIndices.Empty();
		
		if (GetOwner() && FoundPath.Num() > 0)
		{
			FoundPath[0] = GetOwner()->GetActorLocation(); 
		}
		
        CurrentPathDestinationIndex = 1;
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
            DistanceAlongSpline = 0.f;
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

	FoundPath[0] = GetOwner()->GetActorLocation();

	CurrentState = EPathfindingState::FollowPath;
	MovementMode = EPathMovementMode::Spline;
	GetWorld()->GetTimerManager().UnPauseTimer(PathOptimizationTimer);

	if (PathSplinePtr.IsValid())
	{
		CurrentPathDestinationIndex = 0;
		DistanceAlongSpline = 0.f; //Update가 완료되면 Spline 0.f로 하기
		bIsReachDestination = false;

		PathSplinePtr->ClearSplinePoints();

		int32 NumPoints = FoundPath.Num();

		// UE_LOG(LogTemp, Warning, TEXT("Create Spline index"));
		for (int i = 0; i < NumPoints; i++)
		{
			// UE_LOG(LogTemp, Warning, TEXT("index: %d, Location: %.2f %.2f %.2f"), i, FoundPath[i].X, FoundPath[i].Y, FoundPath[i].Z);

			PathSplinePtr->AddSplinePoint(FoundPath[i], ESplineCoordinateSpace::World, false);
		}

		//아래는 spline곡선이 튀지 않게 방지하는 코드
		// const float Tension = 0.2f; // 곡선의 팽팽함. 0.5가 보통 가장 자연스러움. -> UProperty로 빼서 사용자가 지정할 수 있게 변경
		//
		// for (int32 i = 0; i < NumPoints; ++i)
		// {
		// 	FVector Tangent;
		//
		// 	if (i == 0)
		// 	{
		// 		// --- 시작점 (First Point) ---
		// 		// 다음 포인트(1)와 현재 포인트(0)의 차이를 탄젠트로 사용합니다.
		// 		// 이렇게 하면 곡선이 경로의 시작 방향으로 부드럽게 뻗어나갑니다.
		// 		const FVector P0 = PathSplinePtr->GetLocationAtSplinePoint(0, ESplineCoordinateSpace::Local);
		// 		const FVector P1 = PathSplinePtr->GetLocationAtSplinePoint(1, ESplineCoordinateSpace::Local);
		// 		Tangent = (P1 - P0) * Tension;
		// 	}
		// 	else if (i == NumPoints - 1)
		// 	{
		// 		// --- 끝점 (Last Point) ---
		// 		// 현재 포인트(Last)와 이전 포인트(Last-1)의 차이를 탄젠트로 사용합니다.
		// 		// 곡선이 경로의 끝 방향으로 자연스럽게 수렴합니다.
		// 		const FVector PN_1 = PathSplinePtr->GetLocationAtSplinePoint(i - 1, ESplineCoordinateSpace::Local);
		// 		const FVector PN = PathSplinePtr->GetLocationAtSplinePoint(i, ESplineCoordinateSpace::Local);
		// 		Tangent = (PN - PN_1) * Tension;
		// 	}
		// 	else
		// 	{
		// 		// --- 중간점 (Intermediate Points) ---
		// 		// 이것이 Catmull-Rom의 핵심입니다.
		// 		// 이전 포인트(i-1)에서 다음 포인트(i+1)를 향하는 벡터를 계산합니다.
		// 		const FVector P_Prev = PathSplinePtr->GetLocationAtSplinePoint(i - 1, ESplineCoordinateSpace::Local);
		// 		const FVector P_Next = PathSplinePtr->GetLocationAtSplinePoint(i + 1, ESplineCoordinateSpace::Local);
		// 		Tangent = (P_Next - P_Prev) * Tension;
		// 	}
		//
		// 	// 계산된 탄젠트를 스플라인 포인트에 설정합니다.
		// 	PathSplinePtr->SetTangentAtSplinePoint(i, Tangent, ESplineCoordinateSpace::Local, true);
		// }
		
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

	AUniformGridVolume* GridVolume = Cast<AUniformGridVolume>(TargetVolume);
	if (!GridVolume || GridVolume->VoxelExtent == 0) return false;

	const int32 AgentRadiusInVoxels = FMath::CeilToInt((AgentRadius / GridVolume->VoxelExtent) - 0.5f);

	const int32 LastCheckIndex = FMath::Min(CurrentPathDestinationIndex + PathValidationLookAhead,
	                                        FoundPathIndices.Num() - 2);
	for (int32 i = CurrentPathDestinationIndex; i <= LastCheckIndex; ++i)
	{
		if (!OctreeVoxelVolume->IsRegionWalkable(FoundPathIndices[i], AgentRadiusInVoxels))
		{
			UE_LOG(LogTemp, Warning,
			       TEXT("Path Invalidated: A future path segment (from waypoint %d to %d) is blocked!"), i, i + 1);
			return false;
		}
	}

	return true;
}
