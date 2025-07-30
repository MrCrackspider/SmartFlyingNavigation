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
#include "Components/WorldPartitionStreamingSourceComponent.h"
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

	StreamingSourceComponent = CreateDefaultSubobject<UWorldPartitionStreamingSourceComponent>(TEXT("StreamingSourceComponent"));
	StreamingSourceComponent->DisableStreamingSource();
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

	if (StreamingSourceComponent)
	{
		ActivateStreaming();
	}

	if (AVoxelVolumeBase* StartingVolume = GetCurrentVolume())
	{
		GoalUpdateThreshold = StartingVolume->VoxelExtent;
	}
	else
	{
		// 기본값 설정
		// 근데 이거를 VoxelExtent값이랑 동일하게 처리하고 싶음.
		GoalUpdateThreshold = 100.0f;
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
	// Avoidance 사용하면 Avoidance Subsystem에서 아래 함수들 호출해주기 때문에 일단 주석처리.
	CurrentVelocity = PreferredVelocity;
	UpdateAgentRadiusFromScale();

	/***
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (CurrentState == EPathfindingState::FollowPath && !bIsReachDestination)
	{
		if (FoundPath.IsValidIndex(CurrentPathDestinationIndex) && CurrentPathDestinationIndex > 0)
		{
			const AActor* Owner = GetOwner();
			if (!Owner) return;

			const FVector& TargetWaypoint = FoundPath[CurrentPathDestinationIndex];
			const FVector& PrevWaypoint = FoundPath[CurrentPathDestinationIndex - 1];

			const FVector PathSegmentDirection = (TargetWaypoint - PrevWaypoint).GetSafeNormal();

			const FVector VectorFromTargetToAgent = Owner->GetActorLocation() - TargetWaypoint;

			const float DotProduct = FVector::DotProduct(VectorFromTargetToAgent, PathSegmentDirection);

			if (DotProduct >= 0.0f)
			{
				ProgressNextPoint();

				return;
			}
		}
	}

	if (!IsValid(AvoidanceSubsystem) || !AvoidanceSubsystem->GetbUseAvoidance())
	{
		UpdateSteering(DeltaTime);
		ApplyMovement(PreferredVelocity, DeltaTime);
	}

	UpdateAgentRadiusFromScale();
	 */
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

	if (StreamingSourceComponent)
	{
		DeactivateStreaming();
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
	// 펜딩된 요청이 없거나, 등록된 볼륨이 내가 기다리던 볼륨이 아니면 무시
	if (!bHasPendingPathRequest) return;

	UOctreeNavSubsystem* NavSubsystem = GetWorld()->GetSubsystem<UOctreeNavSubsystem>();
	if (!NavSubsystem) return;

	AOctreeVoxelVolume* CurrentVolume = NavSubsystem->GetVolumeChunkAt(GetOwner()->GetActorLocation());
	if (RegisteredVolume != CurrentVolume) return;

	UE_LOG(LogTemp, Log, TEXT("Start volume '%s' is now ready. Executing pending path request."),
	       *RegisteredVolume->GetName());

	bHasPendingPathRequest = false;

	// 델리게이트는 이제 필요 없으므로 해제
	NavSubsystem->OnVolumeRegistered.RemoveDynamic(this, &UPathfindingComponent::OnVolumeBecameReadyForPendingRequest);

	// 저장해둔 목표 지점으로 길찾기를 다시 요청
	RequestPathToLocation(PendingTargetLocation);
}

bool UPathfindingComponent::RequestPathToLocation(const FVector& TargetLocation)
{
	UOctreeNavSubsystem* NavSubsystem = GetWorld()->GetSubsystem<UOctreeNavSubsystem>();
	if (!NavSubsystem || !PathfindingSubsystem) return false;

	const FVector StartLocation = GetOwner()->GetActorLocation();
	AOctreeVoxelVolume* StartVolume = NavSubsystem->GetVolumeChunkAt(StartLocation);

	// --- 1. 시작 볼륨 유효성 검사 ---
	if (!StartVolume)
	{
		UE_LOG(LogTemp, Warning, TEXT("Pathfinding request failed: Agent is not inside any navigation volume."));
		return false;
	}

	// --- 2. 시작 볼륨이 아직 준비되지 않은 경우, 요청을 보류합니다. ---
	if (!StartVolume->IsReadyForNavigation())
	{
		UE_LOG(LogTemp, Log, TEXT("Start volume '%s' is not ready. Path request is pending."), *StartVolume->GetName());
		bHasPendingPathRequest = true;
		PendingTargetLocation = TargetLocation;
		NavSubsystem->OnVolumeRegistered.RemoveDynamic(this, &UPathfindingComponent::OnVolumeBecameReadyForPendingRequest);
		NavSubsystem->OnVolumeRegistered.AddDynamic(this, &UPathfindingComponent::OnVolumeBecameReadyForPendingRequest);
		return true; // 요청 접수됨
	}

	// --- 3. 로컬/계층 길찾기 분기 ---
	bHasPendingPathRequest = false; // 보류 중인 요청이 있었다면 해제
	CurrentTargetLocation = TargetLocation;

	AOctreeVoxelVolume* EndVolume = NavSubsystem->GetVolumeChunkAt(TargetLocation);

	// --- CASE A: 명백한 로컬 길찾기 ---
	// 시작 볼륨과 끝 볼륨이 동일하고, 유효하며, 준비된 상태입니다.
	if (EndVolume && StartVolume == EndVolume)
	{
		// UE_LOG(LogTemp, Log, TEXT("Requesting LOCAL path within volume '%s'."), *StartVolume->GetName());
		// 단일 볼륨 내 길찾기는 더 간단한 FPathfindingRequest를 직접 사용합니다.
		// 이 경우 HandleWorldPathResult가 아닌 다른 콜백을 사용하거나, HandleWorldPathResult를 수정해야 합니다.
		// 여기서는 기존 구조를 최대한 활용하기 위해 RequestLocalPath_Internal을 사용합니다.
		return RequestLocalPath_Internal(StartLocation, TargetLocation, StartVolume);
	}
	// --- CASE B: 계층적 길찾기 ---
	// 끝 볼륨이 다르거나, 스트리밍되지 않아 null인 경우입니다.
	else
	{
		UE_LOG(LogTemp, Log, TEXT("Requesting HIERARCHICAL path from '%s' to an external or unloaded volume."), *StartVolume->GetName());

		FHierarchicalPathRequest HRequest;
		HRequest.RequestingComponent = this;
		HRequest.StartLocation = StartLocation;
		HRequest.EndLocation = TargetLocation;
		HRequest.AgentRadius = this->AgentRadius;
		HRequest.Strategy = PathfindingStrategy;
		HRequest.OnHierarchicalPathComplete.BindDynamic(this, &UPathfindingComponent::HandleWorldPathResult);

		CurrentRequestID = PathfindingSubsystem->RequestHierarchicalPath(this, HRequest);
		return CurrentRequestID != -1;
	}
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

	// 볼륨에서 데이터 스냅샷 생성
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

void UPathfindingComponent::ActivateStreaming()
{
	// 지금은 BeginPlay에서 호출하고 있지만 길찾기를 하는 순간 호출해도 괜찮음.
	// 그런데 그러면 Load되는 시간이 있기 때문에 문제가 될 수도 있는 점을 감안해야 함.
	if (StreamingSourceComponent)
	{
		StreamingSourceComponent->EnableStreamingSource();
	}
}

void UPathfindingComponent::DeactivateStreaming()
{
	// 길찾기 종료 시에 호출
	if (StreamingSourceComponent)
	{
		StreamingSourceComponent->DisableStreamingSource();
	}
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
	
	// 1. 단순하게 이동: 예측이나 계산 없이, 주어진 속도로 그냥 움직입니다.
	const FVector MoveDelta = FinalVelocity * DeltaTime;
	FHitResult HitResult;
	Owner->AddActorWorldOffset(MoveDelta, true, &HitResult);

	AdjustProgressToSpline(MoveDelta);
	
	Owner->SetActorRotation(MoveDelta.ToOrientationQuat());
	
	// 2. 물리적 충돌 처리: 실제로 부딪혔을 때만 비상 상황을 선포합니다.
	if (HitResult.bBlockingHit)
	{
		UE_LOG(LogTemp, Warning, TEXT("Movement physically blocked by %s. Invalidating path."), *HitResult.GetActor()->GetName());
		InvalidatePathAndRequestRepath(); // 이 함수는 상태를 명확히 바꾸는 최종 버전이어야 합니다.
		return;
	}

	// 3. [핵심] 도착 판단: 이동이 끝난 '현재' 위치를 기준으로, 목표에 충분히 가까워졌는지 확인합니다.
	//    루프를 사용하여, 한 프레임에 여러 웨이포인트를 통과하는 경우도 완벽하게 처리합니다.
	bool bAdvanced;
	float AcceptanceRadius = 100.0f;
	do
	{
		bAdvanced = false;
		if (FoundPath.IsValidIndex(CurrentPathDestinationIndex))
		{
			const float DistToTargetSq = FVector::DistSquared(Owner->GetActorLocation(), FoundPath[CurrentPathDestinationIndex]);

			// 현재 위치가 목표 지점의 허용 반경 안에 있는가?
			if (DistToTargetSq < FMath::Square(AcceptanceRadius))
			{
				// 도착했으므로, 다음 목표 지점으로 넘어갑니다.
				ProgressNextPoint();
				bAdvanced = true; // 경로가 바뀌었으니, 새 목표에 대해서도 바로 도착했는지 확인하기 위해 루프를 한 번 더 돕니다. 
			}
		}
	}
	while (bAdvanced && !bIsReachDestination); // 경로의 끝에 도달하면 루프를 멈춥니다.
}

void UPathfindingComponent::AdjustProgressToSpline(const FVector& FinalVelocity)
{
	if (!IsValid(PathSplinePtr) || PathSplinePtr->GetNumberOfSplinePoints() < 2)
	{
		return;
	}
		
	DistanceAlongSpline += FinalVelocity.Length();
	
	// const FVector SplineDirection = PathSplinePtr->GetDirectionAtDistanceAlongSpline(
	// 		DistanceAlongSpline, ESplineCoordinateSpace::World);
	// const float ForwardDistance = FVector::DotProduct(FinalVelocity, SplineDirection);
	// DistanceAlongSpline += ForwardDistance;
	// const float SplineLength = PathSplinePtr->GetSplineLength();
	// DistanceAlongSpline = FMath::Clamp(DistanceAlongSpline, 0.f, SplineLength);
}

void UPathfindingComponent::ProgressNextPoint()
{
	// Linear로 가도 partialSplineUpdate는 해야해서 if문으로 감싸지 마세요
	PartialSplinePointUpdate(CurrentPathDestinationIndex);
	
	if (CurrentPathDestinationIndex + 1 >= FoundPath.Num())
	{
		const FVector LastPathPoint = FoundPath.Last();
		const float DistanceToFinalGoalSq = FVector::DistSquared(LastPathPoint, CurrentTargetLocation);

		if (DistanceToFinalGoalSq <= FMath::Square(GoalUpdateThreshold))
		{
			UE_LOG(LogTemp, Log, TEXT("Reached the final destination. Stopping movement."));
			OnDestinationReached.Broadcast();
			StopMovement();
			return;
		}
		else
		{
			UE_LOG(LogTemp, Warning, TEXT("Reached end of a partial path. Requesting next path segment immediately."));

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

/**
 * 
void UPathfindingComponent::HandleWorldPathResult(const TArray<FVector>& Path, bool bSuccess)
{
	if (!IsValid(PathSplinePtr))
	{
		UE_LOG(LogTemp, Warning, TEXT("PathSplinePtr is not valid. Cannot set up spline path."));
		return;
	}

	if (!bSuccess || Path.IsEmpty())
	{
		UE_LOG(LogTemp, Warning, TEXT("Pathfinding failed or path is empty."));
		if (bPeriodicRepath)
		{
			GetWorld()->GetTimerManager().SetTimer(RepathCooldownTimer, this, &UPathfindingComponent::PeriodicRepath, RepathCooldown, false);
		}
		return;
	}
	
	if (bSuccess && Path.Num() > 0)
	{
		// if (IsPathObstructed(GetOwner()->GetActorLocation(), Path[0]))
		// {
		// 	UE_LOG(LogTemp, Warning, TEXT("New path is obstructed from actor's current location. Requesting a new path immediately."));
		// 	RequestPathToLocation(CurrentTargetLocation);
		// 	return;
		// }
		
		// *** Case: Path is safe ***
		FoundPath = Path;
		if (FVector::DistSquared(GetOwner()->GetActorLocation(), FoundPath[0]) < FMath::Square(10.0f))
		{
			CurrentPathDestinationIndex = 0;
		}
		else
		{
			FoundPath.Insert(GetOwner()->GetActorLocation(), 0);
			CurrentPathDestinationIndex = 1;
		}

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
 */
void UPathfindingComponent::HandleWorldPathResult(const TArray<FVector>& Path, bool bSuccess)
{
	if (!IsValid(PathSplinePtr))
	{
		UE_LOG(LogTemp, Warning, TEXT("PathSplinePtr is not valid. Cannot set up spline path."));
		return;
	}

	if (!bSuccess || Path.IsEmpty())
	{
		UE_LOG(LogTemp, Warning, TEXT("Pathfinding failed or path is empty. Retrying if periodic repath is enabled."));
		if (bPeriodicRepath)
		{
			GetWorld()->GetTimerManager().SetTimer(RepathCooldownTimer, this, &UPathfindingComponent::PeriodicRepath, RepathCooldown, false);
		}
		return;
	}

	// --- 여기가 Repath 문제를 해결하는 핵심 로직입니다 ---

	const FVector CurrentLocation = GetOwner()->GetActorLocation();
	TArray<FVector> NewPath;


	int32 FinalStartIndex = -1;

	for (int32 i = 1; i < Path.Num(); ++i)
	{
		if (!IsPathObstructed(CurrentLocation, Path[i]))
		{
			FinalStartIndex = i;
			break;
		}
	}

	NewPath.Add(CurrentLocation);

	if (FinalStartIndex != -1)
	{
		for (int32 i = FinalStartIndex; i < Path.Num(); ++i)
		{
			NewPath.Add(Path[i]);
		}
	}
	else
	{
		NewPath.Append(Path);
	}

	FoundPath = NewPath;

	// 상태를 리셋합니다.
	bIsReachDestination = false;
	CurrentState = EPathfindingState::FollowPath;
	CurrentPathDestinationIndex = 1;


	// 스플라인 및 디버그 드로잉 로직
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

void UPathfindingComponent::HandlePathfindingResult()
{
	if (bPeriodicRepath && !bIsReachDestination)
	{
		if (UWorld* World = GetWorld())
		{
			World->GetTimerManager().SetTimer(RepathCooldownTimer, this, &UPathfindingComponent::PeriodicRepath,
			                                  RepathCooldown, false);
		}
	}
}

void UPathfindingComponent::InvalidatePathAndRequestRepath()
{
	if (CurrentState == EPathfindingState::PathInvalidated) return;

	UE_LOG(LogTemp, Warning, TEXT("Path Invalidated! Stopping movement and scheduling a repath..."));

	if (UWorld* World = GetWorld())
	{
		World->GetTimerManager().ClearTimer(RepathCooldownTimer);
	}

	RequestPathToLocation(CurrentTargetLocation);
	// End Test
}

// 
bool UPathfindingComponent::IsPathObstructed(const FVector& Start, const FVector& End, bool bIsPortalApproach) const
{
	AOctreeVoxelVolume* OctreeVolume = Cast<AOctreeVoxelVolume>(GetCurrentVolume());
	if (!OctreeVolume)
	{
		return true;
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
			// 일반적인 경로: 기존과 같이 에이전트 반경을 고려하여 오프셋 테스트를 수행합니다.
			const FVector TestOffsets[] = {
				FVector::ZeroVector,
				FVector(0, AgentRadius, 0),  // 좌
				FVector(0, -AgentRadius, 0), // 우
				FVector(0, 0, AgentRadius),  // 위
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
	/**
	* 
	AOctreeVoxelVolume* OctreeVolume = Cast<AOctreeVoxelVolume>(GetCurrentVolume());
	if (!OctreeVolume)
	{
		return true;
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

		const FVector TestOffsets[] = {
			FVector::ZeroVector,
			FVector(0, AgentRadius, 0), // 좌
			FVector(0, -AgentRadius, 0), // 우
			FVector(0, 0, AgentRadius), // 위
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

	if (OctreeVolume->IsLocationInBlockedVoxel(End))
	{
		return true;
	}

	return false;
	 */
}

void UPathfindingComponent::DrawDebugPath()
{
	UWorld* World = GetWorld();
	if (!World || FoundPath.Num() < 2)
	{
		return;
	}

	const float Duration = 1.0f;
	const float LineThickness = 5.0f;
	const float BoxThickness = AgentRadius * 0.7;
	const FVector BoxHalfSize(AgentRadius * 0.05);

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
	AOctreeVoxelVolume* VoxelVolume = Cast<AOctreeVoxelVolume>(GetCurrentVolume());
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

	const float CurrentDistance = PathSplinePtr->GetDistanceAlongSplineAtSplinePoint(CurrentPathDestinationIndex - 1);
	const float NextDistance = PathSplinePtr->GetDistanceAlongSplineAtSplinePoint(CurrentPathDestinationIndex);

	for (float Distance = CurrentDistance; Distance <= NextDistance; Distance += SafetyCheckStepSize)
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
