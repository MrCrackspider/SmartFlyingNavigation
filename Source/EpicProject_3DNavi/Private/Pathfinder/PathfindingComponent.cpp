#include "Pathfinder/PathfindingComponent.h"
#include "Pathfinder/PathfindingSubsystem.h"
#include "Pathfinder/AvoidanceSubsystem.h" // 새로운 회피 서브시스템 헤더
#include "Pathfinder/FenwickVoxelVolume.h" // IsPathAheadValid에서 필요
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

    // Tick에서는 더 이상 직접 움직임을 처리하지 않습니다.
    // 대신, 상태 관리 및 경로 유효성 검사 등 다른 중요한 작업을 수행합니다.
    switch (CurrentState)
    {
    case EPathfindingState::Moving:
        // 이동 중일 때, 경로가 막히지 않았는지 계속 확인합니다.
        CheckAndHandlePathInvalidation();
        break;

    case EPathfindingState::RequestingPath:
        HandleRequestingPathState();
        break;

    case EPathfindingState::PathInvalidated:
    case EPathfindingState::Idle:
    default:
        // 이 상태들에서는 아무것도 하지 않습니다.
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

FVector UPathfindingComponent::GetPreferredVelocity() const
{
    AActor* Owner = GetOwner();
    // 이동 중이 아니거나, 경로가 없거나, 주인이 없으면 멈춥니다.
    if (!Owner || CurrentState != EPathfindingState::Moving || CurrentPathIndex < 0 || CurrentPathIndex >= FoundPath.Num())
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
    if (CurrentState == EPathfindingState::Moving && CurrentPathIndex >= 0 && CurrentPathIndex < FoundPath.Num())
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
            CurrentState = EPathfindingState::Moving; // 상태를 '이동 중'으로 변경!

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