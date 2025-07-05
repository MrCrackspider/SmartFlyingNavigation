#include "Pathfinder/AvoidanceSubsystem.h"
#include "Pathfinder/PathfindingComponent.h"
#include "GameFramework/Actor.h"
#include "Stats/Stats.h"
#include "DrawDebugHelpers.h"
#include "Algo/Sort.h"
#include "Pathfinder/Utility/Parallel/ParallelAvoidanceTask.h"

DECLARE_CYCLE_STAT(TEXT("AvoidanceSubsystem Tick"), STAT_AvoidanceSubsystem_Tick, STATGROUP_Game);

void UAvoidanceSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
    Super::Initialize(Collection);
    bIsTaskRunning = false;
    AsyncTask = nullptr;

    TimeHorizon = 1.f;
    MaxNeighbors = 4;
    GridCellSize = 200.f;
    SymmetryBreakingRangeMultiplier = 4.f;
    SolverIterations = 5;
    SideBiasMagnitude = 0.8f;
    InterpSpeed = 10.f;
}

void UAvoidanceSubsystem::PostInitialize()
{
    Super::PostInitialize();
    /**
    * 여기서 GridCellSize를 VoxelExtent의 두 배로 설정할 예정
    * 그래서 UPathfindingSubsystem에서 Volume을 다루고, 해당 Extent를 가져올 수 있게 수정해야함.
    */
}

void UAvoidanceSubsystem::Deinitialize()
{
    if (AsyncTask)
    {
        AsyncTask->EnsureCompletion();
        delete AsyncTask;
        AsyncTask = nullptr;
    }
    Super::Deinitialize();
}

void UAvoidanceSubsystem::RegisterAgent(UPathfindingComponent* AgentComponent)
{
    if (AgentComponent)
    {
        RegisteredAgentComponents.AddUnique(AgentComponent);
        AgentMap.Add(AgentComponent->GetUniqueID(), AgentComponent);
    }
}

void UAvoidanceSubsystem::UnregisterAgent(UPathfindingComponent* AgentComponent)
{
    if (AgentComponent)
    {
        RegisteredAgentComponents.Remove(AgentComponent);
        AgentMap.Remove(AgentComponent->GetUniqueID());
    }
}

TStatId UAvoidanceSubsystem::GetStatId() const
{
    RETURN_QUICK_DECLARE_CYCLE_STAT(UAvoidanceSubsystem, STATGROUP_Tickables);
}

void UAvoidanceSubsystem::Tick(float DeltaTime) 
{

    Super::Tick(DeltaTime);
    SCOPE_CYCLE_COUNTER(STAT_AvoidanceSubsystem_Tick);

    if (bIsTaskRunning && AsyncTask && AsyncTask->IsWorkDone())
    {
        TMap<uint32, FAvoidanceResult> LocalResults;
        {
            FScopeLock Lock(&ResultsCriticalSection);
            LocalResults = MoveTemp(AvoidanceResults);
        }

        for (const auto& ResultPair : LocalResults)
        {
            if (UPathfindingComponent* Agent = AgentMap.FindRef(ResultPair.Key))
            {
                // 워커스레드에서 Avoidance가 적용된 속도를 가져옴
                const FVector OptimalVelocity_AgentOnly = ResultPair.Value.OptimalVelocity;

				// 게임스레드에서 StaticObstacleConstraints를 생성
                TArray<FPlane> VoxelConstraints;
                GenerateVoxelConstraints(Agent, VoxelConstraints);

                // 정적 장애물 제약 조건을 적용하여 최종 속도를 계산
                // 이때 선호 속도는 에이전트 간 회피가 적용된 속도
                const FVector FinalOptimalVelocity = SolveOptimalVelocity(VoxelConstraints, OptimalVelocity_AgentOnly);

                // 최종 속도를 보간
                const FVector CurrentVelocity = Agent->GetCurrentVelocity();
                const FVector InterpolatedVelocity = FMath::VInterpTo(CurrentVelocity, FinalOptimalVelocity, DeltaTime, InterpSpeed);
                const FVector FinalVelocity = InterpolatedVelocity.GetClampedToMaxSize(Agent->GetMovementSpeed());

                Agent->ApplyMovement(FinalVelocity, DeltaTime);

                DebugVisualizeVelocities(Agent, ResultPair.Value, FinalOptimalVelocity);
            }
        }

        AsyncTask->EnsureCompletion();
        delete AsyncTask;
        AsyncTask = nullptr;
        bIsTaskRunning = false;
    }

    if (!bIsTaskRunning)
    {
        if (RegisteredAgentComponents.Num() < 2)
        {
            if (RegisteredAgentComponents.Num() == 1 && RegisteredAgentComponents[0])
            {
                RegisteredAgentComponents[0]->ApplyMovement(RegisteredAgentComponents[0]->GetPreferredVelocity(), DeltaTime);
            }
            return;
        }

        TArray<FAgentThreadData> AgentDataForThread;
        AgentDataForThread.Reserve(RegisteredAgentComponents.Num());
        for (UPathfindingComponent* AgentComponent : RegisteredAgentComponents)
        {
            if (AgentComponent && AgentComponent->GetOwner())
            {
                AgentDataForThread.Emplace(FAgentThreadData{
                    AgentComponent->GetUniqueID(),
                    AgentComponent->GetOwner()->GetActorLocation(),
                    AgentComponent->GetCurrentVelocity(),
                    AgentComponent->GetPreferredVelocity(),
                    AgentComponent->GetAgentRadius(),
                    AgentComponent->GetMovementSpeed(),
                    AgentComponent->GetUniqueID()
                    });
            }
        }

        {
            FScopeLock Lock(&ResultsCriticalSection);
            AvoidanceResults.Empty();
        }

        AsyncTask = new FAsyncTask<FParallelAvoidanceTask>(MoveTemp(AgentDataForThread), AvoidanceResults, ResultsCriticalSection, TimeHorizon, MaxNeighbors, GridCellSize, SymmetryBreakingRangeMultiplier, SolverIterations, SideBiasMagnitude);
        AsyncTask->StartBackgroundTask();
        bIsTaskRunning = true;
    }
}
static void GenerateAgentConstraints_ThreadSafe(const FAgentThreadData& CurrentAgent, const TArray<const FAgentThreadData*>& NearbyAgents, TArray<FPlane>& OutConstraints, float TimeHorizon)
{
    for (const FAgentThreadData* AgentB : NearbyAgents)
    {
        const FVector RelativePosition = AgentB->Position - CurrentAgent.Position;
        const FVector RelativeVelocity = CurrentAgent.CurrentVelocity - AgentB->CurrentVelocity;
        const float DistSq = RelativePosition.SizeSquared();
        const float CombinedRadius = CurrentAgent.AgentRadius + AgentB->AgentRadius;
        const float CombinedRadiusSq = FMath::Square(CombinedRadius);

        float EffectiveTimeHorizon = TimeHorizon;
        bool bIsCollidingOrApproaching = false;

        const float RelativeVelSq = RelativeVelocity.SizeSquared();
        if (RelativeVelSq > KINDA_SMALL_NUMBER)
        {
            const float DotProduct_RelPos_RelVel = FVector::DotProduct(RelativePosition, RelativeVelocity);
            const float Discriminant = FMath::Square(DotProduct_RelPos_RelVel) - RelativeVelSq * (DistSq - CombinedRadiusSq);

            if (Discriminant >= 0.f && DotProduct_RelPos_RelVel < 0.f)
            {
                const float TTC = (-DotProduct_RelPos_RelVel - FMath::Sqrt(Discriminant)) / RelativeVelSq;
                if (TTC > 0.f)
                {
                    EffectiveTimeHorizon = FMath::Min(TimeHorizon, TTC);
                    bIsCollidingOrApproaching = true;
                }
            }
        }

        if (DistSq > CombinedRadiusSq && !bIsCollidingOrApproaching) continue;

        const float InverseEffectiveTimeHorizon = 1.0f / EffectiveTimeHorizon;
        FVector u = FVector::ZeroVector;

        if (DistSq > CombinedRadiusSq)
        {
            const FVector w = RelativeVelocity - RelativePosition * InverseEffectiveTimeHorizon;
            const float w_sq = w.SizeSquared();
            const float VOptRadius = CombinedRadius * InverseEffectiveTimeHorizon;
            const float VOptRadiusSq = FMath::Square(VOptRadius);

            if (FVector::DotProduct(w, RelativePosition) < 0.0f && FMath::Square(FVector::DotProduct(w, RelativePosition)) > w_sq * VOptRadiusSq)
            {
                const float w_len = FMath::Sqrt(w_sq);
                const FVector n_w = w / w_len;
                u = (n_w * VOptRadius - w);
            }
        }

        if (!u.IsNearlyZero(KINDA_SMALL_NUMBER))
        {
            const FVector w_correction = u * 0.5f;
            const FVector n_correction = u.GetSafeNormal();
            if (!n_correction.IsNearlyZero(KINDA_SMALL_NUMBER))
            {
                OutConstraints.Emplace(CurrentAgent.CurrentVelocity + w_correction, n_correction);
            }
        }
    }
}

static FVector CalculateSymmetryBreakingVelocity_ThreadSafe(const FAgentThreadData& CurrentAgent, const FVector& PreferredVelocity, const TArray<const FAgentThreadData*>& NearbyAgents, float SymmetryBreakingRangeMultiplier, float SideBiasMagnitude)
{
    if (NearbyAgents.IsEmpty() || PreferredVelocity.IsNearlyZero())
    {
        return PreferredVelocity;
    }

    const FAgentThreadData* ObstacleAgent = nullptr;
    float MinThreatDistSq = TNumericLimits<float>::Max();

    for (const FAgentThreadData* AgentB : NearbyAgents)
    {
        const FVector ToAgentB = AgentB->Position - CurrentAgent.Position;
        const float DistToAgentBSq = ToAgentB.SizeSquared();
        const float CombinedRadius = CurrentAgent.AgentRadius + AgentB->AgentRadius;

        if (DistToAgentBSq < FMath::Square(CombinedRadius * SymmetryBreakingRangeMultiplier) && FVector::DotProduct(PreferredVelocity, ToAgentB) > 0)
        {
            if (DistToAgentBSq < MinThreatDistSq)
            {
                MinThreatDistSq = DistToAgentBSq;
                ObstacleAgent = AgentB;
            }
        }
    }

    if (ObstacleAgent && CurrentAgent.UniqueID > ObstacleAgent->UniqueID)
    {
        FVector ForwardVec = PreferredVelocity.GetSafeNormal();
        FVector SideVector = FVector::CrossProduct(ForwardVec, FVector::UpVector);
        if (SideVector.IsNearlyZero())
        {
            SideVector = FVector::CrossProduct(ForwardVec, FVector::RightVector);
        }
        SideVector.Normalize();

        FVector ActualPreferredVelocity = PreferredVelocity + SideVector * CurrentAgent.MovementSpeed * SideBiasMagnitude;
        return ActualPreferredVelocity.GetSafeNormal() * CurrentAgent.MovementSpeed;
    }

    return PreferredVelocity;
}

static FVector SolveOptimalVelocity_ThreadSafe(const TArray<FPlane>& Constraints, const FVector& PreferredVelocity, int32 SolverIterations)
{
    FVector NewVelocity = PreferredVelocity;
    for (int32 i = 0; i < SolverIterations; ++i)
    {
        bool bViolated = false;
        for (const FPlane& Constraint : Constraints)
        {
            const float ViolationDepth = Constraint.PlaneDot(NewVelocity);
            if (ViolationDepth < 0.0f)
            {
                NewVelocity -= (FVector)Constraint * ViolationDepth;
                bViolated = true;
            }
        }
        if (!bViolated) break;
    }
    return NewVelocity;
}

void UAvoidanceSubsystem::RunAvoidanceCalculation(const TArray<FAgentThreadData>& AgentData, TMap<uint32, FAvoidanceResult>& OutResults, const float InTimeHorizon, const int32 InMaxNeighbors, const float InGridCellSize, const float InSymmetryBreakingRangeMultiplier, const int32 InSolverIterations, const float InSideBiasMagnitude)
{
    if (AgentData.Num() < 2 || InGridCellSize <= 0) return;

    TMap<FIntVector, TArray<const FAgentThreadData*>> AgentGrid;
    for (const FAgentThreadData& Agent : AgentData)
    {
        const FIntVector GridPosition(FMath::FloorToInt(Agent.Position.X / InGridCellSize), FMath::FloorToInt(Agent.Position.Y / InGridCellSize), FMath::FloorToInt(Agent.Position.Z / InGridCellSize));
        AgentGrid.FindOrAdd(GridPosition).Add(&Agent);
    }

    for (const FAgentThreadData& Agent : AgentData)
    {
        TArray<const FAgentThreadData*> NearbyAgents;
        const int32 SearchRange = FMath::Max(1, FMath::CeilToInt((Agent.AgentRadius * 2) / InGridCellSize));
        const FIntVector CenterGridPosition(FMath::FloorToInt(Agent.Position.X / InGridCellSize), FMath::FloorToInt(Agent.Position.Y / InGridCellSize), FMath::FloorToInt(Agent.Position.Z / InGridCellSize));

        for (int32 z = -SearchRange; z <= SearchRange; ++z)
        {
            for (int32 y = -SearchRange; y <= SearchRange; ++y)
            {
                for (int32 x = -SearchRange; x <= SearchRange; ++x)
                {
                    if (const TArray<const FAgentThreadData*>* CellAgents = AgentGrid.Find(CenterGridPosition + FIntVector(x, y, z)))
                    {
                        for (const FAgentThreadData* OtherAgent : *CellAgents)
                        {
                            if (OtherAgent->AgentId != Agent.AgentId)
                            {
                                NearbyAgents.Add(OtherAgent);
                            }
                        }
                    }
                }
            }
        }

        if (NearbyAgents.Num() > InMaxNeighbors)
        {
            NearbyAgents.Sort([&Agent](const FAgentThreadData& Lhs, const FAgentThreadData& Rhs) {
                return FVector::DistSquared(Agent.Position, Lhs.Position) < FVector::DistSquared(Agent.Position, Rhs.Position);
                });
            NearbyAgents.SetNum(InMaxNeighbors);
        }

        TArray<FPlane> OrcaConstraints;
        GenerateAgentConstraints_ThreadSafe(Agent, NearbyAgents, OrcaConstraints, InTimeHorizon);

        const FVector ActualPreferredVelocity = CalculateSymmetryBreakingVelocity_ThreadSafe(Agent, Agent.PreferredVelocity, NearbyAgents, InSymmetryBreakingRangeMultiplier, InSideBiasMagnitude);
        const FVector OptimalVelocity = SolveOptimalVelocity_ThreadSafe(OrcaConstraints, ActualPreferredVelocity, InSolverIterations);

        OutResults.Add(Agent.AgentId, { Agent.AgentId, OptimalVelocity, Agent.PreferredVelocity, ActualPreferredVelocity });
    }
}

void UAvoidanceSubsystem::GenerateVoxelConstraints(UPathfindingComponent* CurrentAgent, TArray<FPlane>& OutConstraints)
{
    UWorld* World = GetWorld();
    if (!World || !CurrentAgent || !CurrentAgent->GetOwner()) return;

    const FVector PositionA = CurrentAgent->GetOwner()->GetActorLocation();
    const float RadiusA = CurrentAgent->GetAgentRadius();
    const FVector CurrentVelocityA = CurrentAgent->GetCurrentVelocity();

    FCollisionObjectQueryParams ObjectQueryParams;
    ObjectQueryParams.AddObjectTypesToQuery(ECC_WorldStatic);
    ObjectQueryParams.AddObjectTypesToQuery(ECC_PhysicsBody);

    FCollisionQueryParams CollisionParams;
    CollisionParams.AddIgnoredActor(CurrentAgent->GetOwner());

    TArray<FOverlapResult> OverlapResults;
    const float OverlapRadius = RadiusA * 1.f;

    bool bHasOverlaps = World->OverlapMultiByObjectType(
        OverlapResults,
        PositionA,
        FQuat::Identity,
        ObjectQueryParams,
        FCollisionShape::MakeSphere(OverlapRadius),
        CollisionParams
    );

    if (bHasOverlaps)
    {
        for (const FOverlapResult& Result : OverlapResults)
        {
            UPrimitiveComponent* HitComponent = Result.GetComponent();
            if (!HitComponent) continue;

            FVector ClosestPointOnObstacle;
            if (!HitComponent->GetClosestPointOnCollision(PositionA, ClosestPointOnObstacle))
            {
                continue;
            }

            const FVector RelativePos = ClosestPointOnObstacle - PositionA;

            if (RelativePos.SizeSquared() < FMath::Square(RadiusA))
            {
                const FVector RelativeVel = CurrentVelocityA; // ObstacleVel is Zero
                const float InverseTimeHorizon = 1.0f / TimeHorizon;
                const FVector w = RelativeVel - RelativePos * InverseTimeHorizon;
                const float w_len_sq = w.SizeSquared();
                FVector u = FVector::ZeroVector;

                if (w_len_sq > KINDA_SMALL_NUMBER)
                {
                    const float DotProduct = FVector::DotProduct(RelativePos, w);
                    if (DotProduct < 0.0f && FMath::Square(DotProduct) > w_len_sq * FMath::Square(RadiusA))
                    {
                        const float w_len = FMath::Sqrt(w_len_sq);
                        u = (w / w_len) * (RadiusA * InverseTimeHorizon - w_len);
                    }
                }

                if (!u.IsNearlyZero())
                {
                    const FVector w_correction = u; // 정적 장애물은 100% 책임
                    const FVector n_correction = u.GetSafeNormal();
                    if (!n_correction.IsNearlyZero())
                    {
                        OutConstraints.Emplace(CurrentVelocityA + w_correction, n_correction);
                    }
                }
            }
        }
    }
}

FVector UAvoidanceSubsystem::SolveOptimalVelocity(const TArray<FPlane>& Constraints, const FVector& PreferredVelocity) const
{
    FVector NewVelocity = PreferredVelocity;
    for (int32 i = 0; i < SolverIterations; ++i)
    {
        bool bViolated = false;
        for (const FPlane& Constraint : Constraints)
        {
            const float ViolationDepth = Constraint.PlaneDot(NewVelocity);
            if (ViolationDepth < 0.0f)
            {
                NewVelocity -= (FVector)Constraint * ViolationDepth;
                bViolated = true;
            }
        }
        if (!bViolated) break;
    }
    return NewVelocity;
}

void UAvoidanceSubsystem::DebugVisualizeVelocities(const UPathfindingComponent* Agent, const FAvoidanceResult& Result, const FVector& FinalOptimalVelocity) const
{
    if (!GEngine || !Agent || !Agent->GetOwner()) return;

    UWorld* World = GetWorld();
    if (!World) return;

    const FVector LineStart = Agent->GetOwner()->GetActorLocation() + FVector(0, 0, 10.f);
    const float LineLengthMultiplier = 0.2f;

    // 1. 선호 속도 (Pathfinding) - 파란색
    DrawDebugLine(World, LineStart, LineStart + Result.PreferredVelocity * LineLengthMultiplier, FColor::Blue, false, -1, 0, 2.f);
    // 2. 대칭 파괴 속도 (Symmetry Broken) - 노란색
    DrawDebugLine(World, LineStart, LineStart + Result.ActualPreferredVelocity * LineLengthMultiplier, FColor::Yellow, false, -1, 0, 3.f);
    // 3. 에이전트 회피 속도 (Agent Avoidance) - 주황색
    DrawDebugLine(World, LineStart, LineStart + Result.OptimalVelocity * LineLengthMultiplier, FColor::Orange, false, -1, 0, 4.f);
    // 4. 최종 최적 속도 (Final Optimal) - 초록색
    DrawDebugLine(World, LineStart, LineStart + FinalOptimalVelocity * LineLengthMultiplier, FColor::Green, false, -1, 0, 5.f);
}
