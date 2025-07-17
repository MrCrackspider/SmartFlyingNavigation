#include "Pathfinder/Subsystem/AvoidanceSubsystem.h"
#include "Pathfinder/PathfindingComponent.h"
#include "GameFramework/Actor.h"
#include "Stats/Stats.h"
#include "DrawDebugHelpers.h"
#include "Algo/Sort.h"
#include "Pathfinder/Utility/Parallel/ParallelAvoidanceTask.h"

DECLARE_CYCLE_STAT(TEXT("AvoidanceSubsystem Tick"), STAT_AvoidanceSubsystem_Tick, STATGROUP_Game);

static void GenerateVoxelConstraints_ThreadSafe(const UWorld* World, const FAgentThreadData& CurrentAgent, const FVector& DesiredVelocity, TArray<FPlane>& OutConstraints, float TimeHorizon);
static void GenerateAgentConstraints_ThreadSafe(const FAgentThreadData& CurrentAgent, const FVector& ActualPreferredVelocity, const TArray<const FAgentThreadData*>& NearbyAgents, TArray<FPlane>& OutConstraints, float TimeHorizon);
static FVector CalculateSymmetryBreakingVelocity_ThreadSafe(const FAgentThreadData& CurrentAgent, const FVector& PreferredVelocity, const TArray<const FAgentThreadData*>& NearbyAgents, float SymmetryBreakingRangeMultiplier, float SideBiasMagnitude);


void UAvoidanceSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
    Super::Initialize(Collection);
    bIsTaskRunning = false;
    AsyncTask = nullptr;

    TimeHorizon = 1.f;
    MaxNeighbors = 4;
    GridCellSize = 100.f;
    SymmetryBreakingRangeMultiplier = 4.f;
    SolverIterations = 5;
    SideBiasMagnitude = 1.f;
    InterpSpeed = 10.f;
}

void UAvoidanceSubsystem::PostInitialize()
{
    Super::PostInitialize();
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
        TMap<uint32, FThreadAvoidanceResult> LocalResults;
        {
            FScopeLock Lock(&ResultsCriticalSection);
            LocalResults = MoveTemp(AvoidanceResults);
        }

        for (const auto& ResultPair : LocalResults)
        {
            if (UPathfindingComponent* Agent = AgentMap.FindRef(ResultPair.Key))
            {
                const TArray<FPlane>& AgentConstraints = ResultPair.Value.AgentConstraints;
                const TArray<FPlane>& VoxelConstraints = ResultPair.Value.VoxelConstraints;
                const FVector& ActualPreferredVelocity = ResultPair.Value.ActualPreferredVelocity;

                const FVector SafeVelocity = SolveOptimalVelocity(VoxelConstraints, ActualPreferredVelocity);
                const FVector FinalOptimalVelocity = SolveOptimalVelocity(AgentConstraints, SafeVelocity);

                const FVector CurrentVelocity = Agent->GetCurrentVelocity();
                FVector InterpolatedVelocity = FMath::VInterpTo(CurrentVelocity, FinalOptimalVelocity, DeltaTime, InterpSpeed);
                FVector FinalVelocity = InterpolatedVelocity;

                FHitResult HitResult;
                const FVector AgentLocation = Agent->GetOwner()->GetActorLocation();
                const FVector MoveDelta = FinalVelocity * DeltaTime;

                FCollisionShape OverlapSphere = FCollisionShape::MakeSphere(Agent->GetAvoidanceRadius());
                FCollisionQueryParams OverlapParams(TEXT("PenetrationCheck"), false);
                if (Agent->GetOwner())
                {
                    OverlapParams.AddIgnoredActor(Agent->GetOwner());
                }

                if (MoveDelta.SizeSquared() > KINDA_SMALL_NUMBER)
                {
                    bool bWillCollide = GetWorld()->SweepSingleByChannel(
                        HitResult,
                        AgentLocation,
                        AgentLocation + MoveDelta,
                        FQuat::Identity,
                        ECC_WorldStatic,
                        OverlapSphere,
                        OverlapParams
                    );

                    if (bWillCollide)
                    {
                        if (FMath::IsNearlyZero(HitResult.Time))
                        {
                            FinalVelocity = SafeVelocity;
                        }
                        else
                        {
                            const float TimeOfImpact = HitResult.Time;
                            const FVector MovementToImpact = MoveDelta * TimeOfImpact;
                            const float RemainingTime = DeltaTime * (1.f - TimeOfImpact);
                            const FVector RemainingVelocity = FVector::VectorPlaneProject(FinalVelocity, HitResult.ImpactNormal);
                            const FVector FinalMoveDelta = MovementToImpact + RemainingVelocity * RemainingTime;
                            FinalVelocity = FinalMoveDelta / DeltaTime;
                        }
                    }
                }

                Agent->ApplyMovement(FinalVelocity.GetClampedToMaxSize(Agent->GetMovementSpeed()), DeltaTime);

                if (Agent->GetMovementMode() == EPathMovementMode::Spline && !Agent->IsOnSplineLine())
                {
                    UE_LOG(LogTemp, Warning, TEXT("%s: Out Of Spline Line. Change MovementMode to Linear"), *Agent->GetOwner()->GetName());
                    Agent->SetMovementMode(EPathMovementMode::Linear);
                }

                DebugVisualizeVelocities(Agent, ResultPair.Value, FinalVelocity);
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
                RegisteredAgentComponents[0]->UpdateSteering(DeltaTime);
                RegisteredAgentComponents[0]->ApplyMovement(RegisteredAgentComponents[0]->GetPreferredVelocity(), DeltaTime);
            }
            return;
        }

        TArray<FAgentThreadData> AgentDataForThread;
        AgentDataForThread.Reserve(RegisteredAgentComponents.Num());
        for (UPathfindingComponent* AgentComponent : RegisteredAgentComponents)
        {
            AgentComponent->UpdateSteering(DeltaTime);

            if (AgentComponent && AgentComponent->GetOwner())
            {
                AgentDataForThread.Emplace(FAgentThreadData{
                    AgentComponent->GetUniqueID(),
                    AgentComponent->GetOwner()->GetActorLocation(),
                    AgentComponent->GetCurrentVelocity(),
                    AgentComponent->GetPreferredVelocity(),
                    AgentComponent->GetAvoidanceRadius(),
                    AgentComponent->GetMovementSpeed(),
                    AgentComponent->GetUniqueID()
                    });
            }
        }

        {
            FScopeLock Lock(&ResultsCriticalSection);
            AvoidanceResults.Empty();
        }

        AsyncTask = new FAsyncTask<FParallelAvoidanceTask>(GetWorld(), MoveTemp(AgentDataForThread), AvoidanceResults, ResultsCriticalSection, TimeHorizon, MaxNeighbors, GridCellSize, SymmetryBreakingRangeMultiplier, SideBiasMagnitude);
        AsyncTask->StartBackgroundTask();
        bIsTaskRunning = true;
    }
}

void UAvoidanceSubsystem::RunAvoidanceCalculation(
    const UWorld* World,
    const TArray<FAgentThreadData>& AgentData,
    TMap<uint32, FThreadAvoidanceResult>& OutResults,
    const float InTimeHorizon,
    const int32 InMaxNeighbors,
    const float InGridCellSize,
    const float InSymmetryBreakingRangeMultiplier,
    const float InSideBiasMagnitude)
{
    if (!World || AgentData.Num() < 1 || InGridCellSize <= 0) return;

    TMap<FIntVector, TArray<const FAgentThreadData*>> AgentGrid;
    if (AgentData.Num() > 1)
    {
        for (const FAgentThreadData& Agent : AgentData)
        {
            const FIntVector GridPosition(FMath::FloorToInt(Agent.Position.X / InGridCellSize), FMath::FloorToInt(Agent.Position.Y / InGridCellSize), FMath::FloorToInt(Agent.Position.Z / InGridCellSize));
            AgentGrid.FindOrAdd(GridPosition).Add(&Agent);
        }
    }

    for (const FAgentThreadData& Agent : AgentData)
    {
        TArray<const FAgentThreadData*> NearbyAgents;
        if (AgentData.Num() > 1)
        {
            const int32 SearchRange = FMath::Max(1, FMath::CeilToInt(Agent.AvoidanceRadius / InGridCellSize));
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
        }

        const FVector ActualPreferredVelocity = CalculateSymmetryBreakingVelocity_ThreadSafe(Agent, Agent.PreferredVelocity, NearbyAgents, InSymmetryBreakingRangeMultiplier, InSideBiasMagnitude);

        TArray<FPlane> AgentConstraints;
        GenerateAgentConstraints_ThreadSafe(Agent, ActualPreferredVelocity, NearbyAgents, AgentConstraints, InTimeHorizon);

        TArray<FPlane> VoxelConstraints;
        GenerateVoxelConstraints_ThreadSafe(World, Agent, ActualPreferredVelocity, VoxelConstraints, InTimeHorizon);

        OutResults.Add(Agent.AgentId, { Agent.AgentId, AgentConstraints, VoxelConstraints, Agent.PreferredVelocity, ActualPreferredVelocity });
    }
}

static void GenerateVoxelConstraints_ThreadSafe(const UWorld* World, const FAgentThreadData& CurrentAgent, const FVector& DesiredVelocity, TArray<FPlane>& OutConstraints, float TimeHorizon)
{
    if (!World) return;

    const FVector PositionA = CurrentAgent.Position;
    const float RadiusA = CurrentAgent.AvoidanceRadius;
    const FVector VelocityA = DesiredVelocity;
    const FVector EndPoint = PositionA + VelocityA * TimeHorizon;

    FCollisionObjectQueryParams ObjectQueryParams(FCollisionObjectQueryParams::InitType::AllStaticObjects);
    FCollisionQueryParams CollisionParams(TEXT("VoxelAvoidance_ThreadSafe"), false);
    CollisionParams.bTraceComplex = true;

    TArray<FHitResult> HitResults;
    const bool bHasHits = World->SweepMultiByObjectType(
        HitResults,
        PositionA,
        EndPoint,
        FQuat::Identity,
        ObjectQueryParams,
        FCollisionShape::MakeSphere(RadiusA),
        CollisionParams
    );

    if (bHasHits)
    {
        for (const FHitResult& Hit : HitResults)
        {
            if (Hit.Time > KINDA_SMALL_NUMBER)
            {
                const FVector RelativePosition = Hit.ImpactPoint - PositionA;
                const float EffectiveTimeHorizon = Hit.Time;
                const float InverseEffectiveTimeHorizon = 1.0f / EffectiveTimeHorizon;
                const FVector w = VelocityA - RelativePosition * InverseEffectiveTimeHorizon;
                const float w_sq = w.SizeSquared();
                FVector u = FVector::ZeroVector;
                const float VORadius = RadiusA * InverseEffectiveTimeHorizon;
                const float VORadiusSq = FMath::Square(VORadius);

                if (FVector::DotProduct(w, RelativePosition) < 0.0f && FMath::Square(FVector::DotProduct(w, RelativePosition)) > w_sq * VORadiusSq)
                {
                    const float w_len = FMath::Sqrt(w_sq);
                    const FVector n_w = w / w_len;
                    u = (n_w * VORadius - w);
                }

                if (!u.IsNearlyZero(KINDA_SMALL_NUMBER))
                {
                    const FVector n_correction = u.GetSafeNormal();
                    if (!n_correction.IsNearlyZero(KINDA_SMALL_NUMBER))
                    {
                        OutConstraints.Emplace(VelocityA + u, n_correction);
                    }
                }
            }
            else
            {
                const FVector Normal = Hit.ImpactNormal;
                if (!Normal.IsNearlyZero())
                {
                    OutConstraints.Emplace(FVector::ZeroVector, Normal);
                }
            }
        }
    }
}

static void GenerateAgentConstraints_ThreadSafe(const FAgentThreadData& CurrentAgent, const FVector& ActualPreferredVelocity, const TArray<const FAgentThreadData*>& NearbyAgents, TArray<FPlane>& OutConstraints, float TimeHorizon)
{
    for (const FAgentThreadData* AgentB : NearbyAgents)
    {
        const FVector RelativePosition = AgentB->Position - CurrentAgent.Position;
        const FVector RelativeVelocity = ActualPreferredVelocity - AgentB->CurrentVelocity;
        const float DistSq = RelativePosition.SizeSquared();
        const float CombinedRadius = CurrentAgent.AvoidanceRadius + AgentB->AvoidanceRadius;
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
                OutConstraints.Emplace(ActualPreferredVelocity + w_correction, n_correction);
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
        const float CombinedRadius = CurrentAgent.AvoidanceRadius + AgentB->AvoidanceRadius;

        if (DistToAgentBSq < FMath::Square(CombinedRadius * SymmetryBreakingRangeMultiplier) && FVector::DotProduct(PreferredVelocity, ToAgentB) > 0)
        {
            if (DistToAgentBSq < MinThreatDistSq)
            {
                MinThreatDistSq = DistToAgentBSq;
                ObstacleAgent = AgentB;
            }
        }
    }

    bool bShouldYield = false;
    if (ObstacleAgent)
    {
        if (CurrentAgent.AvoidanceRadius < ObstacleAgent->AvoidanceRadius)
        {
            bShouldYield = true;
        }
        else if (FMath::IsNearlyEqual(CurrentAgent.AvoidanceRadius, ObstacleAgent->AvoidanceRadius) && CurrentAgent.UniqueID > ObstacleAgent->UniqueID)
        {
            bShouldYield = true;
        }
    }

    if (bShouldYield)
    {
        FVector ForwardVec = PreferredVelocity.GetSafeNormal();
        FVector SideVector = FVector::CrossProduct(ForwardVec, FVector::UpVector);
        if (SideVector.IsNearlyZero())
        {
            SideVector = FVector::CrossProduct(ForwardVec, FVector::RightVector);
        }
        SideVector.Normalize();

        // bCanPass
        //FVector ActualPreferredVelocity = PreferredVelocity + SideVector * CurrentAgent.MovementSpeed * SideBiasMagnitude;
        //return ActualPreferredVelocity.GetSafeNormal() * CurrentAgent.MovementSpeed;
        FVector ActualPreferredVelocity = SideVector * CurrentAgent.MovementSpeed * SideBiasMagnitude;
        return ActualPreferredVelocity;
    }

    return PreferredVelocity;
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
                NewVelocity -= (FVector)Constraint * ViolationDepth * (1.01f);
                bViolated = true;
            }
        }
        if (!bViolated) break;
    }
    return NewVelocity;
}

void UAvoidanceSubsystem::DebugVisualizeVelocities(const UPathfindingComponent* Agent, const FThreadAvoidanceResult& Result, const FVector& FinalOptimalVelocity) const
{
    if (!GEngine || !Agent || !Agent->GetOwner()) return;

    UWorld* World = GetWorld();
    if (!World) return;

    const FVector LineStart = Agent->GetOwner()->GetActorLocation() + FVector(0, 0, 10.f);
    const float LineLengthMultiplier = 0.2f;

    DrawDebugLine(World, LineStart, LineStart + Result.PreferredVelocity * LineLengthMultiplier, FColor::Blue, false, -1, 0, 2.f);
    DrawDebugLine(World, LineStart, LineStart + Result.ActualPreferredVelocity * LineLengthMultiplier, FColor::Yellow, false, -1, 0, 3.f);
    DrawDebugLine(World, LineStart, LineStart + FinalOptimalVelocity * LineLengthMultiplier, FColor::Red, false, -1, 0, 5.f);
}