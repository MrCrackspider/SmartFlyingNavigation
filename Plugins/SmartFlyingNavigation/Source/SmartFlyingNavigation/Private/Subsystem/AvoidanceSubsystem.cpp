// Copyright © 2025 TeamHamDev. All Rights Reserved.

#include "Subsystem/AvoidanceSubsystem.h"
#include "Subsystem/OctreeNavSubsystem.h"
#include "Component/PathfindingComponent.h"
#include "NavCore/DynamicObstacleRegistry.h"
#include "NavCore/SmartFlyingNavStats.h"
#include "GameFramework/Actor.h"
#include "Stats/Stats.h"
#include "Engine/Engine.h"
#include "DrawDebugHelpers.h"
#include "Algo/Sort.h"
#include "CollisionQueryParams.h"
#include "CollisionShape.h"
#include "Utility/Parallel/ParallelAvoidanceTask.h"

DECLARE_CYCLE_STAT(TEXT("AvoidanceSubsystem Tick"), STAT_AvoidanceSubsystem_Tick, STATGROUP_Game);

static void GenerateVoxelConstraints_ThreadSafe(const UWorld* World, const FAgentThreadData& CurrentAgent, const FVector& DesiredVelocity, TArray<FPlane>& OutConstraints, float TimeHorizon);
static void GenerateAgentConstraints_ThreadSafe(const FAgentThreadData& CurrentAgent, const FVector& ActualPreferredVelocity, const TArray<const FAgentThreadData*>& NearbyAgents, TArray<FPlane>& OutConstraints, float TimeHorizon);
static FVector CalculateSymmetryBreakingVelocity_ThreadSafe(const FAgentThreadData& CurrentAgent, const FVector& PreferredVelocity, const TArray<const FAgentThreadData*>& NearbyAgents, float SymmetryBreakingRangeMultiplier, float SideBiasMagnitude);


UAvoidanceSubsystem::~UAvoidanceSubsystem() = default;

void UAvoidanceSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
    Super::Initialize(Collection);
}

void UAvoidanceSubsystem::PostInitialize()
{
    Super::PostInitialize();
    ApplyMobileLowSpecSettings();
}

void UAvoidanceSubsystem::Deinitialize()
{
    if (AsyncTask)
    {
        AsyncTask->EnsureCompletion();
        AsyncTask.Reset();
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

void UAvoidanceSubsystem::GatherDynamicObstacleData(TArray<FDynamicObstacleThreadData>& OutObstacles) const
{
    OutObstacles.Reset();

    const UWorld* World = GetWorld();
    const UOctreeNavSubsystem* NavSubsystem = World ? World->GetSubsystem<UOctreeNavSubsystem>() : nullptr;
    if (!NavSubsystem)
    {
        return;
    }

    TArray<FDynamicObstaclePrim> Prims;
    NavSubsystem->GetObstacleRegistry().Snapshot(Prims);

    OutObstacles.Reserve(Prims.Num());
    for (const FDynamicObstaclePrim& Prim : Prims)
    {
        const float BoundingRadius = Prim.GetBoundingRadius();
        if (BoundingRadius > 0.f)
        {
            OutObstacles.Emplace(FDynamicObstacleThreadData{ Prim.Center, Prim.Velocity, BoundingRadius });
        }
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

void UAvoidanceSubsystem::SetbUseAvoidance(bool IsUse)
{
    bUseAvoidance = IsUse;
    if (!IsUse)
    {
        AvoidanceMode = ESmartFlyingAvoidanceMode::None;
    }
    else if (AvoidanceMode == ESmartFlyingAvoidanceMode::None)
    {
        AvoidanceMode = ESmartFlyingAvoidanceMode::Simple;
    }
}

void UAvoidanceSubsystem::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
    SCOPE_CYCLE_COUNTER(STAT_AvoidanceSubsystem_Tick);

#if STATS
    SET_DWORD_STAT(STAT_SFN_Agents, RegisteredAgentComponents.Num());
    if (const UWorld* StatsWorld = GetWorld())
    {
        if (const UOctreeNavSubsystem* StatsNav = StatsWorld->GetSubsystem<UOctreeNavSubsystem>())
        {
            SET_DWORD_STAT(STAT_SFN_Obstacles, StatsNav->GetObstacleRegistry().Num());
        }
    }
#endif

    if (!GetbUseAvoidance())
    {
        return;
    }

    if (AvoidanceMode == ESmartFlyingAvoidanceMode::Simple)
    {
        TickSimpleAvoidance(DeltaTime);
        return;
    }

    float EffectiveAvoidanceUpdateInterval = 0.f;
#if PLATFORM_ANDROID
    EffectiveAvoidanceUpdateInterval = bUseMobileLowSpecSettings ? MobileAvoidanceUpdateInterval : 0.f;
#endif

    AvoidanceUpdateAccumulator += DeltaTime;
    if (!bIsTaskRunning && EffectiveAvoidanceUpdateInterval > 0.f && AvoidanceUpdateAccumulator < EffectiveAvoidanceUpdateInterval)
    {
        for (UPathfindingComponent* AgentComponent : RegisteredAgentComponents)
        {
            if (!AgentComponent)
            {
                continue;
            }

            AgentComponent->UpdateSteering(DeltaTime);
            const FVector Vel = AgentComponent->GetPreferredVelocity();
            AgentComponent->ApplyMovement(Vel, DeltaTime);
            AgentComponent->CurrentVelocity = Vel;
        }
        return;
    }

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
                FVector FinalVelocity;

                const TArray<FPlane>& AgentConstraints = ResultPair.Value.AgentConstraints;
                const FVector& ActualPreferredVelocity = ResultPair.Value.ActualPreferredVelocity;

                // Solve agent avoidance constraints only.
                // Wall avoidance is handled by collision sweep + AddActorWorldOffset(sweep=true).
                const FVector FinalOptimalVelocity = SolveOptimalVelocity(AgentConstraints, ActualPreferredVelocity);

                // Apply velocity smoothing. InterpSpeed should be high enough (10+) for responsive avoidance.
                // Too low → agent can't follow ORCA output → spinning behavior.
                const FVector CurrentVelocity = Agent->GetCurrentVelocity();
                const float EffectiveInterpSpeed = FMath::Max(InterpSpeed, 5.0f);
                FinalVelocity = FMath::VInterpTo(CurrentVelocity, FinalOptimalVelocity, DeltaTime, EffectiveInterpSpeed);

                FHitResult HitResult;
                const FVector AgentLocation = Agent->GetOwner()->GetActorLocation();
                const FVector MoveDelta = FinalVelocity * DeltaTime;

                if (!Agent->bExecutorHandlesCollision)
                {
                    // Use AgentRadius (physical size) for collision, not AvoidanceRadius (ORCA buffer).
                FCollisionShape OverlapSphere = FCollisionShape::MakeSphere(Agent->GetAgentRadius());
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
                                // Already touching a wall. Slide along the wall surface.
                                FinalVelocity = FVector::VectorPlaneProject(FinalVelocity, HitResult.ImpactNormal);
                            }
                            else
                            {
                                // Will collide partway - slide along the wall after impact.
                                const float TimeOfImpact = HitResult.Time;
                                const FVector MovementToImpact = MoveDelta * TimeOfImpact;
                                const float RemainingTime = DeltaTime * (1.f - TimeOfImpact);
                                const FVector SlideVelocity = FVector::VectorPlaneProject(FinalVelocity, HitResult.ImpactNormal);
                                const FVector FinalMoveDelta = MovementToImpact + SlideVelocity * RemainingTime;
                                FinalVelocity = FinalMoveDelta / DeltaTime;
                            }
                        }
                    }
                }

                const FVector ClampedVelocity = FinalVelocity.GetClampedToMaxSize(Agent->GetMovementSpeed());
                Agent->ApplyMovement(ClampedVelocity, DeltaTime);

                // Update the agent's CurrentVelocity with the actual avoidance-adjusted result.
                // This is critical: ORCA uses CurrentVelocity from the previous frame to compute relative velocities.
                Agent->CurrentVelocity = ClampedVelocity;

                if (Agent->GetMovementMode() == EPathMovementMode::Spline && !Agent->IsOnSplineLine() && Agent->FoundPath.Num() > 0)
                {
                    Agent->SetMovementMode(EPathMovementMode::Linear);
                }

                if (bShowVelocityDebugLine)
                {
                    DebugVisualizeVelocities(Agent, ResultPair.Value, FinalVelocity);
                }
            }
        }

        AsyncTask->EnsureCompletion();
        AsyncTask.Reset();
        bIsTaskRunning = false;
    }

    if (bIsTaskRunning)
    {
        for (UPathfindingComponent* AgentComponent : RegisteredAgentComponents)
        {
            if (!AgentComponent)
            {
                continue;
            }

            AgentComponent->UpdateSteering(DeltaTime);
            const FVector Vel = AgentComponent->GetPreferredVelocity();
            AgentComponent->ApplyMovement(Vel, DeltaTime);
            AgentComponent->CurrentVelocity = Vel;
        }
        return;
    }

    if (!bIsTaskRunning)
    {
        AvoidanceUpdateAccumulator = 0.f;
        if (RegisteredAgentComponents.Num() < 2)
        {
            if (RegisteredAgentComponents.Num() == 1 && RegisteredAgentComponents[0])
            {
                UPathfindingComponent* Solo = RegisteredAgentComponents[0];
                Solo->UpdateSteering(DeltaTime);
                const FVector Vel = Solo->GetPreferredVelocity();
                Solo->ApplyMovement(Vel, DeltaTime);
                Solo->CurrentVelocity = Vel;
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
                FAgentThreadData AgentData;
                AgentData.AgentId = AgentComponent->GetUniqueID();
                AgentData.Position = AgentComponent->GetOwner()->GetActorLocation();
                AgentData.UpVector = AgentComponent->GetOwner()->GetActorUpVector();
                AgentData.CurrentVelocity = AgentComponent->GetCurrentVelocity();
                AgentData.PreferredVelocity = AgentComponent->GetPreferredVelocity();
                AgentData.AvoidanceRadius = AgentComponent->GetAvoidanceRadius();
                AgentData.MovementSpeed = AgentComponent->GetMovementSpeed();

                AgentDataForThread.Emplace(MoveTemp(AgentData));
            }
        }
        // Collect dynamic obstacle snapshots from the world obstacle registry.
        TArray<FDynamicObstacleThreadData> DynamicObstacleData;
        if (bAvoidDynamicObstacles)
        {
            GatherDynamicObstacleData(DynamicObstacleData);
        }

        {
            FScopeLock Lock(&ResultsCriticalSection);
            AvoidanceResults.Empty();
        }

        AsyncTask = MakeUnique<FAsyncTask<FParallelAvoidanceTask>>(GetWorld(), MoveTemp(AgentDataForThread), AvoidanceResults, ResultsCriticalSection, TimeHorizon, MaxNeighbors, GridCellSize, SymmetryBreakingRangeMultiplier, SideBiasMagnitude, MoveTemp(DynamicObstacleData));
        AsyncTask->StartBackgroundTask();
        bIsTaskRunning = true;
    }
}

TStatId UAvoidanceSubsystem::GetStatId() const
{
    RETURN_QUICK_DECLARE_CYCLE_STAT(UAvoidanceSubsystem, STATGROUP_Tickables);
}

void UAvoidanceSubsystem::TickSimpleAvoidance(float DeltaTime)
{
    TArray<UPathfindingComponent*> ValidAgents;
    ValidAgents.Reserve(RegisteredAgentComponents.Num());

    for (UPathfindingComponent* AgentComponent : RegisteredAgentComponents)
    {
        if (AgentComponent && AgentComponent->GetOwner())
        {
            AgentComponent->UpdateSteering(DeltaTime);
            ValidAgents.Add(AgentComponent);
        }
    }

    for (UPathfindingComponent* Agent : ValidAgents)
    {
        AActor* Owner = Agent->GetOwner();
        if (!Owner)
        {
            continue;
        }

        FVector FinalVelocity = Agent->GetPreferredVelocity();
        FVector Separation = FVector::ZeroVector;
        int32 UsedNeighbors = 0;

        const FVector AgentLocation = Owner->GetActorLocation();
        for (UPathfindingComponent* OtherAgent : ValidAgents)
        {
            if (OtherAgent == Agent || !OtherAgent->GetOwner())
            {
                continue;
            }

            const FVector ToAgent = AgentLocation - OtherAgent->GetOwner()->GetActorLocation();
            const float DistanceSq = ToAgent.SizeSquared();
            const float DesiredDistance = Agent->GetAvoidanceRadius() + OtherAgent->GetAvoidanceRadius();
            if (DesiredDistance <= KINDA_SMALL_NUMBER || DistanceSq <= KINDA_SMALL_NUMBER || DistanceSq > FMath::Square(DesiredDistance))
            {
                continue;
            }

            const float Distance = FMath::Sqrt(DistanceSq);
            const float Weight = 1.f - (Distance / DesiredDistance);
            Separation += (ToAgent / Distance) * Weight;

            if (++UsedNeighbors >= FMath::Max(1, MobileMaxNeighbors))
            {
                break;
            }
        }

        if (!Separation.IsNearlyZero())
        {
            FinalVelocity += Separation.GetSafeNormal() * Agent->GetMovementSpeed() * 0.5f;
        }

        const FVector ClampedVelocity = FinalVelocity.GetClampedToMaxSize(Agent->GetMovementSpeed());
        Agent->ApplyMovement(ClampedVelocity, DeltaTime);
        Agent->CurrentVelocity = ClampedVelocity;
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
    const float InSideBiasMagnitude,
    const TArray<FDynamicObstacleThreadData>& DynamicObstacles)
{
    if (!World || AgentData.Num() < 1 || InGridCellSize <= 0) return;

    TMap<FIntVector, TArray<const FAgentThreadData*>> AgentGrid;
    if (AgentData.Num() > 1)
    {
        for (const FAgentThreadData& Agent : AgentData)
        {
            const float Radius = Agent.AvoidanceRadius;
            const FVector MinBounds = Agent.Position - FVector(Radius);
            const FVector MaxBounds = Agent.Position + FVector(Radius);

            const FIntVector MinGridPos(
                FMath::FloorToInt(MinBounds.X / InGridCellSize),
                FMath::FloorToInt(MinBounds.Y / InGridCellSize),
                FMath::FloorToInt(MinBounds.Z / InGridCellSize)
            );

            const FIntVector MaxGridPos(
                FMath::FloorToInt(MaxBounds.X / InGridCellSize),
                FMath::FloorToInt(MaxBounds.Y / InGridCellSize),
                FMath::FloorToInt(MaxBounds.Z / InGridCellSize)
            );

            for (int32 z = MinGridPos.Z; z <= MaxGridPos.Z; ++z)
            {
                for (int32 y = MinGridPos.Y; y <= MaxGridPos.Y; ++y)
                {
                    for (int32 x = MinGridPos.X; x <= MaxGridPos.X; ++x)
                    {
                        AgentGrid.FindOrAdd(FIntVector(x, y, z)).Add(&Agent);
                    }
                }
            }
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

        // Wall avoidance is handled by collision sweep in Tick() + AddActorWorldOffset(sweep=true).
        // No VoxelConstraints needed - removes expensive per-agent SweepMultiByObjectType from background thread.
        TArray<FPlane> VoxelConstraints;

        // Generate ORCA constraints from dynamic obstacles (treated as moving agents with known velocity).
        for (const FDynamicObstacleThreadData& Obstacle : DynamicObstacles)
        {
            const FVector RelativePosition = Obstacle.Position - Agent.Position;
            const float DistSq = RelativePosition.SizeSquared();
            const float CombinedRadius = Agent.AvoidanceRadius + Obstacle.Radius;
            const float InteractionRange = CombinedRadius * 3.0f;

            // Skip obstacles that are too far away.
            if (DistSq > FMath::Square(InteractionRange)) continue;

            const FVector RelativeVelocity = ActualPreferredVelocity - Obstacle.Velocity;
            const float CombinedRadiusSq = FMath::Square(CombinedRadius);
            float EffectiveTimeHorizon = InTimeHorizon;

            // TTC calculation.
            const float RelativeVelSq = RelativeVelocity.SizeSquared();
            if (RelativeVelSq > KINDA_SMALL_NUMBER)
            {
                const float Dot = FVector::DotProduct(RelativePosition, RelativeVelocity);
                const float Discriminant = FMath::Square(Dot) - RelativeVelSq * (DistSq - CombinedRadiusSq);
                if (Discriminant >= 0.f)
                {
                    const float TTC = (-Dot - FMath::Sqrt(Discriminant)) / RelativeVelSq;
                    if (TTC > 0.f)
                    {
                        EffectiveTimeHorizon = FMath::Min(InTimeHorizon, TTC);
                    }
                }
            }

            FVector u = FVector::ZeroVector;
            if (DistSq > CombinedRadiusSq)
            {
                const float InvTau = 1.0f / EffectiveTimeHorizon;
                const FVector w = RelativeVelocity - RelativePosition * InvTau;
                const float wSq = w.SizeSquared();
                const float VOptRadius = CombinedRadius * InvTau;
                if (FVector::DotProduct(w, RelativePosition) < 0.0f &&
                    FMath::Square(FVector::DotProduct(w, RelativePosition)) > wSq * FMath::Square(VOptRadius))
                {
                    const float wLen = FMath::Sqrt(wSq);
                    const FVector nw = w / wLen;
                    u = nw * VOptRadius - w;
                }
            }
            else
            {
                // Penetrating - push apart.
                const float Dist = FMath::Sqrt(DistSq);
                FVector PushDir = (Dist > KINDA_SMALL_NUMBER) ? -RelativePosition / Dist : FVector(1, 0, 0);
                u = PushDir * ((CombinedRadius - Dist) / EffectiveTimeHorizon + 0.01f);
            }

            if (!u.IsNearlyZero(KINDA_SMALL_NUMBER))
            {
                // Full responsibility on the agent (obstacle doesn't cooperate).
                const FVector nCorrection = u.GetSafeNormal();
                if (!nCorrection.IsNearlyZero(KINDA_SMALL_NUMBER))
                {
                    AgentConstraints.Emplace(ActualPreferredVelocity + u, nCorrection);
                }
            }
        }

        OutResults.Add(Agent.AgentId, { Agent.AgentId, AgentConstraints, VoxelConstraints, Agent.PreferredVelocity, ActualPreferredVelocity });
    }
}

static FVector CalculateSymmetryBreakingVelocity_ThreadSafe(const FAgentThreadData& CurrentAgent, const FVector& PreferredVelocity, const TArray<const FAgentThreadData*>& NearbyAgents, float SymmetryBreakingRangeMultiplier, float SideBiasMagnitude)
{
    if (NearbyAgents.IsEmpty() || PreferredVelocity.IsNearlyZero())
    {
        return PreferredVelocity;
    }

    // Accumulate weighted bias from ALL nearby agents (not just the closest).
    // This handles multi-agent convergence scenarios.
    FVector AccumulatedBias = FVector::ZeroVector;
    const FVector ForwardVector = PreferredVelocity.GetSafeNormal();
    const FVector UpVector = CurrentAgent.UpVector;

    for (const FAgentThreadData* AgentB : NearbyAgents)
    {
        const FVector ToAgentB = AgentB->Position - CurrentAgent.Position;
        const float DistToAgentBSq = ToAgentB.SizeSquared();
        const float CombinedRadius = CurrentAgent.AvoidanceRadius + AgentB->AvoidanceRadius;
        const float ThresholdSq = FMath::Square(CombinedRadius * SymmetryBreakingRangeMultiplier);

        if (DistToAgentBSq < ThresholdSq && DistToAgentBSq > KINDA_SMALL_NUMBER)
        {
            const float Weight = 1.0f - FMath::Sqrt(DistToAgentBSq) / (CombinedRadius * SymmetryBreakingRangeMultiplier);
            FVector SideVector = FVector::CrossProduct(ForwardVector, UpVector).GetSafeNormal();

            if (!SideVector.IsNearlyZero())
            {
                const float DotWithSide = FVector::DotProduct(ToAgentB, SideVector);
                if (DotWithSide > 0.0f) SideVector *= -1.0f;
                AccumulatedBias += SideVector * Weight;
            }
        }
    }

    if (!AccumulatedBias.IsNearlyZero())
    {
        return PreferredVelocity + AccumulatedBias.GetSafeNormal() * CurrentAgent.MovementSpeed * SideBiasMagnitude;
    }
    return PreferredVelocity;
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

            if (Discriminant >= 0.f)
            {
                const float TTC = (-DotProduct_RelPos_RelVel - FMath::Sqrt(Discriminant)) / RelativeVelSq;
                if (TTC > 0.f)
                {
                    EffectiveTimeHorizon = FMath::Min(TimeHorizon, TTC);
                    bIsCollidingOrApproaching = true;
                }
            }
        }

        // Deadlock buffer: also generate constraints for close-but-stationary agents.
        const float DeadlockBuffer = CombinedRadius * 1.5f;
        if (DistSq > FMath::Square(DeadlockBuffer) && !bIsCollidingOrApproaching) continue;

        const float InverseEffectiveTimeHorizon = 1.0f / EffectiveTimeHorizon;
        FVector u = FVector::ZeroVector;

        if (DistSq > CombinedRadiusSq)
        {
            // Non-colliding case: standard ORCA velocity obstacle.
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
        else
        {
            // Penetrating case: agents are overlapping. Push apart.
            const float Dist = FMath::Sqrt(DistSq);
            FVector PushDir;
            if (Dist > KINDA_SMALL_NUMBER)
            {
                PushDir = -RelativePosition / Dist;
            }
            else
            {
                // Completely overlapping - pick arbitrary perpendicular direction.
                PushDir = FVector::CrossProduct(CurrentAgent.UpVector, FVector::ForwardVector).GetSafeNormal();
                if (PushDir.IsNearlyZero()) PushDir = FVector(1, 0, 0);
            }
            const float PenetrationDepth = CombinedRadius - Dist;
            u = PushDir * (PenetrationDepth * InverseEffectiveTimeHorizon + 0.01f);
        }

        if (!u.IsNearlyZero(KINDA_SMALL_NUMBER))
        {
            const float Responsibility = 0.5f;
            const FVector w_correction = u * Responsibility;
            const FVector n_correction = u.GetSafeNormal();
            if (!n_correction.IsNearlyZero(KINDA_SMALL_NUMBER))
            {
                OutConstraints.Emplace(ActualPreferredVelocity + w_correction, n_correction);
            }
        }
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

    // Wall avoidance is handled by the collision sweep in Tick(), not by ORCA constraints.
    // ORCA constraints for walls cause circling behavior because PreferredVelocity
    // points toward the waypoint (through the wall) while wall constraints push tangentially.
    TArray<FHitResult> HitResults;
    bool bHasHits = World->SweepMultiByObjectType(
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

FVector UAvoidanceSubsystem::SolveOptimalVelocity(const TArray<FPlane>& Constraints, const FVector& PreferredVelocity) const
{
    // Incremental half-plane LP solver for 3D ORCA.
    // Processes constraints one by one. For each new violated constraint:
    // 1. Project onto the plane
    // 2. If a previous constraint is violated, optimize along the intersection line
    // 3. If infeasible, find the safest velocity

    if (Constraints.Num() == 0)
    {
        return PreferredVelocity;
    }

    FVector Result = PreferredVelocity;

    for (int32 i = 0; i < Constraints.Num(); ++i)
    {
        const FVector Normal_i(Constraints[i].X, Constraints[i].Y, Constraints[i].Z);
        const float Dist_i = Constraints[i].PlaneDot(Result);

        if (Dist_i >= 0.0f)
        {
            continue; // Already satisfied.
        }

        // Project result onto plane i.
        FVector Candidate = Result - Normal_i * Dist_i;
        bool bFeasible = true;

        // Check all previous constraints.
        for (int32 j = 0; j < i; ++j)
        {
            const FVector Normal_j(Constraints[j].X, Constraints[j].Y, Constraints[j].Z);

            if (Constraints[j].PlaneDot(Candidate) >= -KINDA_SMALL_NUMBER)
            {
                continue; // Previous constraint still satisfied.
            }

            // Planes i and j conflict. Find their intersection line.
            const FVector LineDir = FVector::CrossProduct(Normal_i, Normal_j);

            if (LineDir.SizeSquared() < KINDA_SMALL_NUMBER)
            {
                // Parallel planes.
                if (FVector::DotProduct(Normal_i, Normal_j) > 0.0f)
                {
                    continue; // Compatible - use the more restrictive.
                }
                bFeasible = false;
                break;
            }

            const FVector L = LineDir.GetSafeNormal();

            // Find a point on the intersection line using Cramer's rule.
            // Solve: n_i · P = W_i and n_j · P = W_j, with P = a*n_i + b*n_j
            const float d = FVector::DotProduct(Normal_i, Normal_j);
            const float det = 1.0f - d * d;

            if (FMath::Abs(det) < KINDA_SMALL_NUMBER)
            {
                bFeasible = false;
                break;
            }

            const float InvDet = 1.0f / det;
            const float a = (Constraints[i].W - Constraints[j].W * d) * InvDet;
            const float b = (Constraints[j].W - Constraints[i].W * d) * InvDet;
            const FVector LinePoint = a * Normal_i + b * Normal_j;

            // Optimize along the line: find t closest to PreferredVelocity.
            float tOpt = FVector::DotProduct(PreferredVelocity - LinePoint, L);

            // Clamp t against all previous constraints (0..j-1).
            for (int32 k = 0; k < j; ++k)
            {
                const FVector Normal_k(Constraints[k].X, Constraints[k].Y, Constraints[k].Z);
                const float Numerator = Constraints[k].PlaneDot(LinePoint);
                const float Denominator = FVector::DotProduct(Normal_k, L);

                if (FMath::Abs(Denominator) < KINDA_SMALL_NUMBER)
                {
                    if (Numerator < -KINDA_SMALL_NUMBER)
                    {
                        bFeasible = false;
                        break;
                    }
                    continue;
                }

                const float tBound = -Numerator / Denominator;
                if (Denominator > 0.0f)
                {
                    // Upper bound.
                    tOpt = FMath::Min(tOpt, tBound);
                }
                else
                {
                    // Lower bound.
                    tOpt = FMath::Max(tOpt, tBound);
                }
            }

            if (!bFeasible) break;

            Candidate = LinePoint + L * tOpt;
            break; // Only handle first violated previous constraint.
        }

        if (bFeasible)
        {
            Result = Candidate;
        }
        else
        {
            // Infeasible: minimize penetration depth across all constraints seen so far.
            // Project preferred velocity onto each plane and pick the least-violating result.
            float BestScore = -TNumericLimits<float>::Max();
            FVector BestVelocity = FVector::ZeroVector;

            for (int32 k = 0; k <= i; ++k)
            {
                const FVector Nk(Constraints[k].X, Constraints[k].Y, Constraints[k].Z);
                const float Dk = Constraints[k].PlaneDot(PreferredVelocity);
                const FVector Proj = PreferredVelocity - Nk * Dk;

                // Score = minimum distance to satisfaction across all other constraints.
                float MinDist = TNumericLimits<float>::Max();
                for (int32 m = 0; m <= i; ++m)
                {
                    const float PlaneDist = Constraints[m].PlaneDot(Proj);
                    MinDist = FMath::Min(MinDist, PlaneDist);
                }

                if (MinDist > BestScore)
                {
                    BestScore = MinDist;
                    BestVelocity = Proj;
                }
            }

            Result = BestVelocity;
        }
    }

    return Result;
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

void UAvoidanceSubsystem::ApplyMobileLowSpecSettings()
{
#if PLATFORM_ANDROID
    if (AvoidanceMode == ESmartFlyingAvoidanceMode::ORCA)
    {
        if (bUseMobileLowSpecSettings)
        {
            UE_LOG(LogTemp, Warning, TEXT("SmartFlyingNavigation: High ORCA avoidance is too expensive for low-spec Android. Downgrading to Simple avoidance because bUseMobileLowSpecSettings is enabled."));
            if (GEngine)
            {
                GEngine->AddOnScreenDebugMessage(-1, 8.f, FColor::Yellow, TEXT("SmartFlyingNavigation: Android downgraded High ORCA avoidance to Simple."));
            }
            AvoidanceMode = ESmartFlyingAvoidanceMode::Simple;
        }
        else
        {
            UE_LOG(LogTemp, Warning, TEXT("SmartFlyingNavigation: High ORCA avoidance is enabled on Android. This is supported, but may exceed CPU/RAM budgets on low-spec devices."));
            if (GEngine)
            {
                GEngine->AddOnScreenDebugMessage(-1, 8.f, FColor::Yellow, TEXT("SmartFlyingNavigation: High ORCA avoidance on Android may exceed low-spec budgets."));
            }
        }
    }

    if (!bUseMobileLowSpecSettings)
    {
        return;
    }

    MaxNeighbors = FMath::Min(MaxNeighbors, MobileMaxNeighbors);
    SolverIterations = FMath::Min(SolverIterations, 2);
    TimeHorizon = FMath::Min(TimeHorizon, 0.75f);
    SymmetryBreakingRangeMultiplier = FMath::Min(SymmetryBreakingRangeMultiplier, 4.f);
    bShowVelocityDebugLine = false;

    if (bMobileDisableDynamicObstacleAvoidance)
    {
        bAvoidDynamicObstacles = false;
    }
#endif
}
