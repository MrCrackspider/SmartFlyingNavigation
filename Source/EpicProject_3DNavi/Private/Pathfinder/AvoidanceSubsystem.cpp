#include "Pathfinder/AvoidanceSubsystem.h"
#include "Pathfinder/PathfindingComponent.h"
#include "Pathfinder/FenwickVoxelVolume.h"
#include "GameFramework/Actor.h"
#include "Kismet/KismetSystemLibrary.h"
#include "Stats/Stats.h"
#include "DrawDebugHelpers.h"
#include "Pathfinder/Utility/StrategyUtils.h"

DECLARE_CYCLE_STAT(TEXT("AvoidanceSubsystem Tick"), STAT_AvoidanceSubsystem_Tick, STATGROUP_Game);

void UAvoidanceSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
    Super::Initialize(Collection);

    TimeHorizon = 0.3f;
    MaxNeighbors = 4;
    GridCellSize = 200.f; // VolumeExtent의 배수이자 (64의 약수)*100로 하는게 좋음
    SymmetryBreakingRangeMultiplier = 3.f;
    SolverIterations = 5;
    //SideBiasMagnitude = 0.3f;
    //InterpSpeed = 10.f;
}

void UAvoidanceSubsystem::Deinitialize()
{
    Super::Deinitialize();
}

void UAvoidanceSubsystem::RegisterAgent(UPathfindingComponent* AgentComponent)
{
    if (AgentComponent) RegisteredAgents.AddUnique(AgentComponent);
}

void UAvoidanceSubsystem::UnregisterAgent(UPathfindingComponent* AgentComponent)
{
    RegisteredAgents.Remove(AgentComponent);
}

TStatId UAvoidanceSubsystem::GetStatId() const
{
    RETURN_QUICK_DECLARE_CYCLE_STAT(UAvoidanceSubsystem, STATGROUP_Tickables);
}

void UAvoidanceSubsystem::UpdateAgentGrid()
{
    AgentGrid.Empty();
    if (GridCellSize <= 0) return;
    for (UPathfindingComponent* Agent : RegisteredAgents)
    {
        if (!Agent || !Agent->GetOwner()) continue;
        const FVector Location = Agent->GetOwner()->GetActorLocation();
        const FIntVector GridPosition(FMath::FloorToInt(Location.X / GridCellSize), FMath::FloorToInt(Location.Y / GridCellSize), FMath::FloorToInt(Location.Z / GridCellSize));
        AgentGrid.FindOrAdd(GridPosition).Add(Agent);
    }
}

TArray<UPathfindingComponent*> UAvoidanceSubsystem::GetNearbyAgents(UPathfindingComponent* Agent)
{
    TArray<UPathfindingComponent*> NearbyAgents;
    if (!Agent || !Agent->GetOwner() || GridCellSize <= 0) return NearbyAgents;
    const FVector Location = Agent->GetOwner()->GetActorLocation();
    const FIntVector CenterGridPosition(FMath::FloorToInt(Location.X / GridCellSize), FMath::FloorToInt(Location.Y / GridCellSize), FMath::FloorToInt(Location.Z / GridCellSize));

    for (int32 z = -1; z <= 1; ++z)
    {
        for (int32 y = -1; y <= 1; ++y)
        {
            for (int32 x = -1; x <= 1; ++x)
            {
                FIntVector GridPosition = CenterGridPosition + FIntVector(x, y, z);
                if (const TArray<UPathfindingComponent*>* CellAgents = AgentGrid.Find(GridPosition))
                {
                    for (UPathfindingComponent* OtherAgent : *CellAgents)
                    {
                        if (OtherAgent != Agent)
                        {
                            NearbyAgents.Add(OtherAgent);
                        }
                    }
                }
            }
        }
    }
    return NearbyAgents;
}

void UAvoidanceSubsystem::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
    SCOPE_CYCLE_COUNTER(STAT_AvoidanceSubsystem_Tick);

    if (RegisteredAgents.Num() < 2)
    {
        if (RegisteredAgents.Num() == 1 && RegisteredAgents[0])
        {
            RegisteredAgents[0]->ApplyMovement(RegisteredAgents[0]->GetPreferredVelocity(), DeltaTime);
        }
        return;
    }

    if (DeltaTime < KINDA_SMALL_NUMBER) return;

    UpdateAgentGrid();

    TMap<UPathfindingComponent*, FVector> NewOptimalVelocities;

    for (UPathfindingComponent* AgentA : RegisteredAgents)
    {
        if (!AgentA || !AgentA->GetOwner()) continue;

        const FVector PositionA = AgentA->GetOwner()->GetActorLocation();
        const float RadiusA = AgentA->GetAgentRadius();
        const FVector CurrentVelocityA = AgentA->GetCurrentVelocity();
        const FVector PreferredVelocityA = AgentA->GetPreferredVelocity();

        TArray<FPlane> OrcaConstraints;

        TArray<UPathfindingComponent*> NearbyAgents = GetNearbyAgents(AgentA);
        if (NearbyAgents.Num() > MaxNeighbors)
        {
            NearbyAgents.Sort([PositionA](const UPathfindingComponent& Lhs, const UPathfindingComponent& Rhs) {
                return FVector::DistSquared(PositionA, Lhs.GetOwner()->GetActorLocation()) < FVector::DistSquared(PositionA, Rhs.GetOwner()->GetActorLocation());
                });
            NearbyAgents.SetNum(MaxNeighbors);
        }

        // 유령 제약 방지를 위한 TTC 로직 및 제약 조건
        for (UPathfindingComponent* AgentB : NearbyAgents)
        {
            const FVector PositionB = AgentB->GetOwner()->GetActorLocation();
            const float RadiusB = AgentB->GetAgentRadius();
            const FVector VelocityB = AgentB->GetCurrentVelocity();

            const FVector RelativePosition = PositionB - PositionA;
            const FVector RelativeVelocity = CurrentVelocityA - VelocityB;
            const float DistSq = RelativePosition.SizeSquared();
            const float CombinedRadius = RadiusA + RadiusB;
            const float CombinedRadiusSq = FMath::Square(CombinedRadius);

            float EffectiveTimeHorizon = TimeHorizon;

            const float RelativeVelSq = RelativeVelocity.SizeSquared();
            bool bIsCollidingOrApproaching = false;

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

            // 에이전트가 아직 겹치지 않았고 (DistSq > CombinedRadiusSq),
            // 동시에 충돌 경로에 있지 않다면 (bIsCollidingOrApproaching가 false)
            if (DistSq > CombinedRadiusSq && !bIsCollidingOrApproaching)
            {
                continue;
            }

            FVector u;
            const float InverseEffectiveTimeHorizon = 1.0f / EffectiveTimeHorizon;

            // 에이전트가 아직 겹치지 않았을 때 (단, 이제는 충돌 위협이 있는 경우만 여기에 도달)
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
                else
                {
                    u = FVector::ZeroVector;
                }
            }
            else // 에이전트가 이미 겹쳤을 때
            {
                const float invDeltaTime = 1.0f / DeltaTime;
                u = (RelativePosition.GetSafeNormal() * (CombinedRadius - RelativePosition.Size()) * invDeltaTime) - RelativeVelocity;
            }

            if (!u.IsNearlyZero(KINDA_SMALL_NUMBER)) // u 벡터가 0이 아니면 제약 조건 추가
            {
                const FVector w_correction = u * 0.5f;
                const FVector n_correction = u.GetSafeNormal();

                if (!n_correction.IsNearlyZero(KINDA_SMALL_NUMBER))
                {
                    OrcaConstraints.Emplace(CurrentVelocityA + w_correction, n_correction);
                }
            }
        }
        // 대칭성 파괴 로직 (목표 속도를 옆으로 완전히 트는 방식)
        FVector ActualPreferredVelocity = PreferredVelocityA;
        if (!NearbyAgents.IsEmpty() && !PreferredVelocityA.IsNearlyZero())
        {
            UPathfindingComponent* ObstacleAgent = nullptr;
            float MinThreatDistSq = TNumericLimits<float>::Max();

            // 현재 A를 향해오거나 충돌 위험이 있는 에이전트
            for (UPathfindingComponent* AgentB : NearbyAgents)
            {
                const FVector ToAgentB = AgentB->GetOwner()->GetActorLocation() - PositionA;
                const float DistToAgentBSq = ToAgentB.SizeSquared();
                const float CombinedRadius = RadiusA + AgentB->GetAgentRadius();

                // 일정 거리 이내이고, PreferredVelocityA가 AgentB를 향하고 있을 때
                if (DistToAgentBSq < FMath::Square(CombinedRadius * SymmetryBreakingRangeMultiplier) && FVector::DotProduct(PreferredVelocityA, ToAgentB) > 0)
                {
                    if (DistToAgentBSq < MinThreatDistSq)
                    {
                        MinThreatDistSq = DistToAgentBSq;
                        ObstacleAgent = AgentB;
                    }
                }
            }

            if (ObstacleAgent)
            {
                // 고유 ID를 비교하여 한쪽만 행동을 결정
                if (AgentA->GetUniqueID() > ObstacleAgent->GetUniqueID())
                {
                    FVector ForwardVec = PreferredVelocityA.GetSafeNormal();
                    FVector SideVector;

                    // 수직 충돌 엣지 케이스 처리
                    if (FMath::Abs(FVector::DotProduct(ForwardVec, FVector::UpVector)) > 0.99f)
                    {
                        SideVector = FVector::CrossProduct(ForwardVec, FVector::RightVector);
                    }
                    else
                    {
                        SideVector = FVector::CrossProduct(ForwardVec, FVector::UpVector);
                    }
                    SideVector.Normalize();

                    ActualPreferredVelocity = SideVector * AgentA->MovementSpeed;
                    // 아래 방법으로 수정을 하면 자연스럽지만, 매직넘버를 찾기 매우 까다로움.
                    // MovementSpeed, Radius, GridCellSize 등을 고려해야 함.
                    //ActualPreferredVelocity += SideVector * AgentA->MovementSpeed;
                }
            }
        }

        FVector OptimalVelocity = SolveOptimalVelocity_Final(OrcaConstraints, ActualPreferredVelocity);

        AFenwickVoxelVolume* VoxelVolume = Cast<AFenwickVoxelVolume>(AgentA->TargetVolume);
        if (VoxelVolume)
        {
            // 가성비의 16
            const int32 NumSamples = 16; 
            const float LookaheadDist = RadiusA * 2.0f;

            for (int32 i = 0; i < NumSamples; ++i)
            {
                // Fibonacci Sphere
                const float Y = 1.0f - (i / (float)(NumSamples - 1)) * 2.0f;
                const float RadiusAtY = FMath::Sqrt(1.0f - Y * Y);

                const float GoldenAngle = PI * (3.0f - FMath::Sqrt(5.0f));
                const float Theta = GoldenAngle * i;

                const float X = FMath::Cos(Theta) * RadiusAtY;
                const float Z = FMath::Sin(Theta) * RadiusAtY;

                const FVector SampleDir(X, Y, Z);

                const FIntVector VoxelIndexStart = VoxelVolume->WorldToVoxelIndex(PositionA);
                const FIntVector VoxelIndexEnd = VoxelVolume->WorldToVoxelIndex(PositionA + SampleDir * LookaheadDist);

                // 복셀 단위로 레이캐스트를 수행하여 가장 가까운 장애물 복셀을 탐색
                FIntVector HitVoxelIndex;
                if (FStrategyUtils::Raycast(VoxelVolume, VoxelIndexStart, VoxelIndexEnd, HitVoxelIndex))
                {
                    // 감지된 장애물 복셀을 "움직이지 않는 에이전트"로 간주
                    const FVector ObstaclePos = VoxelVolume->VoxelIndexToWorld(HitVoxelIndex);
                    const FVector ObstacleVel = FVector::ZeroVector;

                    const FVector RelativePos = ObstaclePos - PositionA;
                    const FVector RelativeVel = CurrentVelocityA - ObstacleVel;

                    // 정적 장애물에 대한 u 벡터 계산
                    const float InverseTimeHorizon = 1.0f / TimeHorizon;
                    const FVector w = RelativeVel - RelativePos * InverseTimeHorizon;
                    const float w_len = w.Size();
                    FVector u = FVector::ZeroVector;
                    if (w_len > KINDA_SMALL_NUMBER)
                    {
                        // 정적 장애물은 반지름이 없다고 가정하고 계산을 단순화
                        // 이 부분은 에이전트가 장애물에 너무 가까이 붙지 않도록 하는 역할
                        const float DotProduct = FVector::DotProduct(RelativePos, w);
                        if (DotProduct < 0.0f && FMath::Square(DotProduct) > w_len * w_len * FMath::Square(RadiusA))
                        {
                            u = (w / w_len) * (RadiusA * InverseTimeHorizon - w_len);
                        }
                    }

                    if (!u.IsNearlyZero())
                    {
                        // 정적 장애물은 회피 책임이 0%이므로, 에이전트가 100% 책임
                        const FVector w_correction = u;
                        const FVector n_correction = u.GetSafeNormal();
                        if (!n_correction.IsNearlyZero())
                        {
                            OrcaConstraints.Emplace(CurrentVelocityA + w_correction, n_correction);
                        }
                    }
                }
            }
        }

		// Begin Debug Visualization
        if (GEngine)
        {
            UWorld* World = GetWorld();
            if (World)
            {
                const FVector LineStart = PositionA + FVector(0, 0, 10.f);
                const float LineLengthMultiplier = 0.2f;

                // 1. 선호 속도 (Preferred Velocity) - 파란색
                DrawDebugLine(
                    World,
                    LineStart,
                    LineStart + PreferredVelocityA * LineLengthMultiplier,
                    FColor::Blue,
                    false, -1, 0, 2.f
                );

                // 2. 수정된 선호 속도 (Actual Preferred Velocity) - 노란색
                DrawDebugLine(
                    World,
                    LineStart,
                    LineStart + ActualPreferredVelocity * LineLengthMultiplier,
                    FColor::Yellow,
                    false, -1, 0, 3.f
                );

                // 3. 최종 최적 속도 (Optimal Velocity) - 초록색
                DrawDebugLine(
                    World,
                    LineStart,
                    LineStart + OptimalVelocity * LineLengthMultiplier,
                    FColor::Green,
                    false, -1, 0, 4.f
                );
            }
        }
		// End Debug Visualization

        NewOptimalVelocities.Add(AgentA, OptimalVelocity);
    }

    for (auto const& [Agent, OptimalVelocity] : NewOptimalVelocities)
    {
        if (Agent)
        {
            // VInterpTo 사용시 기존 보간 로직과 충돌 가능성 + 매직 넘버 탐색
            FVector FinalVelocity = OptimalVelocity;
            Agent->ApplyMovement(FinalVelocity.GetClampedToMaxSize(Agent->MovementSpeed), DeltaTime);
        }
    }
}

FVector UAvoidanceSubsystem::SolveOptimalVelocity_Final(const TArray<FPlane>& Constraints, const FVector& PreferredVelocity) const
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
                // 위반한 깊이만큼 노말 방향으로 밀어냄
                // 보정 강도로 1.01f를 사용할 수 있음.
                NewVelocity -= (FVector)Constraint * ViolationDepth;
                bViolated = true;
            }
        }
        if (!bViolated)
        {
            break;
        }
    }

    return NewVelocity;
}