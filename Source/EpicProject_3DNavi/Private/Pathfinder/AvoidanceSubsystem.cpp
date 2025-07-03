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

    TimeHorizon = 1.f;
    MaxNeighbors = 4;
    GridCellSize = 200.f; // VolumeExtent의 배수이자 (64의 약수)*100로 하는게 좋음
    SymmetryBreakingRangeMultiplier = 4.f;
    SolverIterations = 5;
    SideBiasMagnitude = 0.8f;
    InterpSpeed = 10.f;
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
    const float Radius = Agent->GetAgentRadius();

    // 에이전트의 반지름이 그리드 셀 크기를 초과할 경우를 대비해 탐색 범위를 동적으로 계산
    const int32 SearchRange = FMath::Max(1, FMath::CeilToInt(Radius / GridCellSize));

    const FIntVector CenterGridPosition(FMath::FloorToInt(Location.X / GridCellSize), FMath::FloorToInt(Location.Y / GridCellSize), FMath::FloorToInt(Location.Z / GridCellSize));

    for (int32 z = -SearchRange; z <= SearchRange; ++z)
    {
        for (int32 y = -SearchRange; y <= SearchRange; ++y)
        {
            for (int32 x = -SearchRange; x <= SearchRange; ++x)
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

    // --- 1. 초기 조건 검사 ---
    if (RegisteredAgents.Num() < 2)
    {
        if (RegisteredAgents.Num() == 1 && RegisteredAgents[0])
        {
            RegisteredAgents[0]->ApplyMovement(RegisteredAgents[0]->GetPreferredVelocity(), DeltaTime);
        }
        return;
    }
    if (DeltaTime < KINDA_SMALL_NUMBER) return;

    // --- 2. 공간 분할 그리드 업데이트 ---
    UpdateAgentGrid();

    TMap<UPathfindingComponent*, FVector> NewOptimalVelocities;

    // --- 3. 각 에이전트에 대한 최적 속도 계산 ---
    for (UPathfindingComponent* Agent : RegisteredAgents)
    {
        if (!Agent || !Agent->GetOwner()) continue;

        // 주변 에이전트 탐색 및 필터링
        TArray<UPathfindingComponent*> NearbyAgents = GetNearbyAgents(Agent);
        if (NearbyAgents.Num() > MaxNeighbors)
        {
            NearbyAgents.Sort([AgentPos = Agent->GetOwner()->GetActorLocation()](const UPathfindingComponent& Lhs, const UPathfindingComponent& Rhs) {
                return FVector::DistSquared(AgentPos, Lhs.GetOwner()->GetActorLocation()) < FVector::DistSquared(AgentPos, Rhs.GetOwner()->GetActorLocation());
                });
            NearbyAgents.SetNum(MaxNeighbors);
        }

        TArray<FPlane> OrcaConstraints;

        // Step 3a: 다른 에이전트와의 제약 조건 생성
        GenerateAgentConstraints(Agent, NearbyAgents, OrcaConstraints);

        // Step 3b: 정적 복셀 장애물과의 제약 조건 생성
        GenerateVoxelConstraints(Agent, OrcaConstraints);

        // Step 3c: 대칭성 파괴를 위한 선호 속도 조정
        const FVector PreferredVelocity = Agent->GetPreferredVelocity();
        const FVector ActualPreferredVelocity = CalculateSymmetryBreakingVelocity(Agent, PreferredVelocity, NearbyAgents);

        // Step 3d: 제약 조건들을 기반으로 최적 속도 해결
        const FVector OptimalVelocity = SolveOptimalVelocity(OrcaConstraints, ActualPreferredVelocity);

        // Step 3e: 디버그 시각화
        DebugVisualizeVelocities(Agent, PreferredVelocity, ActualPreferredVelocity, OptimalVelocity);

        NewOptimalVelocities.Add(Agent, OptimalVelocity);
    }

    // --- 4. 계산된 속도를 모든 에이전트에 적용 ---
    for (auto const& [Agent, OptimalVelocity] : NewOptimalVelocities)
    {
        if (Agent)
        {
            const FVector CurrentVelocity = Agent->GetCurrentVelocity();
            const FVector InterpolatedVelocity = FMath::VInterpTo(CurrentVelocity, OptimalVelocity, DeltaTime, InterpSpeed);
            const FVector FinalVelocity = InterpolatedVelocity.GetClampedToMaxSize(Agent->MovementSpeed);

            Agent->ApplyMovement(FinalVelocity, DeltaTime);
        }
    }
}

void UAvoidanceSubsystem::GenerateAgentConstraints(UPathfindingComponent* CurrentAgent, const TArray<UPathfindingComponent*>& NearbyAgents, TArray<FPlane>& OutConstraints)
{
    const FVector PositionA = CurrentAgent->GetOwner()->GetActorLocation();
    const float RadiusA = CurrentAgent->GetAgentRadius();
    const FVector CurrentVelocityA = CurrentAgent->GetCurrentVelocity();

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
        bool bIsCollidingOrApproaching = false;

        // TTC (Time to Collision) 계산
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

        if (DistSq > CombinedRadiusSq && !bIsCollidingOrApproaching)
        {
            continue;
        }

        const float InverseEffectiveTimeHorizon = 1.0f / EffectiveTimeHorizon;
        FVector u;

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
            // TO-DO: 아래 코드를 주석 처리해서 Reciprocal Dancing 현상을 제거했지만, 겹침 해소를 할 수 없음.
            // @minseokbae
            // const float invDeltaTime = 1.0f / GetWorld()->GetDeltaSeconds();
            // u = (RelativePosition.GetSafeNormal() * (CombinedRadius - RelativePosition.Size()) * invDeltaTime) - RelativeVelocity;
        }

        if (!u.IsNearlyZero(KINDA_SMALL_NUMBER))
        {
            const FVector w_correction = u * 0.5f;
            const FVector n_correction = u.GetSafeNormal();
            if (!n_correction.IsNearlyZero(KINDA_SMALL_NUMBER))
            {
                OutConstraints.Emplace(CurrentVelocityA + w_correction, n_correction);
            }
        }
    }
}

void UAvoidanceSubsystem::GenerateVoxelConstraints(UPathfindingComponent* CurrentAgent, TArray<FPlane>& OutConstraints)
{
    AFenwickVoxelVolume* VoxelVolume = Cast<AFenwickVoxelVolume>(CurrentAgent->TargetVolume);
    if (!VoxelVolume) return;

    const FVector PositionA = CurrentAgent->GetOwner()->GetActorLocation();
    const float RadiusA = CurrentAgent->GetAgentRadius();
    const FVector CurrentVelocityA = CurrentAgent->GetCurrentVelocity();

    const int32 NumSamples = 16;
    const float LookaheadDist = RadiusA * 2.0f;

    for (int32 i = 0; i < NumSamples; ++i)
    {
        // Fibonacci Sphere 샘플링
        const float Y = 1.0f - (i / (float)(NumSamples - 1)) * 2.0f;
        const float RadiusAtY = FMath::Sqrt(1.0f - Y * Y);
        const float GoldenAngle = PI * (3.0f - FMath::Sqrt(5.0f));
        const float Theta = GoldenAngle * i;
        const FVector SampleDir(FMath::Cos(Theta) * RadiusAtY, Y, FMath::Sin(Theta) * RadiusAtY);

        const FIntVector VoxelIndexStart = VoxelVolume->WorldToVoxelIndex(PositionA);
        const FIntVector VoxelIndexEnd = VoxelVolume->WorldToVoxelIndex(PositionA + SampleDir * LookaheadDist);

        FIntVector HitVoxelIndex;
        if (FStrategyUtils::Raycast(VoxelVolume, VoxelIndexStart, VoxelIndexEnd, HitVoxelIndex))
        {
            // 정적 장애물은 점이 아닌 부피로 계산
            const FVector VoxelWorldPos = VoxelVolume->VoxelIndexToWorld(HitVoxelIndex);
            // VoxelVolume->VoxelExtent가 복셀 한 변 길이의 절반(half extent)이라고 가정
            const FBox VoxelBounds(VoxelWorldPos - VoxelVolume->VoxelExtent, VoxelWorldPos + VoxelVolume->VoxelExtent);
            const FVector ObstaclePos = VoxelBounds.GetClosestPointTo(PositionA);

            const FVector RelativePos = ObstaclePos - PositionA;
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

FVector UAvoidanceSubsystem::CalculateSymmetryBreakingVelocity(UPathfindingComponent* CurrentAgent, const FVector& PreferredVelocity, const TArray<UPathfindingComponent*>& NearbyAgents)
{
    if (NearbyAgents.IsEmpty() || PreferredVelocity.IsNearlyZero())
    {
        return PreferredVelocity;
    }

    const FVector PositionA = CurrentAgent->GetOwner()->GetActorLocation();
    const float RadiusA = CurrentAgent->GetAgentRadius();

    UPathfindingComponent* ObstacleAgent = nullptr;
    float MinThreatDistSq = TNumericLimits<float>::Max();

    for (UPathfindingComponent* AgentB : NearbyAgents)
    {
        const FVector ToAgentB = AgentB->GetOwner()->GetActorLocation() - PositionA;
        const float DistToAgentBSq = ToAgentB.SizeSquared();
        const float CombinedRadius = RadiusA + AgentB->GetAgentRadius();

        if (DistToAgentBSq < FMath::Square(CombinedRadius * SymmetryBreakingRangeMultiplier) && FVector::DotProduct(PreferredVelocity, ToAgentB) > 0)
        {
            if (DistToAgentBSq < MinThreatDistSq)
            {
                MinThreatDistSq = DistToAgentBSq;
                ObstacleAgent = AgentB;
            }
        }
    }

    if (ObstacleAgent && CurrentAgent->GetUniqueID() > ObstacleAgent->GetUniqueID())
    {
        FVector ForwardVec = PreferredVelocity.GetSafeNormal();
        FVector SideVector;

        if (FMath::Abs(FVector::DotProduct(ForwardVec, FVector::UpVector)) > 0.99f)
        {
            SideVector = FVector::CrossProduct(ForwardVec, FVector::RightVector);
        }
        else
        {
            SideVector = FVector::CrossProduct(ForwardVec, FVector::UpVector);
        }
        SideVector.Normalize();

        FVector ActualPreferredVelocity = PreferredVelocity + SideVector * CurrentAgent->MovementSpeed * SideBiasMagnitude;
        return ActualPreferredVelocity.GetSafeNormal() * CurrentAgent->MovementSpeed;
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

void UAvoidanceSubsystem::DebugVisualizeVelocities(const UPathfindingComponent* Agent, const FVector& PreferredVelocity, const FVector& ActualPreferredVelocity, const FVector& OptimalVelocity) const
{
    if (!GEngine) return;

    UWorld* World = GetWorld();
    if (!World) return;

    const FVector LineStart = Agent->GetOwner()->GetActorLocation() + FVector(0, 0, 10.f);
    const float LineLengthMultiplier = 0.2f;

    // 1. 선호 속도 (Preferred Velocity) - 파란색
    DrawDebugLine(World, LineStart, LineStart + PreferredVelocity * LineLengthMultiplier, FColor::Blue, false, -1, 0, 2.f);
    // 2. 수정된 선호 속도 (Actual Preferred Velocity) - 노란색
    DrawDebugLine(World, LineStart, LineStart + ActualPreferredVelocity * LineLengthMultiplier, FColor::Yellow, false, -1, 0, 3.f);
    // 3. 최종 최적 속도 (Optimal Velocity) - 초록색
    DrawDebugLine(World, LineStart, LineStart + OptimalVelocity * LineLengthMultiplier, FColor::Green, false, -1, 0, 4.f);
}