// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Strategy/PathfindingStrategy.h"
#include "AStarStrategy.generated.h"

struct FPathfindingRequest;

UCLASS()
class SMARTFLYINGNAVIGATION_API UAStarStrategy : public UPathfindingStrategy
{
	GENERATED_BODY()

public:
	virtual void Initialize() override;
	virtual bool FindPathWithStrategy(const FPathfindingRequest& InRequest, TArray<FVector>& OutPath) override;

	virtual bool ShouldRepathForOptimization(const FIntVector& CurrentAgentVoxel,
	                                         const TArray<FIntVector>& CurrentPathVoxels,
	                                         int32 CurrentPathIndex,
	                                         const FIntVector& GoalVoxel,
	                                         uint32 AgentRadiusVoxels) const override;

	virtual FName GetStrategyName() const override;

public:
	UPROPERTY(EditDefaultsOnly, Category = "A* | Fallback Search", meta=(ClampMin="50.0"))
	float NearbyGoalSearchRadius = 2000.f;

	UPROPERTY(EditDefaultsOnly, Category = "A* | Fallback Search", meta=(ClampMin="1"))
	int32 NearbyGoalSearchIterations = 100;

	/**
	 * Extra cost multiplier applied to moves that hug obstacles (0 disables).
	 * Requires a baked distance field; with weight W, a move with zero clearance
	 * costs up to (1 + W) times its distance.
	 */
	UPROPERTY(EditDefaultsOnly, Category = "A* | Clearance", meta=(ClampMin="0.0", ClampMax="10.0"))
	float ClearancePenaltyWeight = 1.0f;

	/** Clearance at or above AgentRadius * this multiplier incurs no penalty. */
	UPROPERTY(EditDefaultsOnly, Category = "A* | Clearance", meta=(ClampMin="1.0", ClampMax="10.0"))
	float PreferredClearanceMultiplier = 2.0f;
};
