// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "NavCore/NavNodeTypes.h"
#include "Strategy/PathfindingStrategy.h"
#include "AirwayGraphStrategy.generated.h"

struct FPathfindingRequest;

UCLASS()
class SMARTFLYINGNAVIGATION_API UAirwayGraphStrategy : public UPathfindingStrategy
{
	GENERATED_BODY()

public:
	virtual void Initialize() override;
	virtual bool FindPathWithStrategy(const FPathfindingRequest& InRequest, TArray<FVector>& OutPath) override;
	virtual FName GetStrategyName() const override;

private:
	int32 FindNearestUsableNode(const FAirwayGraph& Graph, const FVector& Location, float AgentRadius) const;
	void ReconstructPath(const TArray<int32>& CameFrom, int32 StartNode, int32 GoalNode, const FAirwayGraph& Graph, const FVector& StartLocation, const FVector& EndLocation, TArray<FVector>& OutPath) const;
};
