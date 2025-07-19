// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once
#include "CoreMinimal.h"
#include "AStarStrategyUtils.generated.h"

struct FPathfindingRequest;
struct FOctreePathfindingData;

USTRUCT()
struct FAstarPathNode
{
	GENERATED_BODY()

	int32 NodeIndex = -1;
	FVector WorldPosition;

	int64 GCost = 0;
	int64 HCost = 0;
	int64 FCost = 0;
	FAstarPathNode* Parent = nullptr;

	FAstarPathNode() = default;

	void CalculateAndSetFCost()
	{
		FCost = GCost + HCost;
	}

	void SetParent(FAstarPathNode* InParent, int64 NewGCost, int64 NewHCost)
	{
		Parent = InParent;
		GCost = NewGCost;
		HCost = NewHCost;
		CalculateAndSetFCost();
	}
};

template <typename T>
struct TPathNodePtrCompare
{
	bool operator()(const T& A, const T& B) const
	{
		if (A.FCost != B.FCost)
		{
			return A.FCost < B.FCost;
		}
		return A.HCost < B.HCost;
	}

	bool operator()(const T* A, const T* B) const
	{
		if (!A) return false;
		if (!B) return true;
		return (*this)(*A, *B);
	}
};

namespace AStarStrategyUtils
{
	static FBox MakeAgentQueryBox(const FVector& Center, float Radius)
	{
		return FBox(Center - FVector(Radius), Center + FVector(Radius));
	}
}

