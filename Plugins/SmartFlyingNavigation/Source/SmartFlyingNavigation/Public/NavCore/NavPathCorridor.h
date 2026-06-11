// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

struct FNavPathPortal
{
	int32 FromCell = -1;
	int32 ToCell = -1;
	FVector Center = FVector::ZeroVector;
	FVector Normal = FVector::ZeroVector;
	FVector ExtentU = FVector::ZeroVector;
	FVector ExtentV = FVector::ZeroVector;
	float Clearance = 0.f;
};

struct FNavPathCorridor
{
	TArray<int32> CellPath;
	TArray<FNavPathPortal> Portals;
	TArray<FVector> Waypoints;
	float RequiredAgentRadius = 0.f;
	bool bDynamicValidated = false;

	void Reset()
	{
		CellPath.Reset();
		Portals.Reset();
		Waypoints.Reset();
		RequiredAgentRadius = 0.f;
		bDynamicValidated = false;
	}

	bool IsValid() const
	{
		return Waypoints.Num() >= 2 && RequiredAgentRadius > 0.f;
	}
};
