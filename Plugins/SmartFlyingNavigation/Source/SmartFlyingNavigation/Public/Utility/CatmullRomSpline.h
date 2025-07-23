// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "CatmullRomSpline.generated.h"

struct ReturnByDistance
{
	int Point;
	float Alpha;
};

USTRUCT()
struct FCatmullRomSpline
{
	GENERATED_BODY()

	FCatmullRomSpline() = default;
	FCatmullRomSpline(const TArray<FVector>& FoundPath);

	void Initialize(const TArray<FVector>& FoundPath);
	
	float GetSplineLength() const;
	float GetDistanceByIndex(int32 PointIndex) const;
	FVector GetPointByIndex(int32 PointIndex) const;

	FVector GetPositionByDistance(float Distance) const;
	FVector GetDirectionOnSpline(float Distance) const;
	
private:
	TOptional<ReturnByDistance> GetAlphaByDistance(float CurrentDistance) const;
	void GetCatmullRomPointsForSegment(int32 CurrentIndex, FVector& OutP0, FVector& OutP1, FVector& OutP2, FVector& OutP3) const;

private:
	TArray<FVector> ControlPoints;
	TArray<float> DistanceOfPoints;
};
