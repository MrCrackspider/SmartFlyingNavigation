// Copyright © 2025 TeamHamDev. All Rights Reserved.

#include "Utility/CatmullRomSpline.h"

FCatmullRomSpline::FCatmullRomSpline(const TArray<FVector>& FoundPath)
{
	Initialize(FoundPath);
}

void FCatmullRomSpline::Initialize(const TArray<FVector>& FoundPath)
{
	ControlPoints = FoundPath;
	DistanceOfPoints.Empty(); // 배열 초기화

	if (ControlPoints.Num() < 2)
	{
		return;
	}

	if (ControlPoints.Num() == 2)
	{
		DistanceOfPoints.Add(0.0f);
		DistanceOfPoints.Add(FVector::Distance(ControlPoints[0], ControlPoints[1]));
	}
	else
	{
		DistanceOfPoints.Add(0.0f);

		float AccumulatedDistance = 0.0f;
		const int32 SegmentsToApproximate = 20;

		for (int32 i = 0; i < ControlPoints.Num() - 1; ++i)
		{
			FVector P0, P1, P2, P3;
			GetCatmullRomPointsForSegment(i, P0, P1, P2, P3);

			float SegmentArcLength = 0.0f;
			FVector LastPoint = P1;

			for (int32 j = 1; j <= SegmentsToApproximate; ++j)
			{
				const float Alpha = static_cast<float>(j) / static_cast<float>(SegmentsToApproximate);
				const FVector CurrentPoint = FMath::CubicInterp(P0, P1, P2, P3, Alpha);
				SegmentArcLength += FVector::Distance(LastPoint, CurrentPoint);
				LastPoint = CurrentPoint;
			}

			AccumulatedDistance += SegmentArcLength;
			DistanceOfPoints.Add(AccumulatedDistance);
		}
	}
}

float FCatmullRomSpline::GetSplineLength() const
{
	if (DistanceOfPoints.Num() < 1)
	{
		return 0.f;
	}
	
	return DistanceOfPoints.Last();
}

float FCatmullRomSpline::GetDistanceByIndex(int32 PointIndex) const
{
	if (DistanceOfPoints.Num() < 1)
	{
		return 0.f;
	}

	return DistanceOfPoints[PointIndex];
}

FVector FCatmullRomSpline::GetPointByIndex(int32 PointIndex) const
{
	if (ControlPoints.Num() < 1)
	{
		return FVector::ZeroVector;
	}
	
	return ControlPoints[PointIndex];
}

FVector FCatmullRomSpline::GetPositionByDistance(float Distance) const
{
    if (ControlPoints.Num() < 2)
    {
        return ControlPoints.IsEmpty() ? FVector::ZeroVector : ControlPoints[0];
    }
    
    const TOptional<ReturnByDistance> Result = GetAlphaByDistance(Distance);

	if (!Result.IsSet())
	{
		return ControlPoints.IsEmpty() ? FVector::ZeroVector : ControlPoints[0];
	}

	ReturnByDistance RealResult = Result.GetValue();
	
    const int32 CurrentSegmentIndex = RealResult.Point;
    const float Alpha = RealResult.Alpha;

	// UE_LOG(LogTemp, Error, TEXT("GetPoistionByDistance - Distance: %f, CurrentIndex: %d, Alpha: %f"), Distance, CurrentSegmentIndex, Alpha);
	
	FVector P0, P1, P2, P3;

	GetCatmullRomPointsForSegment(CurrentSegmentIndex, P0, P1, P2, P3);
	
	UE_LOG(LogTemp, Error, TEXT("Distance: %f, P0 : %f %f %f, P1 : %f %f %f, P2 : %f %f %f, P3 : %f %f %f"),
		Distance, P0.X, P0.Y, P0.Z, P1.X, P1.Y, P1.Z, P2.X, P2.Y, P2.Z, P3.X, P3.Y, P3.Z);

    return FMath::CubicInterp(P0, P1, P2, P3, Alpha);
}

FVector FCatmullRomSpline::GetDirectionOnSpline(float Distance) const
{
    if (ControlPoints.Num() < 2)
    {
        return FVector::ZeroVector;
    }

    const TOptional<ReturnByDistance> Result = GetAlphaByDistance(Distance);

	if (!Result.IsSet())
	{
		return FVector::ZeroVector;
	}

	ReturnByDistance RealResult = Result.GetValue();
	
    const int32 CurrentSegmentIndex = RealResult.Point;
    const float Alpha = RealResult.Alpha;

    FVector P0, P1, P2, P3;

	GetCatmullRomPointsForSegment(CurrentSegmentIndex, P0, P1, P2, P3);

    const FVector Tangent = FMath::CubicInterpDerivative(P0, P1, P2, P3, Alpha);

    return Tangent.GetSafeNormal();
}

TOptional<ReturnByDistance> FCatmullRomSpline::GetAlphaByDistance(float CurrentDistance) const
{
	ReturnByDistance ReturnValue;

	const int32 NumPoints = DistanceOfPoints.Num();

	if (NumPoints < 2)
	{
		return {};
	}

	if (CurrentDistance < 0.f)
	{
		return {};
	}

	const float TotalLength = GetSplineLength();
	if (CurrentDistance >= TotalLength)
	{
		//return Previous Point of Goal Point And Set Alpha 1.0f
		ReturnValue.Point = NumPoints - 2;
		ReturnValue.Alpha = 1.0f;         
		return ReturnValue;
	}
	
	const int32 EndPointIndex = Algo::UpperBound(DistanceOfPoints, CurrentDistance);

	const int32 StartPointIndex = EndPointIndex - 1;
	
	ReturnValue.Point = StartPointIndex;

	const float SegmentStartDistance = DistanceOfPoints[StartPointIndex];
	const float SegmentEndDistance = DistanceOfPoints[EndPointIndex];
	const float SegmentLength = SegmentEndDistance - SegmentStartDistance;

	if (FMath::IsNearlyZero(SegmentLength))
	{
		ReturnValue.Alpha = 0.0f;
	}
	else
	{
		const float DistanceIntoSegment = CurrentDistance - SegmentStartDistance;
		ReturnValue.Alpha = DistanceIntoSegment / SegmentLength;
	}

	return ReturnValue;
}

void FCatmullRomSpline::GetCatmullRomPointsForSegment(int32 CurrentSegmentIndex, FVector& OutP0, FVector& OutP1, FVector& OutP2, FVector& OutP3) const
{
	const int32 NumPoints = ControlPoints.Num();

	// 제어점이 2개 미만일 경우 의미가 없으므로 조기 리턴
	if (NumPoints < 2)
	{
		return;
	}
    
	// 세그먼트의 시작점과 끝점은 항상 고정입니다.
	OutP1 = ControlPoints[CurrentSegmentIndex];
	OutP2 = ControlPoints[CurrentSegmentIndex + 1];

	// 이전 점(P0) 설정: 첫 번째 세그먼트일 경우 시작점을 중복 사용 (Clamped)
	OutP0 = (CurrentSegmentIndex > 0) ? ControlPoints[CurrentSegmentIndex - 1] : ControlPoints[CurrentSegmentIndex];

	// 다음 점(P3) 설정: 마지막 세그먼트일 경우 끝점을 중복 사용 (Clamped)
	OutP3 = (CurrentSegmentIndex < NumPoints - 2) ? ControlPoints[CurrentSegmentIndex + 2] : ControlPoints[CurrentSegmentIndex + 1];
}