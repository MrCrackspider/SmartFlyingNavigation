// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "StrategyUtils.generated.h"


class AFenwickVoxelVolume;

constexpr int32 COST_STRAIGHT = 1000;
constexpr int32 COST_DIAGONAL_PLANE = 1414;
constexpr int32 COST_DIAGONAL_SPACE = 1732;

USTRUCT()
struct FPathNode
{
    GENERATED_BODY()
    FIntVector Position;
    FPathNode* Parent = nullptr;
    uint32 GCost = 0;
    uint32 HCost = 0;
    uint32 FCost = 0;

    FIntVector Direction; // 부모로부터 이 노드까지의 이동 방향
    bool bIsJumpPoint = false; // 이 노드가 점프 포인트인지 여부
    
    FPathNode() = default;

    explicit FPathNode(const FIntVector& InPos)
     : Position(InPos), Parent(nullptr), GCost(0), HCost(0), FCost(0), Direction(FIntVector::ZeroValue), bIsJumpPoint(false)
    {
    }

    void CalculateAndSetFCost()
    {
        FCost = GCost + HCost;
    }

    void SetParent(FPathNode*& InParent, int64  NewGCost, int64  NewHCost)
    {
        Parent = InParent;
        GCost = NewGCost;
        HCost = NewHCost;
        CalculateAndSetFCost();
        
        // 부모가 설정되면, 현재 노드까지의 방향을 업데이트합니다.
        if (Parent)
        {
            Direction = Position - Parent->Position; // 부모로부터 현재까지의 방향 벡터
        }
        else
        {
            Direction = FIntVector::ZeroValue; // 부모가 없으면 방향 0
        }
    }
};
struct FPathNodePtrCompare
{
    // ★★★ 버전 1: 기존의 포인터 비교 함수 (그대로 둡니다) ★★★
    bool operator()(const FPathNode& A, const FPathNode& B) const
    {
        // F-Cost가 낮은 노드가 우선순위가 높습니다 (힙의 top에 위치).
        // F-Cost가 같다면 H-Cost가 낮은 노드를 우선합니다.
        if (A.FCost != B.FCost)
        {
            return A.FCost < B.FCost;
        }
        return A.HCost < B.HCost;
    }

    // ★★★ 버전 2: 포인터를 받는 경우를 위한 오버로딩 ★★★
    // TArray::HeapPush/Pop은 내부적으로 포인터를 역참조하여 
    // 위 버전 1의 비교 함수를 호출합니다.
    // 따라서 이 함수는 아래와 같이 포인터를 받아서 역참조하여
    // 버전 1의 비교를 수행하도록 만들어야 합니다.
    bool operator()(const FPathNode* A, const FPathNode* B) const
    {
        // 널 포인터 체크는 안전을 위해 추가합니다.
        if (!A || !B)
        {
            return false;
        }
        // 포인터를 역참조(*)하여 실제 객체 비교 로직을 재사용합니다.
        return (*this)(*A, *B);
    }
};

USTRUCT()
struct FStrategyUtils
{
    GENERATED_BODY()


    static int32 Heuristic_Octile(const FIntVector& A, const FIntVector& B)
    {
        const int32 dx = FMath::Abs(A.X - B.X);
        const int32 dy = FMath::Abs(A.Y - B.Y);
        const int32 dz = FMath::Abs(A.Z - B.Z);

        const int32 d_max = FMath::Max3(dx, dy, dz);
        const int32 d_min = FMath::Min3(dx, dy, dz);
        const int32 d_mid = (dx + dy + dz) - d_max - d_min;

        const int32 Cost =
            (COST_DIAGONAL_SPACE * d_min) +
            (COST_DIAGONAL_PLANE * (d_mid - d_min)) +
            (COST_STRAIGHT * (d_max - d_mid));

        return Cost;
    }

    static int32 Heuristic_Manhattan(const FIntVector& A, const FIntVector& B)
    {
        return FMath::Abs(A.X - B.X) + FMath::Abs(A.Y - B.Y) + FMath::Abs(A.Z - B.Z);
    }

    static FIntVector GetNormalizedDirection(const FIntVector& From, const FIntVector& To)
    {
        FIntVector Dir;
        Dir.X = (To.X > From.X) ? 1 : ((To.X < From.X) ? -1 : 0);
        Dir.Y = (To.Y > From.Y) ? 1 : ((To.Y < From.Y) ? -1 : 0);
        Dir.Z = (To.Z > From.Z) ? 1 : ((To.Z < From.Z) ? -1 : 0);

        return Dir;
    }


    static bool Raycast(AFenwickVoxelVolume* Volume, const FIntVector& StartIndex, const FIntVector& EndIndex, FIntVector& OutHitIndex)
    {
        FIntVector CurrentIndex = StartIndex;
        FVector Direction = FVector(EndIndex - StartIndex);
        if (Direction.IsNearlyZero()) return false;

        FIntVector Step(
            (Direction.X > 0) ? 1 : -1,
            (Direction.Y > 0) ? 1 : -1,
            (Direction.Z > 0) ? 1 : -1
        );

        Direction.Normalize();
        FVector DeltaDist(
            FMath::Abs(1.0f / Direction.X),
            FMath::Abs(1.0f / Direction.Y),
            FMath::Abs(1.0f / Direction.Z)
        );

        FVector SideDist;
        if (Direction.X > 0) SideDist.X = (StartIndex.X + 1 - (float)StartIndex.X) * DeltaDist.X;
        else SideDist.X = ((float)StartIndex.X - StartIndex.X) * DeltaDist.X;
        if (Direction.Y > 0) SideDist.Y = (StartIndex.Y + 1 - (float)StartIndex.Y) * DeltaDist.Y;
        else SideDist.Y = ((float)StartIndex.Y - StartIndex.Y) * DeltaDist.Y;
        if (Direction.Z > 0) SideDist.Z = (StartIndex.Z + 1 - (float)StartIndex.Z) * DeltaDist.Z;
        else SideDist.Z = ((float)StartIndex.Z - StartIndex.Z) * DeltaDist.Z;

        while (Volume->IsIndexValid(CurrentIndex))
        {
            if (!Volume->IsVoxelWalkable(CurrentIndex))
            {
                OutHitIndex = CurrentIndex;
                return true;
            }

            if (SideDist.X < SideDist.Y && SideDist.X < SideDist.Z)
            {
                SideDist.X += DeltaDist.X;
                CurrentIndex.X += Step.X;
            }
            else if (SideDist.Y < SideDist.Z)
            {
                SideDist.Y += DeltaDist.Y;
                CurrentIndex.Y += Step.Y;
            }
            else
            {
                SideDist.Z += DeltaDist.Z;
                CurrentIndex.Z += Step.Z;
            }

            if (CurrentIndex == EndIndex) break;
        }

        return false;
    }
};