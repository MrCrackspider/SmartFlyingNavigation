// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "StrategyUtils.generated.h"

constexpr int32 COST_STRAIGHT = 1000;
constexpr int32 COST_DIAGONAL_PLANE = 1414;
constexpr int32 COST_DIAGONAL_SPACE = 1732;

USTRUCT()
struct FPathNode
{
    GENERATED_BODY()

    FIntVector Position;
    TSharedPtr<FPathNode> Parent;
    int64  GCost = 0;
    int64  HCost = 0;
    int64  FCost = 0;

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

    void SetParent(const TSharedPtr<FPathNode>& InParent, int64  NewGCost, int64  NewHCost)
    {
        Parent = InParent;
        GCost = NewGCost;
        HCost = NewHCost;
        CalculateAndSetFCost();
        
        // 부모가 설정되면, 현재 노드까지의 방향을 업데이트합니다.
        if (Parent.IsValid())
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
    FORCEINLINE bool operator()(const TSharedPtr<FPathNode>& A, const TSharedPtr<FPathNode>& B) const
    {
        return A->FCost < B->FCost;
        // FIX-ME: Minheap�̶� A->FCost > B->FCost�� �Ǵ°� ������, �޸���ƽ�Լ��� �� �� ���� ������ �����Ǿ�� �������� ��ΰ� ����.
        //return A->FCost > B->FCost;
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
};