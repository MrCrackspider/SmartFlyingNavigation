#pragma once
#include "CoreMinimal.h"
#include "AStarStrategyUtils.generated.h"

// --- 전방 선언 ---
struct FPathfindingRequest;
struct FOctreePathfindingData;

// =======================================================================
// 1. 옥트리 전용 FPathNode 정의
// =======================================================================
USTRUCT()
struct FAstarPathNode
{
    GENERATED_BODY()

    // --- 옥트리용 멤버 변수 ---
    int32 NodeIndex = -1;       // 옥트리 노드의 고유 ID (Key로 사용)
    FVector WorldPosition;      // 노드의 월드 중심점 (비용 계산용)

    // --- A* 공통 멤버 변수 ---
    int64 GCost = 0;            // 시작점으로부터의 실제 비용
    int64 HCost = 0;            // 목적지까지의 예상 비용 (휴리스틱)
    int64 FCost = 0;            // GCost + HCost
    FAstarPathNode* Parent = nullptr; // 경로 추적을 위한 부모 노드

    // 기본 생성자
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

template<typename T>
struct TPathNodePtrCompare
{
    // 버전 1: 실제 객체를 비교하는 로직
    bool operator()(const T& A, const T& B) const
    {
        if (A.FCost != B.FCost)
        {
            return A.FCost < B.FCost;
        }
        return A.HCost < B.HCost;
    }

    // 버전 2: 포인터를 받아 역참조하여 버전 1을 호출하는 오버로딩
    // 이것이 C2665 에러를 해결하는 핵심입니다.
    bool operator()(const T* A, const T* B) const
    {
        // 널 포인터 방지
        if (!A) return false;
        if (!B) return true;
        // 실제 객체 비교 로직 재사용
        return (*this)(*A, *B);
    }
};