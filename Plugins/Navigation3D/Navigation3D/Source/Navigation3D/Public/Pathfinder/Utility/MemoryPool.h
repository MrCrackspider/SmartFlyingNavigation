// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Pathfinder/Utility/AStarStrategyUtils.h"
#include "Pathfinder/Utility/StrategyUtils.h"


UENUM(BlueprintType)
enum class EPathfindingNodeType : uint8
{
    Grid,
    Octree
};

struct FPathNodeMemoryPoolBase
{
    virtual ~FPathNodeMemoryPoolBase() {}
    virtual void Reset() = 0;
    virtual EPathfindingNodeType GetNodeType() const = 0;
};

template<typename T>
struct TNodeMemoryPool : public FPathNodeMemoryPoolBase
{
public:
    explicit TNodeMemoryPool(int32 InSlabSize = 1024)
        : InitialSlabSize(InSlabSize),
          CurrentSlabSize(InSlabSize),
          CurrentSlabIndex(0),
          IndexInCurrentSlab(0)
    {
        check(InitialSlabSize > 0);
        AllocateNewSlab();
    }

    ~TNodeMemoryPool()
    {
        for (const auto& Slab : Slabs)
        {
            if (Slab.Memory)
            {
                FMemory::Free(Slab.Memory);
            }
        }
    }
    virtual EPathfindingNodeType GetNodeType() const override;
    
    // 어떤 생성자 파라미터든 받을 수 있는 템플릿 버전 NewNode
    template<typename... Args>
    T* NewNode(Args&&... args)
    {
        if (CurrentSlabIndex >= Slabs.Num() || IndexInCurrentSlab >= Slabs[CurrentSlabIndex].NumNodes)
        {
            AllocateNewSlab();
        }

        if (CurrentSlabIndex >= Slabs.Num())
        {
            return nullptr;
        }

        // T 타입의 크기에 맞춰 메모리 주소 계산
        uint8* NodeMemory = Slabs[CurrentSlabIndex].Memory + (IndexInCurrentSlab * sizeof(T));
        T* NewNodePtr = reinterpret_cast<T*>(NodeMemory);
        
        IndexInCurrentSlab++;

        // placement new로 T 타입의 객체를 직접 생성
        new (NewNodePtr) T(Forward<Args>(args)...);

        return NewNodePtr;
    }

    // !!!!! 제공해주신 Reset 로직을 그대로 적용 !!!!!
    void Reset()
    {
        CurrentSlabIndex = 0;
        IndexInCurrentSlab = 0;

        // 첫 슬래브를 제외한 모든 슬래브의 메모리 해제
        for (int32 i = 1; i < Slabs.Num(); ++i)
        {
            if (Slabs[i].Memory)
            {
                FMemory::Free(Slabs[i].Memory);
            }
        }
        if (Slabs.Num() > 1)
        {
            Slabs.SetNum(1); // 첫 번째 슬래브만 남김
        }

        // 첫 슬래브가 초기 크기보다 커졌다면, 초기 크기로 다시 할당
        if (Slabs.Num() > 0 && Slabs[0].NumNodes > InitialSlabSize)
        {
            if (Slabs[0].Memory)
            {
                FMemory::Free(Slabs[0].Memory);
            }
            // T 타입의 크기와 정렬에 맞춰 메모리 할당
            const int32 BytesToAllocate = InitialSlabSize * sizeof(T);
            Slabs[0].Memory = static_cast<uint8*>(FMemory::Malloc(BytesToAllocate, alignof(T)));
            Slabs[0].NumNodes = InitialSlabSize;
        }
        
        CurrentSlabSize = InitialSlabSize;
    }

private:
    struct FMemorySlab
    {
        uint8* Memory;
        int32 NumNodes;
        FMemorySlab() : Memory(nullptr), NumNodes(0) {}
    };

    void AllocateNewSlab()
    {
        if (Slabs.Num() > 0)
        {
            CurrentSlabSize = FMath::Min(CurrentSlabSize * 2, MaxAllowedSlabSize);
        }

        // T 타입의 크기와 정렬에 맞춰 메모리 할당
        const int32 BytesToAllocate = CurrentSlabSize * sizeof(T);
        uint8* NewMemory = static_cast<uint8*>(FMemory::Malloc(BytesToAllocate, alignof(T)));

        if (Slabs.Num() > 0 && IndexInCurrentSlab >= Slabs[CurrentSlabIndex].NumNodes)
        {
             // 현재 슬래브가 꽉 차서 새 슬래브를 할당하는 경우
            CurrentSlabIndex++;
        }

        if (!Slabs.IsValidIndex(CurrentSlabIndex))
        {
            Slabs.Emplace();
        }

        Slabs[CurrentSlabIndex].Memory = NewMemory;
        Slabs[CurrentSlabIndex].NumNodes = CurrentSlabSize;
        IndexInCurrentSlab = 0;
    }

    TArray<FMemorySlab> Slabs;
    const int32 InitialSlabSize;
    int32 CurrentSlabSize;
    int32 CurrentSlabIndex;
    int32 IndexInCurrentSlab;
    const int32 MaxAllowedSlabSize = 32768; // 예시 값
};

template <>
inline EPathfindingNodeType TNodeMemoryPool<FPathNode>::GetNodeType() const
{
    return EPathfindingNodeType::Grid;
}

template <>
inline EPathfindingNodeType TNodeMemoryPool<FAstarPathNode>::GetNodeType() const
{
    return EPathfindingNodeType::Octree;
}