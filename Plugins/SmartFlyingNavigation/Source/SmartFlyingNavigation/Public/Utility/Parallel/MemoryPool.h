// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Utility/AStarStrategyUtils.h"


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

        uint8* NodeMemory = Slabs[CurrentSlabIndex].Memory + (IndexInCurrentSlab * sizeof(T));
        T* NewNodePtr = reinterpret_cast<T*>(NodeMemory);
        
        IndexInCurrentSlab++;

        new (NewNodePtr) T(Forward<Args>(args)...);

        return NewNodePtr;
    }

    void Reset()
    {
        CurrentSlabIndex = 0;
        IndexInCurrentSlab = 0;

        for (int32 i = 1; i < Slabs.Num(); ++i)
        {
            if (Slabs[i].Memory)
            {
                FMemory::Free(Slabs[i].Memory);
            }
        }
        if (Slabs.Num() > 1)
        {
            Slabs.SetNum(1);
        }

        if (Slabs.Num() > 0 && Slabs[0].NumNodes > InitialSlabSize)
        {
            if (Slabs[0].Memory)
            {
                FMemory::Free(Slabs[0].Memory);
            }
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

        const int32 BytesToAllocate = CurrentSlabSize * sizeof(T);
        uint8* NewMemory = static_cast<uint8*>(FMemory::Malloc(BytesToAllocate, alignof(T)));

        if (Slabs.Num() > 0 && IndexInCurrentSlab >= Slabs[CurrentSlabIndex].NumNodes)
        {
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
    const int32 MaxAllowedSlabSize = 32768;
};

template <>
inline EPathfindingNodeType TNodeMemoryPool<FAstarPathNode>::GetNodeType() const
{
    return EPathfindingNodeType::Octree;
}