// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "StrategyUtils.h"


struct FPathNode;

/**
 * A raw memory slab used by the memory pool.
 */
struct FMemorySlab
{
	uint8* Memory;
	int32 NumNodes;

	FMemorySlab() : Memory(nullptr), NumNodes(0) {}
};

/**
 * A memory pool for FPathNode using slab allocation with raw memory.
 * This version prioritizes extremely fast destruction by bypassing individual node destructors.
 * WARNING: Only use if FPathNode has a trivial destructor.
 */
struct NAVIGATION3D_API FPathNodeMemoryPool
{
public:
	explicit FPathNodeMemoryPool(int32 InSlabSize = 1024);
	~FPathNodeMemoryPool();
	
	template<typename... Args>
	FPathNode* NewNode(Args&&... args)
	{
		if (CurrentSlabIndex >= Slabs.Num() || IndexInCurrentSlab >= Slabs[CurrentSlabIndex].NumNodes)
		{
			AllocateNewSlab();
		}

		if (CurrentSlabIndex >= Slabs.Num())
		{
			return nullptr;
		}

		// Calculate the memory address for the new node
		uint8* NodeMemory = Slabs[CurrentSlabIndex].Memory + (IndexInCurrentSlab * sizeof(FPathNode));
		FPathNode* NewNodePtr = reinterpret_cast<FPathNode*>(NodeMemory);
		
		IndexInCurrentSlab++;

		// Use placement new to construct the object directly in the allocated memory.
		new (NewNodePtr) FPathNode(Forward<Args>(args)...);

		return NewNodePtr;
	}

	void Reset();

private:
	void AllocateNewSlab();

	// Store slabs as raw memory blocks instead of TArray of objects
	TArray<FMemorySlab> Slabs;

	const int32 InitialSlabSize;
	int32 CurrentSlabSize;
	int32 CurrentSlabIndex;
	int32 IndexInCurrentSlab;

	const int32 MaxAllowedSlabSize = 32768;
};
