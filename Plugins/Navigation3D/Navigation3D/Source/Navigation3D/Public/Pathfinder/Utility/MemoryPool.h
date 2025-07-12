// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "StrategyUtils.h"

struct FPathNode;

/**
 * A memory pool for FPathNode using slab allocation.
 * This pool allocates memory in chunks (slabs) on demand, avoiding large upfront
 * memory allocation while maintaining high performance through bump allocation and an O(1) reset.
 */
struct NAVIGATION3D_API FPathNodeMemoryPool
{
public:
	/**
	 * @param InSlabSize The number of nodes to allocate in each memory slab.
	 */
	explicit FPathNodeMemoryPool(int32 InSlabSize = 1024);
	~FPathNodeMemoryPool();

	// Disallow copying and moving to prevent complex memory ownership issues.
	FPathNodeMemoryPool(const FPathNodeMemoryPool&) = delete;
	FPathNodeMemoryPool& operator=(const FPathNodeMemoryPool&) = delete;

	/**
	 * Allocates a new node from the pool. If the current slab is full, a new one is allocated.
	 * @return A raw pointer to the newly constructed node, or nullptr if allocation fails.
	 */
	template<typename... Args>
	FPathNode* NewNode(Args&&... args)
	{
		// Check if the current slab is full or if this is the first allocation.
		if (CurrentSlabIndex >= Slabs.Num() || IndexInCurrentSlab >= SlabSize)
		{
			AllocateNewSlab();
		}

		// If slab allocation failed (e.g., out of memory), return nullptr.
		if (CurrentSlabIndex >= Slabs.Num())
		{
			return nullptr;
		}

		// Get the pointer to the memory location in the current slab.
		FPathNode* NewNodePtr = &Slabs[CurrentSlabIndex][IndexInCurrentSlab];
		IndexInCurrentSlab++;

		// Use placement new to construct the object directly in the allocated memory.
		new (NewNodePtr) FPathNode(Forward<Args>(args)...);

		return NewNodePtr;
	}

	/**
	 * Resets the pool to its initial state for reuse.
	 * This is an O(1) operation that does not deallocate memory.
	 */
	void Reset();

private:
	/**
	 * Allocates a new slab of memory and prepares it for use.
	 */
	void AllocateNewSlab();

	// A list of memory blocks (slabs). Each slab is a TArray of nodes.
	TArray<TArray<FPathNode>> Slabs;

	// The number of nodes contained within each slab.
	int32 SlabSize;

	// The index of the slab we are currently allocating from.
	int32 CurrentSlabIndex;

	// The index of the next available node within the current slab.
	int32 IndexInCurrentSlab;
	
};