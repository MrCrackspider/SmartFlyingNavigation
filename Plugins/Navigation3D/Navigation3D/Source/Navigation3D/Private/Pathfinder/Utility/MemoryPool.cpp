// Fill out your copyright notice in the Description page of Project Settings.

#include "Pathfinder/Utility/MemoryPool.h"
#include "Pathfinder/Utility/StrategyUtils.h" // For FPathNode definition

FPathNodeMemoryPool::FPathNodeMemoryPool(int32 InSlabSize)
	: SlabSize(InSlabSize),
	  CurrentSlabIndex(0),
	  IndexInCurrentSlab(0)
{
	check(SlabSize > 0);
	UE_LOG(LogTemp, Log, TEXT("FPathNodeMemoryPool created with slab size %d."), SlabSize);
	// Allocate the first slab immediately to simplify the NewNode logic.
	AllocateNewSlab();
}

FPathNodeMemoryPool::~FPathNodeMemoryPool()
{
	// TArray's destructor will handle freeing all memory automatically.
	// Slabs.Empty(); // This would also work but is redundant.
	UE_LOG(LogTemp, Log, TEXT("FPathNodeMemoryPool destroyed, freeing %d slabs."), Slabs.Num());
}

void FPathNodeMemoryPool::Reset()
{
	// Reset the counters to point to the start of the first slab.
	// This makes all allocated memory available for reuse without deallocation.
	CurrentSlabIndex = 0;
	IndexInCurrentSlab = 0;
}

void FPathNodeMemoryPool::AllocateNewSlab()
{
	// If we are trying to allocate past the end of our current slab list, add a new one.
	if (CurrentSlabIndex >= Slabs.Num())
	{
		UE_LOG(LogTemp, Log, TEXT("Allocating new slab #%d for PathNodeMemoryPool."), Slabs.Num() + 1);
		// Add a new TArray (slab) to our list of slabs.
		Slabs.Emplace();
		// Allocate the memory for the nodes within the new slab.
		Slabs.Last().SetNum(SlabSize);
	}
	
	// Move to the next slab in the list.
	CurrentSlabIndex++;
	// Reset the node index to the beginning of this new slab.
	IndexInCurrentSlab = 0;
}