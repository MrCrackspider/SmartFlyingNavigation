// Fill out your copyright notice in the Description page of Project Settings.

#include "Pathfinder/Utility/MemoryPool.h"
#include "Pathfinder/Utility/StrategyUtils.h" // For FPathNode definition

FPathNodeMemoryPool::FPathNodeMemoryPool(int32 InSlabSize)
	: InitialSlabSize(InSlabSize),
	  CurrentSlabSize(InSlabSize),
	  CurrentSlabIndex(0),
	  IndexInCurrentSlab(0)
{
	check(InitialSlabSize > 0);
	UE_LOG(LogTemp, Log, TEXT("FPathNodeMemoryPool (Raw Memory) created with slab size %d."), InitialSlabSize);
	AllocateNewSlab();
}

FPathNodeMemoryPool::~FPathNodeMemoryPool()
{
	UE_LOG(LogTemp, Log, TEXT("FPathNodeMemoryPool (Raw Memory) destroying... Freeing %d slabs."), Slabs.Num());
	// This is the FAST part. We just free the memory blocks directly.
	for (const FMemorySlab& Slab : Slabs)
	{
		if (Slab.Memory)
		{
			// Free the entire slab in one go, no individual destructors are called.
			FMemory::Free(Slab.Memory);
		}
	}
	// The TArray 'Slabs' itself will be cleared automatically.
}


void FPathNodeMemoryPool::Reset()
{
	CurrentSlabIndex = 0;
	IndexInCurrentSlab = 0;

	// Free memory for all slabs except the first one.
	for (int32 i = 1; i < Slabs.Num(); ++i)
	{
		if (Slabs[i].Memory)
		{
			FMemory::Free(Slabs[i].Memory);
		}
	}
	Slabs.SetNum(1); // Keep only the first slab entry

	// Reallocate the first slab to its initial size if it has grown.
	if (Slabs[0].NumNodes > InitialSlabSize)
	{
		if (Slabs[0].Memory)
		{
			FMemory::Free(Slabs[0].Memory);
		}
		const int32 BytesToAllocate = InitialSlabSize * sizeof(FPathNode);
		Slabs[0].Memory = static_cast<uint8*>(FMemory::Malloc(BytesToAllocate, alignof(FPathNode)));
		Slabs[0].NumNodes = InitialSlabSize;
	}
	
	CurrentSlabSize = InitialSlabSize;
}

void FPathNodeMemoryPool::AllocateNewSlab()
{
	if (Slabs.Num() > 0)
	{
		CurrentSlabSize = FMath::Min(CurrentSlabSize * 2, MaxAllowedSlabSize);
	}

	const int32 BytesToAllocate = CurrentSlabSize * sizeof(FPathNode);
	// Allocate raw, aligned memory.
	uint8* NewMemory = static_cast<uint8*>(FMemory::Malloc(BytesToAllocate, alignof(FPathNode)));

	if (Slabs.Num() > 0)
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