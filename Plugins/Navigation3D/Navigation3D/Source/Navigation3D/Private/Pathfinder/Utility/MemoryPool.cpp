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
	CurrentSlabIndex = 0;
	IndexInCurrentSlab = 0;
	if (Slabs.Num() > 1)
	{
		Slabs.SetNum(1);
		Slabs[0].SetNum(SlabSize);
	}
}

void FPathNodeMemoryPool::AllocateNewSlab()
{
	// 새 슬래브 크기 결정
	int32 NewSlabSize = (Slabs.Num() == 0) ? SlabSize : SlabSize << Slabs.Num();
	
	Slabs.Emplace();
	Slabs.Last().SetNum(NewSlabSize);

	// 다음 확장에 대비해 SlabSize 업데이트
	SlabSize = NewSlabSize;

	CurrentSlabIndex = Slabs.Num() - 1;
	IndexInCurrentSlab = 0;
}