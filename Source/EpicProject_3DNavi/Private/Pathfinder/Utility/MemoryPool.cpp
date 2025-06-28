// Fill out your copyright notice in the Description page of Project Settings.

#include "Pathfinder/Utility/MemoryPool.h"
#include "Pathfinder/Utility/StrategyUtils.h"

FPathNodeMemoryPool::FPathNodeMemoryPool(int32 MaxNodes)
	: NumUsedNodes(0)
{
	// 생성자에서 미리 메모리를 할당하고 초기화해 둡니다.
	// TArray의 Reserve가 아니라, 실제 메모리를 할당하고 기본 생성자를 호출하는 SetNum를 사용합니다.
	NodeMemory.SetNum(MaxNodes);
	UE_LOG(LogTemp, Log, TEXT("PathNodeMemoryPool initialized with %d nodes."), MaxNodes);
}

FPathNodeMemoryPool::~FPathNodeMemoryPool()
{
}

void FPathNodeMemoryPool::Reset()
{
	// 사용 카운터만 0으로 되돌립니다.
	// 할당된 메모리는 그대로 유지되며, 다음 길찾기 때 재사용됩니다.
	// 이것이 이 방식의 핵심적인 속도 비결입니다.
	NumUsedNodes = 0;
}
