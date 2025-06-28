// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
// #include "MemoryPool.generated.h"

struct FPathNode;

// USTRUCT()
struct EPICPROJECT_3DNAVI_API FPathNodeMemoryPool
{
public:
	// GENERATED_BODY()
	/**
	 * @param MaxNodes 풀이 최대로 가질 수 있는 노드의 개수.
	 */
	explicit FPathNodeMemoryPool(int32 MaxNodes = 8192);
	~FPathNodeMemoryPool();
	
	//복사 및 이동 하지말것
	// 복사 및 이동을 금지하여 메모리 소유권 문제를 방지
	FPathNodeMemoryPool(const FPathNodeMemoryPool&) = delete;
	FPathNodeMemoryPool& operator=(const FPathNodeMemoryPool&) = delete;

	/**
	 * 풀에서 새로운 노드를 할당받습니다.
	 * @return 새로 할당된 노드를 가리키는 로우 포인터.
	 */
	template<typename... Args>
	FPathNode* NewNode(Args&&... args)
	{
		if (NumUsedNodes >= NodeMemory.Num())
		{
			UE_LOG(LogTemp, Error, TEXT("PathNodeMemoryPool is out of memory! Consider increasing the pool size."));
			return nullptr;
		}

		FPathNode* NewNodePtr = &NodeMemory[NumUsedNodes];
		NumUsedNodes++;

		// Placement New: 할당된 메모리 위치에 직접 객체를 생성합니다.
		// FPathNode의 생성자를 호출하여 즉시 초기화합니다.
		new (NewNodePtr) FPathNode(Forward<Args>(args)...);

		return NewNodePtr;
	}

	/**
	 * 풀을 초기 상태로 리셋합니다. 메모리를 해제하지 않고 사용 카운트만 0으로 만듭니다.
	 * 길찾기가 끝날 때마다 호출됩니다.
	 */
	void Reset();

private:
	// 모든 노드를 저장할 연속적인 메모리 블록
	TArray<FPathNode> NodeMemory;

	// 현재까지 몇 개의 노드를 사용했는지 나타내는 카운터
	int32 NumUsedNodes;
	
};