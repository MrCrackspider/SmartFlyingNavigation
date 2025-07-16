#pragma once
#include "CoreMinimal.h"

// D* Lite의 Key는 두 개의 float 값으로 구성됩니다.
struct FDStarLiteKey
{
	float K1 = 0.0f;
	float K2 = 0.0f;

	FDStarLiteKey(float InK1 = FLT_MAX, float InK2 = FLT_MAX) : K1(InK1), K2(InK2) {}

	// 우선순위 비교 연산자
	// bool operator<(const FDStarLiteKey& Other) const
	// {
	// 	if (FMath::IsNearlyEqual(K1, Other.K1))
	// 	{
	// 		return K2 < Other.K2;
	// 	}
	// 	return K1 < Other.K1;
	// }
};

// 우선순위 큐에 저장될 요소
struct FDStarLiteQueueNode
{
	FIntVector VoxelIndex;
	FDStarLiteKey Key;

	// // 힙(Heap) 정렬을 위한 비교 연산자
	// bool operator<(const FDStarLiteQueueNode& Other) const
	// {
	// 	return Other.Key < Key; // Min-Heap: Key가 작은 것이 우선순위가 높음
	// }
};

/**
 * D* Lite 알고리즘을 위한 우선순위 큐.
 * 노드 존재 여부 및 Key 값 업데이트를 O(log N)으로 수행하기 위해 TMap을 함께 사용합니다.
 */
class FDStarLitePriorityQueue
{
public:
	// 큐에 노드를 추가하거나 이미 있다면 Key 값을 업데이트합니다.
	// void PushOrUpdate(const FIntVector& Index, const FDStarLiteKey& NewKey)
	// {
	// 	if (NodeLookup.Contains(Index))
	// 	{
	// 		// 이미 있다면 기존 노드를 찾아 Key를 업데이트하고 힙을 재정렬합니다.
	// 		for (int32 i = 0; i < Heap.Num(); ++i)
	// 		{
	// 			if (Heap[i].VoxelIndex == Index)
	// 			{
	// 				Heap[i].Key = NewKey;
	// 				Heap.Heapify(FDStarLiteQueueNode());
	// 				break;
	// 			}
	// 		}
	// 	}
	// 	else
	// 	{
	// 		// 없다면 새로 추가합니다.
	// 		Heap.HeapPush(FDStarLiteQueueNode{ Index, NewKey }, FDStarLiteQueueNode());
	// 	}
	// 	NodeLookup.Add(Index, NewKey);
	// }
	//
	// // 우선순위가 가장 높은 노드를 꺼냅니다.
	// FIntVector Pop()
	// {
	// 	FDStarLiteQueueNode TopNode;
	// 	Heap.HeapPop(TopNode, FDStarLiteQueueNode());
	// 	NodeLookup.Remove(TopNode.VoxelIndex);
	// 	return TopNode.VoxelIndex;
	// }
	//
	// // 큐에서 노드를 제거합니다.
	// void Remove(const FIntVector& Index)
	// {
	// 	if (!NodeLookup.Contains(Index)) return;
	//
	// 	for (int32 i = 0; i < Heap.Num(); ++i)
	// 	{
	// 		if (Heap[i].VoxelIndex == Index)
	// 		{
	// 			Heap.RemoveAt(i);
	// 			Heap.Heapify(FDStarLiteQueueNode());
	// 			break;
	// 		}
	// 	}
	// 	NodeLookup.Remove(Index);
	// }
	//
	// // 우선순위가 가장 높은 노드의 Key를 반환합니다.
	// FDStarLiteKey TopKey() const
	// {
	// 	return Heap.Num() > 0 ? Heap.HeapTop().Key : FDStarLiteKey();
	// }
	//
	// // 큐가 비어있는지 확인합니다.
	// bool IsEmpty() const { return Heap.Num() == 0; }
	//
	// // 큐에 특정 노드가 있는지 확인합니다.
	// bool Contains(const FIntVector& Index) const { return NodeLookup.Contains(Index); }
	//
	// void Empty()
	// {
	// 	Heap.Empty();
	// 	NodeLookup.Empty();
	// }

private:
	TArray<FDStarLiteQueueNode> Heap;
	TMap<FIntVector, FDStarLiteKey> NodeLookup; // 빠른 조회를 위한 맵
};