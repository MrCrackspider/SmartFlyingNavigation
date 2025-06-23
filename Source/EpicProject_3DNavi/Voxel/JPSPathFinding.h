#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "Public/VoxelGrid.h"
#include "JPSPathFinding.generated.h"

class FJPSNode
{
public:
	FIntVector Position;		//노드의 그리드 좌표
	FIntVector ParentPosition;	//부모 노드의 그리드 좌표

	float GCost = 0.f; //시작점부터 실제비용
	float HCost = 0.f; //목적지까지의 추정비용(Heuristic)
	float FCost = 0.f; //G+H Cost

	FJPSNode(){}
	FJPSNode(const FIntVector& InPosition) : Position(InPosition) {}

	bool operator<(const FJPSNode& Other) const
	{
		return FCost > Other.FCost; // Min-Heap을 위해 > 사용
	}
};

UCLASS()
class EPICPROJECT_3DNAVI_API UJPSPathFinding : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	/**
	 * JPS알고리즘으로 VoxelGrid에서 경로를 찾습니다
	 * @param VoxelGrid - 복셀 데이터가 포함된 그리드 객체
	 * @param StartWorldPos - 시작점 월드좌표
	 * @param EndWorldPos - 목적지 월드좌표
	 * @return 월드 좌표로 구성된 경로 배열, 경로를 찾지 못하면 비어있음
	 */
	UFUNCTION(BlueprintCallable, Category = "PathFinding")
	static TArray<FVector> FindPathJPS(UVoxelGrid* VoxelGrid, const FVector& StartWorldPos, const FVector& EndWorldPos);

private:
	//JPS알고리즘 핵심 헬퍼 함수들
	static void IdentifySuccessors(const FJPSNode& Node, const FIntVector& GoalPos, UVoxelGrid* Grid, TArray<FIntVector>& OutSuccessors);
	static TOptional<FIntVector> Jump(const FIntVector& CurrentPos, const FIntVector& Direction, const FIntVector& GoalPos, UVoxelGrid* Grid);
	static TArray<FVector> ReconstructPath(const FIntVector& GoalPos, const TMap<FIntVector, FIntVector>& CameFrom, UVoxelGrid* Grid);

	//유틸리티 함수들
	static TArray<FIntVector> GetPrunedNeighbors(const FIntVector& CurrentPos, const FIntVector& ParentPos);
	static bool IsWalkable(const FIntVector& GridPos, UVoxelGrid* Grid);
	static float Heuristic(const FIntVector& A, const FIntVector& B);
	static FIntVector GetSignIntVector(const FIntVector& InVec);
};