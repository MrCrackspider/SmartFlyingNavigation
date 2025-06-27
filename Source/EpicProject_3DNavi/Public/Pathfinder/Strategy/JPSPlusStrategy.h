// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "PathfindingStrategy.h"
#include "JPSPlusStrategy.generated.h"

struct FPathNode;
/**
 * 
 */
class AFenwickVoxelVolume;
UCLASS()
class EPICPROJECT_3DNAVI_API UJPSPlusStrategy : public UPathfindingStrategy
{
	GENERATED_BODY()

public:
	virtual void Initialize(AVoxelVolume* InVolume) override;
	virtual bool FindPathWithStrategy(const FIntVector& StartIndex, const FIntVector& EndIndex,
	                                  int32 AgentRadiusInVoxels, TArray<FIntVector>& OutPath) override;
	virtual FName GetStrategyName() const override;

protected:
	EPathfindingResult FindPathInternal(const FIntVector& StartIndex, const FIntVector& EndIndex,
	                                    TArray<FIntVector>& OutPath, uint32 AgentRadiusInVoxels);

	// --- 엄밀하게 재정의된 JPS 핵심 로직 ---

	/** @brief 부모로부터의 방향 Dir을 기반으로, 엄밀한 3D Pruning 규칙에 따라 탐색할 방향을 결정합니다. */
	void GetPrunedDirections(const FIntVector& Dir, TArray<FIntVector>& OutDirections) const;

	/** @brief Pruned Directions에 대해 Jump를 수행하여 후속 점프 포인트(Successors)를 식별합니다. */
	void IdentifySuccessors(const TSharedPtr<FPathNode>& Node, const FIntVector& Goal, int32 AgentRadiusInVoxels,
	                        TArray<TSharedPtr<FPathNode>>& OutSuccessors);

	/** @brief 한 방향으로 점프하며 Goal, Forced Neighbour, 또는 하위 차원 점프 포인트를 찾습니다. */
	TOptional<FIntVector> Jump(const FIntVector& CurrentPos, const FIntVector& Direction, const FIntVector& Goal,
	                           int32 AgentRadiusInVoxels);

	bool HasForcedNeighbour(const FIntVector& NodePos, const FIntVector& CheckDir, const FIntVector& ParentDir, uint32 AgentRadiusInVoxels) const;

	/** @brief 두 지점 간 이동 시, 인접 공간의 장애물 분포를 직접 검사하여 코너 컷팅 발생 여부를 엄밀하게 판단합니다. */
	bool IsAgentMoveFeasible(const FIntVector& From, const FIntVector& To, int32 AgentRadiusInVoxels) const;

	// --- 안전성이 강화된 경로 복원 및 최적화 ---

	/** @brief 3D Bresenham 알고리즘을 이용, 점프 포인트 체인으로부터 안전한 전체 그리드 경로를 보간하여 복원합니다. */
	void RetracePath(TSharedPtr<FPathNode> EndNode, TArray<FIntVector>& OutGridPath) const;

	/** @brief 그리드 경로를 Theta* 스타일의 LOS 검사를 통해 직선화하여 최종 경로를 생성합니다. 경계 조건 처리가 강화되었습니다. */
	void OptimizePath_ThetaStarStyle(const TArray<FIntVector>& GridPath, TArray<FIntVector>& OutOptimizedPath, uint32 AgentRadiusInVoxels) const;

	/** @brief 두 지점 사이에 장애물이 없는지(Line of Sight) 3D Bresenham 알고리즘으로 안전하게 확인합니다. */
	bool HasLineOfSight(const FIntVector& From, const FIntVector& To, int32 AgentRadiusInVoxels) const;

	// --- 유틸리티 함수 ---
	int32 CalculateHeuristic(const FIntVector& A, const FIntVector& B) const;
	int32 GetMoveCost(const FIntVector& A, const FIntVector& B) const;
public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pathfinding|Performance")
	int32 MaxNodesToProcessForPathfinding = 2000;
private:
	TWeakObjectPtr<AFenwickVoxelVolume> Volume;

	// A* 탐색을 위한 자료구조
	TArray<TSharedPtr<FPathNode>> OpenSet;
	TMap<FIntVector, TSharedPtr<FPathNode>> OpenSetLookup;
	TMap<FIntVector, TSharedPtr<FPathNode>> ClosedSet;

	// 미리 계산된 26개 방향 벡터
	TArray<FIntVector> AllDirections;
};
