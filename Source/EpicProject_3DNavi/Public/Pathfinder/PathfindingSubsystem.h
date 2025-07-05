// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "PathfindingSubsystem.generated.h"

class AVoxelVolumeBase;
class UPathfindingComponent;
class UPathfindingStrategy;

// 길찾기 완료 시 호출될 델리게이트 시그니처
// TArray<FIntVector> Path, bool bSuccess
DECLARE_DYNAMIC_DELEGATE_TwoParams(FPathfindingResultDelegate, const TArray<FIntVector>&, Path, bool, bSuccess);

// 길찾기 요청 정보를 담는 구조체
USTRUCT(BlueprintType)
struct FPathfindingRequest
{
	GENERATED_BODY()

	// 각 요청을 식별하기 위한 고유 ID
	UPROPERTY()
	int32 RequestID = -1;

	// 길찾기를 요청한 컴포넌트 (결과를 돌려줄 대상)
	UPROPERTY()
	TWeakObjectPtr<UPathfindingComponent> RequestingComponent;

	// 길찾기 전략
	UPROPERTY()
	TWeakObjectPtr<UPathfindingStrategy> Strategy;

	// 시작점, 끝점, 에이전트 크기
	UPROPERTY()
	FIntVector StartIndex;

	UPROPERTY()
	FIntVector EndIndex;

	UPROPERTY()
	int32 AgentRadiusInVoxels = 0;

	// 요청의 중요도 (나중에 우선순위 큐 구현 시 사용)
	UPROPERTY()
	int32 Priority = 0;

	// 완료 시 호출될 델리게이트
	FPathfindingResultDelegate OnPathfindingComplete;
};

UCLASS()
class EPICPROJECT_3DNAVI_API UPathfindingSubsystem : public UWorldSubsystem, public FTickableGameObject
{
	GENERATED_BODY()

public:
	virtual void Tick(float DeltaTime) override;
	virtual TStatId GetStatId() const override;


	/**
	 * 길찾기를 요청합니다. 컴포넌트는 이 함수를 호출합니다.
	 * @param Request 길찾기에 필요한 모든 정보를 담은 요청 구조체
	 * @return 요청이 성공적으로 접수되면 고유한 RequestID를, 실패하면 -1을 반환합니다.
	 */
	UFUNCTION(BlueprintCallable, Category = "Pathfinding")
	int32 RequestPath(const FPathfindingRequest& Request);

	/**
	 * 특정 길찾기 요청을 취소합니다.
	 * @param RequestID 취소할 요청의 ID
	 */
	UFUNCTION(BlueprintCallable, Category = "Pathfinding")
	void CancelPathRequest(int32 RequestID);

protected:
	// --- Subsystem Lifecycle ---
	virtual void Initialize(FSubsystemCollectionBase& Collection) override;
	virtual void Deinitialize() override;
	virtual void OnWorldBeginPlay(UWorld& InWorld) override; // Tick을 시작하기 좋은 위치

private:
	// --- 새로운 길찾기 요청 처리 시스템 ---

	/** 매 프레임 호출되어 큐를 처리하는 함수 */
	void ProcessPathfindingQueue();

	/** 길찾기 태스크가 완료되었을 때 호출되는 함수 (게임 스레드에서 실행) */
	void OnPathfindingTaskComplete(int32 RequestID, TArray<FIntVector> ResultPath,bool bSuccess);

	/** 다음 요청에 할당할 고유 ID */
	int32 NextRequestID = 0;

	/** 접수된 길찾기 요청을 보관하는 큐 */
	UPROPERTY()
	TArray<FPathfindingRequest> RequestQueue;

	UPROPERTY()
	TMap<int32, FPathfindingRequest> ActiveRequests;

	/** 한 프레임에 처리할 최대 길찾기 태스크 수 */
	UPROPERTY(EditAnywhere, Category = "Pathfinding|Performance")
	int32 MaxTasksPerFrame = 3;

	/** 큐 처리를 위한 타이머 핸들 */
	FTimerHandle QueueProcessTimerHandle;

};
