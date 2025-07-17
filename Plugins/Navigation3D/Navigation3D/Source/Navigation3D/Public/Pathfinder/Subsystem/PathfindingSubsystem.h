// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Pathfinder/Utility/Parallel/PathfindingRequest.h"
#include "Subsystems/WorldSubsystem.h"
#include "PathfindingSubsystem.generated.h"


class AVoxelVolumeBase;
class UPathfindingComponent;
class UPathfindingStrategy;
class FPathfindingWorker;
class FQueuedThreadPool;

struct FPathfindingResult
{
	FPathfindingRequest Request;
	TArray<FIntVector> VoxelPath;
	TArray<FVector> WorldPath;
	bool bSuccess;
};

// FAsyncTask 대신 IQueuedWork 인터페이스를 구현하는 작업 클래스
class FPathfindingAsyncTasks : public IQueuedWork
{
public:
	FPathfindingAsyncTasks(const FPathfindingRequest& InRequest, UPathfindingSubsystem* InOwner)
	  : Request(InRequest) // InRequest의 내용이 멤버 변수 Request로 '복사'됩니다.
	  , OwnerSubsystem(InOwner)
	{
	}

	// IQueuedWork 인터페이스
	virtual void DoThreadedWork() override;
	virtual void Abandon() override;
private:
	FPathfindingRequest Request;
	TWeakObjectPtr<UPathfindingSubsystem> OwnerSubsystem;
};

UCLASS(Config=Game)
class NAVIGATION3D_API UPathfindingSubsystem : public UEngineSubsystem, public FTickableGameObject
{
	GENERATED_BODY()

public:
	friend class FPathfindingAsyncTask;
	/**
	 * 길찾기를 요청합니다. 컴포넌트는 이 함수를 호출합니다.
	 * @param Request 길찾기에 필요한 모든 정보를 담은 요청 구조체
	 * @return 요청이 성공적으로 접수되면 고유한 RequestID를, 실패하면 -1을 반환합니다.
	 */
	UFUNCTION(BlueprintCallable, Category = "Pathfinding")
	int32 RequestPath(FPathfindingRequest& Request);

	void OnPathfindingCompleted(const FPathfindingRequest& Request, const TArray<FIntVector>& VoxelPath, bool bSuccess);
	void OnPathfindingCompleted(const FPathfindingRequest& Request, const TArray<FVector>& WorldPath, bool bSuccess);

	UFUNCTION(BlueprintPure, Category = "Pathfinding|Debug")
	int32 GetNumWorkerThreads() const;

	/** 워커 스레드의 수를 변경하고 스레드 풀을 재시작합니다. */
	UFUNCTION(BlueprintCallable, Category = "Pathfinding|Debug")
	void SetNumWorkerThreads(int32 NewNumThreads);

	void QueueCompletedResult(const FPathfindingResult& Result);
	
	/** 매 프레임 호출될 틱 함수입니다. */
	virtual void Tick(float DeltaTime) override;
	/** 이 객체가 틱을 돌릴 수 있는지 여부를 반환합니다. 에디터에서도 틱을 돌리려면 true를 반환해야 합니다. */
	virtual bool IsTickableInEditor() const override { return true; }

	
	/** 이 객체가 틱을 돌릴 수 있는지 여부를 반환합니다. */
	virtual bool IsTickable() const override { return true; }

	/** 틱 그룹을 반환합니다. 보통 TG_PrePhysics 또는 TG_PostPhysics를 사용합니다. */
	virtual TStatId GetStatId() const override {return TStatId(); };

	// 경로탐색 작업이 메모리 풀을 빌려갈 때 호출하는 함수입니다.
	FPathNodeMemoryPoolBase* LeasePool(EPathfindingNodeType NodeType);

	// 작업이 끝난 후 메모리 풀을 반납할 때 호출하는 함수입니다.
	void ReturnPool(FPathNodeMemoryPoolBase* PoolToReturn);
	
protected:
	// --- Subsystem Lifecycle ---
	virtual void Initialize(FSubsystemCollectionBase& Collection) override;
	virtual void Deinitialize() override;
	
	
private:
	
	UPROPERTY(Config)
	int32 CurrentNumWorkerThreads = 5;

	/** 길찾기 전용 스레드 풀 */
	FQueuedThreadPool* PathfindingThreadPool = nullptr;
	/**
	 * 고유한 요청 ID를 발급하기 위한 스레드 안전 카운터.
	 * 여러 스레드에서 동시에 ID를 요청해도 안전합니다.
	 */
	UPROPERTY(Config)
	int32 MaxCallbacksPerFrame = 10; // 프레임당 처리할 최대 콜백 수
	
	FThreadSafeCounter NextRequestID;
	
	/** ActiveTasks 배열에 대한 동시 접근을 막기 위한 크리티컬 섹션. */
	FCriticalSection TasksCriticalSection;

	// 큐 접근을 보호할 크리티컬 섹션
	FCriticalSection QueueLock;

	/** 완료된 작업 결과를 담는 스레드 안전 큐 */
	TQueue<FPathfindingResult> CompletedTasksQueue;
	
private:
	TArray<FPathNodeMemoryPoolBase*> IdleGridPools;
	TArray<FPathNodeMemoryPoolBase*> IdleOctreePools;

	// 여러 스레드에서 동시에 풀을 빌리거나 반납할 때의 충돌을 막는 잠금 장치입니다.
	FCriticalSection PoolLock;

};
