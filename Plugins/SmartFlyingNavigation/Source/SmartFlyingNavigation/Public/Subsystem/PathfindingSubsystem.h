// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "Tickable.h"
#include "Containers/Queue.h"
#include "HAL/ThreadSafeCounter.h"
#include "HAL/CriticalSection.h"
#include "Misc/IQueuedWork.h"
#include "Utility/Parallel/PathfindingRequest.h"
#include "PathfindingSubsystem.generated.h"

class UWorldNavigationDataAsset;
class UPathfindingSubsystem;
struct FNavigationChunkRecord;
struct FNavigationChunkPortal;
class FQueuedThreadPool;

/** TUniquePtr deleter for FQueuedThreadPool (must be torn down via Destroy(), not delete). */
struct FQueuedThreadPoolDeleter
{
	void operator()(FQueuedThreadPool* Pool) const;
};

class FPathfindingAsyncTasks : public IQueuedWork
{
public:
	FPathfindingAsyncTasks(const FPathfindingRequest& InRequest, UPathfindingSubsystem* InOwner)
	  : Request(InRequest)
	  , OwnerSubsystem(InOwner)
	{
	}

	/** Main work function executed on the thread pool. */
	virtual void DoThreadedWork() override;

	/** Called when the task is abandoned (e.g., during thread pool destruction). */
	virtual void Abandon() override;

private:
	/** The pathfinding request to process. */
	FPathfindingRequest Request;

	/** A weak pointer to the owning subsystem. */
	TWeakObjectPtr<UPathfindingSubsystem> OwnerSubsystem;
};

/**
 * Represents the result of a completed pathfinding request.
 */
struct FPathfindingResult
{
	FPathfindingRequest Request;
	TArray<FVector> WorldPath;
	bool bSuccess;
};

/**
 * Manages asynchronous pathfinding requests using a dedicated thread pool.
 * This subsystem handles queuing requests, executing them on worker threads,
 * and delivering results back to the game thread.
 */
UCLASS(Config = Game)
class SMARTFLYINGNAVIGATION_API UPathfindingSubsystem : public UTickableWorldSubsystem
{
	GENERATED_BODY()

public:
	//~=============================================================================
	// Public API
	//~=============================================================================

	UFUNCTION(BlueprintCallable, Category = "Pathfinding", meta = (WorldContext = "WorldContextObject"))
	int32 RequestPath(const UObject* WorldContextObject, FPathfindingRequest& Request);

	//~=============================================================================
	// Tick Interface
	//~=============================================================================

	virtual void Tick(float DeltaTime) override;
	virtual TStatId GetStatId() const override;

	/** Keep ticking while paused so the debug panel / HUD stay responsive and paused agents keep their path lines. */
	virtual bool IsTickableWhenPaused() const override { return true; }

	//~=============================================================================
	// Debug & Configuration
	//~=============================================================================

	/** Gets the current number of active worker threads. */
	UFUNCTION(BlueprintPure, Category = "Pathfinding|Debug")
	int32 GetNumWorkerThreads() const;

	/**
	 * Sets the number of worker threads. This will destroy and recreate the thread pool.
	 * @param NewNumThreads The desired number of threads. The value will be clamped to a reasonable range.
	 */
	UFUNCTION(BlueprintCallable, Category = "Pathfinding|Debug")
	void SetNumWorkerThreads(int32 NewNumThreads);

	UFUNCTION(BlueprintPure, Category = "Pathfinding|Navigation Data")
	bool HasWorldNavigationData() const { return CurrentWorldNavData != nullptr; }

	const UWorldNavigationDataAsset* GetWorldNavigationData() const { return CurrentWorldNavData; }
	bool TryGetNavigationChunkForLocation(const FVector& WorldLocation, FNavigationChunkRecord& OutChunk) const;
	bool GetChunkPortals(int32 ChunkId, TArray<FNavigationChunkPortal>& OutPortals) const;
	bool FindHighLevelChunkPath(const FVector& StartLocation, const FVector& EndLocation, TArray<int32>& OutChunkPath) const;

	//~=============================================================================
	// Internal Management (Called by worker threads)
	//~=============================================================================

	/**
	 * Queues a completed pathfinding result to be processed on the game thread.
	 * This method is thread-safe.
	 */
	void QueueCompletedResult(const FPathfindingResult& Result);

	/**
	 * Leases a memory pool for a pathfinding task. Thread-safe.
	 */
	FPathNodeMemoryPoolBase* LeasePool(EPathfindingNodeType NodeType);

	/**
	 * Returns a memory pool to the subsystem after a task is complete. Thread-safe.
	 */
	void ReturnPool(FPathNodeMemoryPoolBase* PoolToReturn);

protected:
	//~=============================================================================
	// Subsystem Lifecycle
	//~=============================================================================

	virtual void Initialize(FSubsystemCollectionBase& Collection) override;
	virtual void Deinitialize() override;

private:
	//~=============================================================================
	// Path Completion Callbacks
	//~=============================================================================

	void OnPathfindingCompleted(const FPathfindingRequest& Request, const TArray<FVector>& WorldPath, bool bSuccess);

	/** On-screen benchmark overlay (gated by sfn.ShowNavHUD). */
	void DrawNavHUD(float DeltaTime) const;

#if !UE_BUILD_SHIPPING
	/** Lazily creates the on-device debug panel (gated by sfn.ShowDebugPanel) and handles sfn.StartPaused. */
	void TickDebugTools();

	TSharedPtr<SWidget> DebugPanelWidget;
	bool bStartPausedHandled = false;
#endif

	//~=============================================================================
	// Threading & Task Management
	//~=============================================================================

	TUniquePtr<FQueuedThreadPool, FQueuedThreadPoolDeleter> PathfindingThreadPool;

	UPROPERTY(Config)
	int32 CurrentNumWorkerThreads = 5;

	UPROPERTY(Config)
	int32 MaxCallbacksPerFrame = 10;

	FThreadSafeCounter NextRequestID;
	TQueue<FPathfindingResult, EQueueMode::Mpsc> CompletedTasksQueue;

	//~=============================================================================
	// Memory Pooling
	//~=============================================================================

	FCriticalSection PoolLock;
	TArray<TUniquePtr<FPathNodeMemoryPoolBase>> IdleGridPools;
	TArray<TUniquePtr<FPathNodeMemoryPoolBase>> IdleOctreePools;

	//~=============================================================================
	// World Navigation Manifest
	//~=============================================================================

	UPROPERTY()
	TObjectPtr<UWorldNavigationDataAsset> CurrentWorldNavData;
};
