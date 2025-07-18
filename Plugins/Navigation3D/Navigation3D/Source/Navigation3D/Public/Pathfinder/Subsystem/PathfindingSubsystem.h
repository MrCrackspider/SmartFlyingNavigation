// Fill out your copyright notice in the Description page of Project Settings.
#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "GameFramework/Actor.h" // For FTickableGameObject
#include "Pathfinder/Utility/Parallel/PathfindingRequest.h"
#include "PathfindingSubsystem.generated.h"

// Forward Declarations
class FQueuedThreadPool;
struct FPathNodeMemoryPoolBase;

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
UCLASS(Config=Game)
class NAVIGATION3D_API UPathfindingSubsystem : public UEngineSubsystem, public FTickableGameObject
{
	GENERATED_BODY()

public:
	//~=============================================================================
	// Public API
	//~=============================================================================

	/**
	 * Submits a new pathfinding request to be processed asynchronously.
	 * @param Request The pathfinding request parameters.
	 * @return A unique ID for the submitted request, or -1 if the request is invalid or the subsystem is not initialized.
	 */
	UFUNCTION(BlueprintCallable, Category = "Pathfinding")
	int32 RequestPath(FPathfindingRequest& Request);

	//~=============================================================================
	// Tick Interface
	//~=============================================================================

	virtual void Tick(float DeltaTime) override;
	virtual bool IsTickable() const override { return true; }
	virtual bool IsTickableInEditor() const override { return true; }
	virtual TStatId GetStatId() const override { return TStatId(); }

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

	//~=============================================================================
	// Internal Management (Called by worker threads)
	//~=============================================================================

	/**
	 * Queues a completed pathfinding result to be processed on the game thread.
	 * This method is thread-safe.
	 * @param Result The result from the pathfinding task.
	 */
	void QueueCompletedResult(const FPathfindingResult& Result);

	/**
	 * Leases a memory pool for a pathfinding task.
	 * This method is thread-safe.
	 * @param NodeType The type of node the pool should support.
	 * @return A pointer to a memory pool, or nullptr if allocation fails.
	 */
	FPathNodeMemoryPoolBase* LeasePool(EPathfindingNodeType NodeType);

	/**
	 * Returns a memory pool to the subsystem after a task is complete.
	 * This method is thread-safe.
	 * @param PoolToReturn The pool to return.
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
	
	/** Called on the game thread when a world-space path request is complete. */
	void OnPathfindingCompleted(const FPathfindingRequest& Request, const TArray<FVector>& WorldPath, bool bSuccess);

	//~=============================================================================
	// Threading & Task Management
	//~=============================================================================

	/** The thread pool for executing pathfinding tasks. */
	FQueuedThreadPool* PathfindingThreadPool = nullptr;

	/** The current number of worker threads in the pool. */
	UPROPERTY(Config)
	int32 CurrentNumWorkerThreads = 5;

	/** The maximum number of completed pathfinding callbacks to fire per frame. */
	UPROPERTY(Config)
	int32 MaxCallbacksPerFrame = 10;

	/** A thread-safe counter for generating unique request IDs. */
	FThreadSafeCounter NextRequestID;

	/** A queue for holding results from worker threads that are ready to be processed on the game thread. */
	TQueue<FPathfindingResult> CompletedTasksQueue;

	/** A critical section to protect access to the CompletedTasksQueue. */
	FCriticalSection QueueLock;

	//~=============================================================================
	// Memory Pooling
	//~=============================================================================

	/** A lock to protect access to the memory pools. */
	FCriticalSection PoolLock;

	/** A collection of idle memory pools for grid-based pathfinding nodes. */
	TArray<FPathNodeMemoryPoolBase*> IdleGridPools;

	/** A collection of idle memory pools for octree-based pathfinding nodes. */
	TArray<FPathNodeMemoryPoolBase*> IdleOctreePools;
	
};