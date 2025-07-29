// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "Tickable.h"
#include "Async/AsyncWork.h"
#include "Containers/Queue.h"
#include "HAL/ThreadSafeCounter.h"
#include "HAL/CriticalSection.h"
#include "GameFramework/Actor.h"
#include "Utility/Parallel/PathfindingRequest.h"
#include "Subsystem/OctreeNavSubsystem.h"
#include "NavVolume/OctreeVoxelVolume.h"
#include "PathfindingSubsystem.generated.h"

class UWorldNavigationDataAsset;
class APathfindingStreamingProbe;

/**
 * Represents a request for a path that may span multiple volumes.
 */
struct FHierarchicalPathRequest
{
	int32 RequestID = -1;
	TWeakObjectPtr<UPathfindingComponent> RequestingComponent;
	FVector StartLocation = FVector::ZeroVector;
	FVector EndLocation = FVector::ZeroVector;
	float AgentRadius = 0.f;
	TWeakObjectPtr<UPathfindingStrategy> Strategy;
	FWorldPathfindingResultDelegate OnHierarchicalPathComplete;
};

enum class EHierarchicalTaskState : uint8
{
	Idle,                       // Initial state
	WaitingForVolumeToLoad,     // Waiting for the next volume to stream in
	ReadyToFindLocalPath,       // Volume loaded, ready to begin local pathfinding
	FindingLocalPath,           // Local pathfinding task is in progress
	Complete,                   // All segments are complete
	Failed                      // The task has failed
};

/**
 * Manages the state of an ongoing hierarchical pathfinding task.
 */
struct FHierarchicalPathfindingTask
{
	FHierarchicalPathRequest OriginalRequest;
	TArray<FVector> CombinedWorldPath;
	TArray<TPair<TSoftObjectPtr<UOctreeDataAsset>, int32>> HighLevelDataAssetPath;
	EHierarchicalTaskState State = EHierarchicalTaskState::Idle;
	int32 CurrentSegmentIndex = 0;
	int32 LastLocalRequestID = -1;

	FHierarchicalPathfindingTask(const FHierarchicalPathRequest& InRequest)
		: OriginalRequest(InRequest)
	{
	}
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

	int32 RequestHierarchicalPath(const UObject* WorldContextObject, FHierarchicalPathRequest& Request);

	//~=============================================================================
	// Tick Interface
	//~=============================================================================

	virtual void Tick(float DeltaTime) override;
	virtual TStatId GetStatId() const override;

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

	void OnPathfindingCompleted(const FPathfindingRequest& Request, const TArray<FVector>& WorldPath, bool bSuccess);
	void OnHierarchicalSegmentCompleted(int32 RequestID, const TArray<FVector>& SegmentPath, bool bSuccess);

	//~=============================================================================
	// Threading & Task Management
	//~=============================================================================

	FQueuedThreadPool* PathfindingThreadPool = nullptr;

	UPROPERTY(Config)
	int32 CurrentNumWorkerThreads = 5;

	UPROPERTY(Config)
	int32 MaxCallbacksPerFrame = 100;

	FThreadSafeCounter NextRequestID;
	TQueue<FPathfindingResult, EQueueMode::Mpsc> CompletedTasksQueue;
	TMap<int32, FHierarchicalPathfindingTask> ActiveHierarchicalTasks;

	//~=============================================================================
	// Hierarchical Pathfinding Internals
	//~=============================================================================

	bool FindHighLevelPath(const UObject* WorldContextObject, const FHierarchicalPathRequest& Request, TArray<TPair<TSoftObjectPtr<UOctreeDataAsset>, int32>>& OutDataAssetPath);
	void ProcessNextHierarchicalSegment(const UObject* WorldContextObject, FHierarchicalPathfindingTask& Task);

	//~=============================================================================
	// Memory Pooling
	//~=============================================================================

	FCriticalSection PoolLock;
	TArray<FPathNodeMemoryPoolBase*> IdleGridPools;
	TArray<FPathNodeMemoryPoolBase*> IdleOctreePools;

private:
	//~=============================================================================
	// Level Transition Handling 
	//~=============================================================================

	UPROPERTY()
	TObjectPtr<UWorldNavigationDataAsset> CurrentWorldNavData;

	UPROPERTY()
	TMap<int32, TObjectPtr<APathfindingStreamingProbe>> ActiveStreamingProbes;

	void CreateStreamingProbeForRequest(const UObject* WorldContextObject, int32 RequestID, const FVector& Location, float Radius);
	void DestroyStreamingProbeForRequest(const UObject* WorldContextObject, int32 RequestID);
};