// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "Pathfinder/PathfindingAsyncTask.h"
#include "PathfindingSubsystem.generated.h"


class AVoxelVolumeBase;
class UPathfindingComponent;
class UPathfindingStrategy;
class FPathfindingWorker;
class FPathfindingAsyncTask;

UCLASS()
class NAVIGATION3D_API UPathfindingSubsystem : public UWorldSubsystem
{
	GENERATED_BODY()

public:
	
	/**
	 * 길찾기를 요청합니다. 컴포넌트는 이 함수를 호출합니다.
	 * @param Request 길찾기에 필요한 모든 정보를 담은 요청 구조체
	 * @return 요청이 성공적으로 접수되면 고유한 RequestID를, 실패하면 -1을 반환합니다.
	 */
	UFUNCTION(BlueprintCallable, Category = "Pathfinding")
	int32 RequestPath(FPathfindingRequest& Request);
	
protected:
	// --- Subsystem Lifecycle ---
	virtual void Initialize(FSubsystemCollectionBase& Collection) override;
	virtual void Deinitialize() override;
private:
	/**
	 * 고유한 요청 ID를 발급하기 위한 스레드 안전 카운터.
	 * 여러 스레드에서 동시에 ID를 요청해도 안전합니다.
	 */
	FThreadSafeCounter NextRequestID;

	/** 현재 실행 중인 모든 비동기 작업을 추적하는 배열. */
	TArray<FAsyncTask<FPathfindingAsyncTask>*> ActiveTasks;

	/** ActiveTasks 배열에 대한 동시 접근을 막기 위한 크리티컬 섹션. */
	FCriticalSection TasksCriticalSection;

};
