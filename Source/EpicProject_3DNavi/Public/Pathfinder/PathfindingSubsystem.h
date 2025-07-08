// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Pathfinder/FPathfindingRequest.h"
#include "Subsystems/WorldSubsystem.h"
#include "Pathfinder/FPathfindingWorker.h" // ★★★ 새로 만든 Worker 클래스의 헤더를 반드시 포함해야 합니다.
#include "PathfindingSubsystem.generated.h"


class AVoxelVolumeBase;
class UPathfindingComponent;
class UPathfindingStrategy;
class FPathfindingWorker;

UCLASS()
class EPICPROJECT_3DNAVI_API UPathfindingSubsystem : public UWorldSubsystem
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

	TQueue<FPathfindingRequest, EQueueMode::Mpsc> PathRequestQueue;

	/**
	 * 실제 길찾기 계산을 수행하는 전용 작업자 스레드 객체.
	 * TUniquePtr로 소유하여 서브시스템이 파괴될 때 자동으로 메모리를 관리합니다.
	 */
	TUniquePtr<FPathfindingWorker> PathfindingWorker;

	/**
	 * 고유한 요청 ID를 발급하기 위한 스레드 안전 카운터.
	 * 여러 스레드에서 동시에 ID를 요청해도 안전합니다.
	 */
	FThreadSafeCounter NextRequestID;

};
