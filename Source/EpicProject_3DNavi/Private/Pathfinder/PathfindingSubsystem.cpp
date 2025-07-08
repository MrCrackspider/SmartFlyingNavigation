// Fill out your copyright notice in the Description page of Project Settings.

#include "Pathfinder/PathfindingSubsystem.h"


// --- 1. 서브시스템 초기화 ---
// 서브시스템이 생성될 때 단 한 번 호출됩니다.
void UPathfindingSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);

	// NextRequestID 카운터를 0으로 초기화합니다.
	NextRequestID.Set(0);

	// FPathfindingWorker 객체를 생성합니다.
	// Worker 생성자 내부에서 실제 백그라운드 스레드가 생성되고 즉시 실행을 시작합니다.
	PathfindingWorker = MakeUnique<FPathfindingWorker>(this, &PathRequestQueue);

	UE_LOG(LogTemp, Log, TEXT("PathfindingSubsystem Initialized and Worker Thread Started."));
}

// --- 2. 서브시스템 소멸 ---
// 서브시스템이 파괴될 때 (예: 맵 전환, 게임 종료) 단 한 번 호출됩니다.
void UPathfindingSubsystem::Deinitialize()
{
	// Worker 객체가 유효한지 확인합니다.
	if (PathfindingWorker)
	{
		// 2. 그 다음에 스레드 종료를 요청하고 기다립니다.
		PathfindingWorker->EnsureCompletion();
		PathfindingWorker.Reset();
	}

	UE_LOG(LogTemp, Log, TEXT("PathfindingSubsystem Deinitialized and Worker Thread Stopped."));

	Super::Deinitialize();
}

// --- 3. 길찾기 요청 접수 (Public API) ---
// PathfindingComponent에서 이 함수를 호출합니다.
int32 UPathfindingSubsystem::RequestPath(FPathfindingRequest& Request)
{
	// 요청의 유효성을 검사합니다.
	if (!Request.Strategy.IsValid() || !Request.RequestingComponent.IsValid())
	{
		UE_LOG(LogTemp, Error, TEXT("Pathfinding request failed: Strategy or RequestingComponent is invalid."));
		return -1;
	}

	// 스레드에 안전한 방식으로 고유 ID를 생성하여 요청에 부여합니다.
	Request.RequestID = NextRequestID.Increment();

	// 스레드 안전 큐에 요청을 넣습니다.
	// 백그라운드에서 대기 중인 Worker가 이 요청을 알아서 가져가 처리할 것입니다.
	PathRequestQueue.Enqueue(Request);

	UE_LOG(LogTemp, Log, TEXT("Pathfinding request %d has been queued."), Request.RequestID);
	return Request.RequestID;
}
