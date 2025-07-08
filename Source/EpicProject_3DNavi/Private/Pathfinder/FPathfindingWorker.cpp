// Fill out your copyright notice in the Description page of Project Settings.


#include "Pathfinder/FPathfindingRequest.h"
#include "Pathfinder/Strategy/PathfindingStrategy.h"
#include "FPathfindingWorker.h"
#include "Pathfinder/PathfindingSubsystem.h"

FPathfindingWorker::FPathfindingWorker(UPathfindingSubsystem* InSubsystem, TQueue<FPathfindingRequest, EQueueMode::Mpsc>* InRequestQueue)
	: bStopThread(false)
	, PathfindingSubsystemPtr(InSubsystem)
	, RequestQueue(InRequestQueue)
{
	// FRunnableThread::Create를 호출하여 실제 스레드를 생성하고 즉시 실행합니다.
	// "this"는 FRunnable 인터페이스를 구현한 이 객체 자신을 의미합니다.
	// "PathfindingWorkerThread"는 디버깅 시 표시될 스레드의 이름입니다.
	Thread = FRunnableThread::Create(this, TEXT("PathfindingWorkerThread"), 0, TPri_Normal);
}

// --- 소멸자 ---
FPathfindingWorker::~FPathfindingWorker()
{
	// 만약 스레드 포인터가 여전히 유효하다면, EnsureCompletion이 호출되지 않았을 수 있습니다.
	// 확실하게 스레드를 정리하고 삭제합니다.
	if (Thread)
	{
		delete Thread;
		Thread = nullptr;
	}
}

// --- FRunnable 인터페이스 구현 ---

bool FPathfindingWorker::Init()
{
	// 스레드 시작 전 초기화 작업. 특별히 할 일이 없으면 true만 반환합니다.
	UE_LOG(LogTemp, Log, TEXT("PathfindingWorker: Thread has been initialized."));
	return true;
}

uint32 FPathfindingWorker::Run()
{
	// 이 while 루프가 작업자 스레드의 본체입니다.
	// bStopThread가 true가 될 때까지 계속 반복됩니다.
	while (!bStopThread)
	{
		FPathfindingRequest CurrentRequest;
		// 1. 스레드 안전 큐에서 작업을 꺼내옵니다. 성공하면 true를 반환합니다.
		if (RequestQueue->Dequeue(CurrentRequest))
		{
			// 2. 길찾기 계산을 수행합니다. 이 작업은 백그라운드 스레드에서 실행됩니다.
			TArray<FIntVector> PathIndices;
			bool bSuccess = false;
			if (CurrentRequest.Strategy.IsValid())
			{
				// 실제 계산 수행
				bSuccess = CurrentRequest.Strategy->FindPathWithStrategy(CurrentRequest,PathIndices);
			}
			
			// 3. 계산 결과를 게임 스레드에서 처리하도록 작업을 예약합니다.
			//    델리게이트를 값으로 캡쳐하여, Subsystem의 생명주기와 상관없이 안전하게 호출합니다.
			AsyncTask(ENamedThreads::GameThread, [OnComplete = CurrentRequest.OnPathfindingComplete, PathIndices, bSuccess]()
			{
				// 이 람다 함수는 게임 스레드에서 실행됩니다.
				OnComplete.ExecuteIfBound(PathIndices, bSuccess);
			});
		}
		else
		{
			// 4. 큐가 비어있으면, CPU를 낭비하지 않도록 잠시 대기합니다.
			FPlatformProcess::Sleep(0.01f); // 10ms
		}
	}

	// 루프가 종료되면 0을 반환하여 스레드가 성공적으로 끝났음을 알립니다.
	return 0;
}

void FPathfindingWorker::Stop()
{
	// 외부에서 스레드 종료를 요청하면, Run()의 while 루프를 멈추기 위해 플래그를 설정합니다.
	bStopThread = true;
}

void FPathfindingWorker::Exit()
{
	// 스레드가 완전히 종료된 후, 마지막 정리 작업을 수행합니다.
	UE_LOG(LogTemp, Log, TEXT("PathfindingWorker: Thread has been exited."));
}

// --- 공개 헬퍼 함수 ---

void FPathfindingWorker::EnsureCompletion()
{
	// 1. 스레드에 종료를 요청합니다.
	Stop();

	// 2. 스레드가 Run() 함수를 완전히 끝내고 종료될 때까지 현재 스레드(보통 게임 스레드)를 대기시킵니다.
	if (Thread)
	{
		Thread->WaitForCompletion();
	}
}
