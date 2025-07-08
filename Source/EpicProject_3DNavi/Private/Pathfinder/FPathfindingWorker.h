// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"

struct FPathfindingRequest;
class UPathfindingSubsystem;

class EPICPROJECT_3DNAVI_API FPathfindingWorker : public FRunnable
{
public:
	/**
		 * 생성자
		 * @param InSubsystem 이 Worker를 소유한 서브시스템
		 * @param InRequestQueue 작업을 가져올 큐에 대한 포인터
		 */
	FPathfindingWorker(UPathfindingSubsystem* InSubsystem, TQueue<FPathfindingRequest, EQueueMode::Mpsc>* InRequestQueue);

	/**
	 * 소멸자
	 * 스레드가 안전하게 종료되었는지 확인하고 리소스를 정리합니다.
	 */
	virtual ~FPathfindingWorker() override;

	// FRunnable 인터페이스 구현부
	// 이 함수들은 FRunnableThread에 의해 호출됩니다.
	
	/** 스레드 초기화. 스레드 시작 직전에 호출됩니다. */
	virtual bool Init() override;

	/** 스레드의 메인 루프. 이 함수가 끝나면 스레드도 종료됩니다. */
	virtual uint32 Run() override;

	/** 외부에서 스레드 종료를 요청할 때 호출됩니다. */
	virtual void Stop() override;

	/** 스레드 종료 직후, 소멸 직전에 호출됩니다. */
	virtual void Exit() override;

	/**
	 * 이 Worker의 스레드가 완전히 종료될 때까지 현재 스레드를 대기시킵니다.
	 * 서브시스템의 Deinitialize에서 호출하여 안전한 종료를 보장합니다.
	 */
	void EnsureCompletion();
private:
	/** 스레드를 중지시키기 위한 플래그. FThreadSafeBool은 여러 스레드에서 안전하게 접근할 수 있습니다. */
	FThreadSafeBool bStopThread;

	/** 이 FRunnable을 실행하는 실제 스레드에 대한 포인터. */
	FRunnableThread* Thread;

	/**
	 * 이 Worker를 소유한 서브시스템에 대한 약한 참조 포인터.
	 * Worker가 살아있는 동안 Subsystem이 파괴될 수 있는 경우를 대비해 TWeakObjectPtr를 사용합니다.
	 */
	TWeakObjectPtr<UPathfindingSubsystem> PathfindingSubsystemPtr;

	/**
	 * 처리할 길찾기 요청이 담긴 큐.
	 * Subsystem과 이 포인터를 공유하여, Subsystem은 Enqueue만 하고 Worker는 Dequeue만 합니다.
	 */
	TQueue<FPathfindingRequest, EQueueMode::Mpsc>* RequestQueue;
};
