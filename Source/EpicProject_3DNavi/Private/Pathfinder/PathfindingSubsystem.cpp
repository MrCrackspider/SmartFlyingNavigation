// Fill out your copyright notice in the Description page of Project Settings.

#include "Pathfinder/PathfindingSubsystem.h"
#include "Pathfinder/Strategy/PathfindingStrategy.h"
#include "Pathfinder/FenwickVoxelVolume.h"
#include "Engine/World.h"
#include "Kismet/GameplayStatics.h"

void UPathfindingSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
    Super::Initialize(Collection);
    NextRequestID = 0;
}

void UPathfindingSubsystem::Deinitialize()
{
    // 월드가 파괴될 때 타이머 정리
    if (UWorld* World = GetWorld())
    {
        World->GetTimerManager().ClearTimer(QueueProcessTimerHandle);
    }
    Super::Deinitialize();
}

void UPathfindingSubsystem::OnWorldBeginPlay(UWorld& InWorld)
{
    Super::OnWorldBeginPlay(InWorld);
    // 게임이 시작되면 0.1초마다 큐를 처리하는 타이머를 설정합니다. (매 틱보다 부하가 적음)
    InWorld.GetTimerManager().SetTimer(QueueProcessTimerHandle, this, &UPathfindingSubsystem::ProcessPathfindingQueue, 0.1f, true);
}

void UPathfindingSubsystem::Tick(float DeltaTime)
{
    // 매 틱마다 큐를 처리하는 함수를 호출합니다.
    ProcessPathfindingQueue();
}

// GetStatId는 통계(stat) 시스템에서 이 틱을 식별하기 위해 필요합니다.
TStatId UPathfindingSubsystem::GetStatId() const
{
    // 특별한 stat 그룹을 사용하지 않는다면 기본값으로 충분합니다.
    RETURN_QUICK_DECLARE_CYCLE_STAT(UPathfindingSubsystem, STATGROUP_Tickables);
}


int32 UPathfindingSubsystem::RequestPath(const FPathfindingRequest& Request)
{
    if (!Request.Strategy.IsValid() || !Request.RequestingComponent.IsValid())
    {
        UE_LOG(LogTemp, Error, TEXT("Pathfinding request failed: Strategy or RequestingComponent is invalid."));
        return -1;
    }

    // 새 요청에 고유 ID 할당
    FPathfindingRequest NewRequest = Request;
    NewRequest.RequestID = NextRequestID++;

    // 큐에 요청 추가 (나중에 우선순위 큐로 변경 가능)
    RequestQueue.Add(NewRequest);

    UE_LOG(LogTemp, Log, TEXT("Pathfinding request %d queued."), NewRequest.RequestID);
    return NewRequest.RequestID;
}

void UPathfindingSubsystem::CancelPathRequest(int32 RequestID)
{
    if (RequestID < 0) return;

    // 대기 큐에서 제거
    RequestQueue.RemoveAll([RequestID](const FPathfindingRequest& Request)
    {
        return Request.RequestID == RequestID;
    });

    // 현재 활성 태스크 목록에 있다면, 취소 플래그를 설정하는 로직을 추가할 수 있습니다.
    // (더 복잡한 구현이 필요하므로 여기서는 생략)
    UE_LOG(LogTemp, Log, TEXT("Pathfinding request %d cancelled."), RequestID);
}

void UPathfindingSubsystem::ProcessPathfindingQueue()
{
    // 현재 처리 중인 태스크가 너무 많거나, 큐가 비어있으면 아무것도 하지 않음
    if (ActiveRequests.Num() >= MaxTasksPerFrame || RequestQueue.IsEmpty())
    {
        return;
    }
    // 처리할 요청을 큐에서 꺼냄
    FPathfindingRequest Request = RequestQueue[0];
    RequestQueue.RemoveAt(0);

    // 처리 시작을 알림
    ActiveRequests.Add(Request.RequestID, Request); // 이 줄로 교체합니다.
    UE_LOG(LogTemp, Log, TEXT("Processing pathfinding request %d..."), Request.RequestID);

    // 비동기 태스크 실행
    Async(EAsyncExecution::ThreadPool, [Request, this]()
    {
        TArray<FIntVector> PathIndices;
        bool bSuccess = false;
        if (Request.Strategy.IsValid())
        {
            // 실제 길찾기 로직 실행 (이 부분은 스레드에서 실행됨)
            bSuccess = Request.Strategy->FindPathWithStrategy(Request.StartIndex, Request.EndIndex, Request.AgentRadiusInVoxels, PathIndices);
        }

        // 결과 처리를 위해 게임 스레드로 다시 작업을 보냄
        AsyncTask(ENamedThreads::GameThread, [this, RequestID = Request.RequestID, PathIndices, bSuccess]()
        {
            // OnPathfindingTaskComplete 함수는 게임 스레드에서 안전하게 호출됨
            OnPathfindingTaskComplete(RequestID, PathIndices, bSuccess);
        });
    });
}

void UPathfindingSubsystem::OnPathfindingTaskComplete(int32 RequestID, TArray<FIntVector> ResultPath, bool bSuccess)
{
    // 1. 맵에서 해당 ID를 가진 요청 정보를 찾습니다.
    //    Find()는 해당 키가 존재하면 값에 대한 포인터를, 없으면 nullptr를 반환합니다.
    FPathfindingRequest* FoundRequest = ActiveRequests.Find(RequestID);

    // 2. 요청 정보를 찾았는지 확인합니다.
    if (FoundRequest)
    {
        // 3. 찾은 요청서에 저장된 델리게이트를 실행하여 결과를 전달합니다.
        FoundRequest->OnPathfindingComplete.ExecuteIfBound(ResultPath, bSuccess);

        UE_LOG(LogTemp, Log, TEXT("Result for request %d delivered. Success: %s"), RequestID, bSuccess ? TEXT("True") : TEXT("False"));
        
        // 4. 이제 작업이 끝났으므로 맵에서 이 요청을 제거합니다.
        ActiveRequests.Remove(RequestID);
    }
    else
    {
        // 맵에서 해당 ID를 찾지 못한 경우 (이런 일이 발생하면 안 됨)
        UE_LOG(LogTemp, Warning, TEXT("Could not find active request with ID %d to deliver result."), RequestID);
    }
}
