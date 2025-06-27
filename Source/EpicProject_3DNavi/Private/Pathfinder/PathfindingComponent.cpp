// Fill out your copyright notice in the Description page of Project Settings.


#include "Pathfinder/PathfindingComponent.h"

#include "Pathfinder/FenwickVoxelVolume.h"
#include "Pathfinder/PathfindingSubsystem.h"
#include "Pathfinder/Strategy/PathfindingStrategy.h"
#include "Pathfinder/VoxelVolume.h"

UPathfindingComponent::UPathfindingComponent()
{
	PrimaryComponentTick.bCanEverTick = true;

	// 변수 초기화
	CurrentPathIndex = -1; // 처음에는 이동하지 않음
	MovementSpeed = 500.f; // 기본 이동 속도
	CurrentState = EPathfindingState::Idle;
}

bool UPathfindingComponent::IsPathAheadValid() const
{
	// TargetVolume이 AFenwickVoxelVolume 타입이어야 IsPathSegmentWalkable 사용할 수 있음.
	AFenwickVoxelVolume* FenwickVolume = Cast<AFenwickVoxelVolume>(TargetVolume);

	if (!FenwickVolume || FoundPathIndices.Num() == 0 || Radius <= 0.f)
	{
		return true;
	}
	// 다음 목표 지점(Jump Point)의 복셀 인덱스
	const FIntVector NextTargetIndex = FoundPathIndices[CurrentPathIndex];

	// 'PathValidationLookAhead' 값만큼 앞의 경로 지점들을 순회
	const int32 LastCheckIndex = FMath::Min(CurrentPathIndex + PathValidationLookAhead, FoundPathIndices.Num() - 1);
	for (int32 i = CurrentPathIndex; i < LastCheckIndex; ++i)
	{
		const FIntVector NextNextTargetIndex = FoundPathIndices[CurrentPathIndex + 1];
		if (!FenwickVolume->IsPathSegmentWalkable(NextTargetIndex, NextNextTargetIndex, static_cast<int32>(Radius)))
		{
			return false;
		}
	}

	return true;
}

void UPathfindingComponent::BeginPlay()
{
	Super::BeginPlay();

	if (UWorld* World = GetWorld())
	{
		UGameInstance* GameInstance = World->GetGameInstance();
		if (!GameInstance) return;

		PathSubsystem = GameInstance->GetSubsystem<UPathfindingSubsystem>();
		if (!PathSubsystem)
		{
			UE_LOG(LogTemp, Error, TEXT("PathfindingComponent: PathfindingSubsystem not found!"));
			return;
		}

		const TArray<TObjectPtr<AVoxelVolume>>& Volumes = PathSubsystem->GetVoxelVolumes();
		if (Volumes.Num() > 0)
		{
			TargetVolume = Volumes[0];
		}
		else
		{
			UE_LOG(LogTemp, Warning, TEXT("PathfindingComponent: No VoxelVolumes available at BeginPlay."));
		}

		if (PathfindingStrategyClass)
		{
			PathfindingStrategy = NewObject<UPathfindingStrategy>(this, PathfindingStrategyClass);
			if (PathfindingStrategy)
			{
				PathfindingStrategy->Initialize(TargetVolume);
			}
		}
		else
		{
			UE_LOG(LogTemp, Error,
			       TEXT(
				       "PathfindingComponent on Actor '%s': PathfindingStrategyClass is not set! Please assign a strategy in the Details panel."
			       ), *GetOwner()->GetName());
		}
	}
}

void UPathfindingComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	Super::EndPlay(EndPlayReason);
	// 컴포넌트가 사라질 때, 실행 중일 수 있는 타이머를 확실하게 정리합니다. (크래시 방지)
	GetWorld()->GetTimerManager().ClearTimer(RepathCooldownTimer);
}

void UPathfindingComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	switch (CurrentState)
	{
	case EPathfindingState::Moving:
		HandleMovingState(DeltaTime);
		break;
		
	case EPathfindingState::RequestingPath:
		HandleRequestingPathState();
		break;
		
	case EPathfindingState::PathInvalidated:
	case EPathfindingState::Idle:
	default:
		// 이동 중이 아니거나 재탐색 대기 중일 때는 아무것도 하지 않음
		break;
	}
}

bool UPathfindingComponent::FindPathToLocation(const FVector& TargetLocation)
{
	// 이미 경로를 요청 중이라면, 새로운 요청 무시.
	if (CurrentState == EPathfindingState::RequestingPath)
	{
		return false;
	}

	if (!PathfindingStrategy || !TargetVolume) return false;

	// 재탐색을 위해 목표 위치 저장
	CurrentTargetLocation = TargetLocation;

	// 상태를 '경로 요청 중'으로 변경
	CurrentState = EPathfindingState::RequestingPath;
	FoundPath.Empty();
	FoundPathIndices.Empty();
	CurrentPathIndex = -1;

	UE_LOG(LogTemp, Log, TEXT("Requesting new path asynchronously..."));

	// --- 비동기 작업을 위한 데이터 복사 (스레드 안전성) ---
	TWeakObjectPtr<UPathfindingStrategy> StrategyPtr = PathfindingStrategy;
	const FIntVector StartIndex = TargetVolume->WorldToVoxelIndex(GetOwner()->GetActorLocation());
	const FIntVector EndIndex = TargetVolume->WorldToVoxelIndex(TargetLocation);
	const float AgentRadius = Radius;

	// --- 비동기 작업 시작 ---
	PathfindingTaskFuture = Async(EAsyncExecution::ThreadPool,
	                              [StrategyPtr, StartIndex, EndIndex, AgentRadius]() -> TArray<FIntVector>
	                              {
		                              // 이 람다 함수는 백그라운드 스레드에서 실행됩니다.
		                              TArray<FIntVector> PathIndices;

		                              // 약한 포인터가 유효한지 반드시 확인합니다.
		                              if (StrategyPtr.IsValid())
		                              {
			                              // 실제 길찾기 알고리즘 실행
			                              if (AgentRadius > 0.f)
			                              {
				                              StrategyPtr->FindPathWithStrategy(
					                              StartIndex, EndIndex, AgentRadius, PathIndices);
			                              }
			                              else
			                              {
				                              StrategyPtr->FindPathWithStrategy(StartIndex, EndIndex, PathIndices);
			                              }
		                              }
		                              return PathIndices;
	                              });

	return true; // 요청이 성공적으로 시작되었음을 의미 (경로를 찾았다는 의미가 아님)
}


void UPathfindingComponent::OnRepathTimerFired()
{
	FindPathToLocation(CurrentTargetLocation);
}

void UPathfindingComponent::HandleMovingState(float DeltaTime)
{
	// 1. 먼저 경로가 막혔는지 확인하고 처리
	if (CheckAndHandlePathInvalidation())
	{
		return;
	}

	// 2. 경로가 유효하면 계속해서 경로를 따라 이동
	FollowCurrentPath(DeltaTime);
}

bool UPathfindingComponent::CheckAndHandlePathInvalidation()
{
	if (!IsPathAheadValid())
	{
		// 경로가 막혔다!
		UE_LOG(LogTemp, Warning, TEXT("Path Invalidated! Stopping movement and waiting to repath..."));
		CurrentState = EPathfindingState::PathInvalidated;
		FoundPath.Empty(); // 이동을 즉시 멈추기 위해 경로를 비움
		CurrentPathIndex = -1;

		// 쿨다운 타이머를 설정합니다.
		GetWorld()->GetTimerManager().SetTimer(
			RepathCooldownTimer, this, &UPathfindingComponent::OnRepathTimerFired, RepathCooldown, false);
		
		return true; // 경로가 무효화
	}
	return false; // 경로 유효
}

void UPathfindingComponent::FollowCurrentPath(float DeltaTime)
{
	AActor* Actor = GetOwner();
	if (!Actor || CurrentPathIndex < 0 || FoundPath.Num() == 0)
	{
		CurrentState = EPathfindingState::Idle;
		return;
	}

	// 목표 지점에 도착했는지 확인하고 다음 지점으로 넘어가는 로직
	const FVector CurrentLocation = Actor->GetActorLocation();
	const FVector TargetPoint = FoundPath[CurrentPathIndex];
	if (FVector::DistSquared(CurrentLocation, TargetPoint) < FMath::Square(100.f))
	{
		CurrentPathIndex++;
		if (CurrentPathIndex >= FoundPath.Num())
		{
			CurrentState = EPathfindingState::Idle; // 이동 완료
			CurrentPathIndex = -1;
			UE_LOG(LogTemp, Log, TEXT("Movement Finished."));
			return;
		}
	}

	// 실제 이동 로직
	if (CurrentPathIndex >= 0 && CurrentPathIndex < FoundPath.Num())
	{
		const FVector Direction = (FoundPath[CurrentPathIndex] - CurrentLocation).GetSafeNormal();
		const FVector NewLocation = CurrentLocation + Direction * MovementSpeed * DeltaTime;
		Actor->SetActorLocation(NewLocation);
	}
}

void UPathfindingComponent::HandleRequestingPathState()
{
	// 비동기 작업이 완료되었는지 확인
	if (PathfindingTaskFuture.IsValid() && PathfindingTaskFuture.IsReady())
	{
		TArray<FIntVector> PathIndices = PathfindingTaskFuture.Get();
		FoundPath.Empty();

		if (PathIndices.Num() > 0)
		{
			// 경로 생성
			FoundPathIndices = PathIndices;
			for (const FIntVector& Index : FoundPathIndices)
			{
				FoundPath.Add(TargetVolume->VoxelIndexToWorld(Index));
			}
			CurrentPathIndex = 0;
			CurrentState = EPathfindingState::Moving;

			// 경로 디버그 그리기
			if (bDrawDebugPath)
			{
				for (int32 i = 0; i < FoundPath.Num() - 1; ++i)
				{
					DrawDebugLine(GetWorld(), FoundPath[i], FoundPath[i + 1], FColor::Green, false, 10.f, 0, 2.f);
				}
			}
		}
		else
		{
			// 경로 찾기 실패
			UE_LOG(LogTemp, Warning, TEXT("Async path not found."));
			FoundPathIndices.Empty();
			CurrentState = EPathfindingState::Idle;
		}
		PathfindingTaskFuture.Reset();
	}
}