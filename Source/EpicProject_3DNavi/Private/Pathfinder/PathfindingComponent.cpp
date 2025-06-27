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

	// 볼륨이 없거나, 펜윅 볼륨이 아니거나, 경로가 없거나, 반지름이 0 이하면 검사할 수 없습니다.
	if (!FenwickVolume || FoundPathIndices.Num() == 0 || Radius <= 0.f)
	{
		// 이 경우는 경로가 '유효하다'고 가정하여, 불필요한 재탐색을 방지합니다.
		return true; 
	}
	// 다음 목표 지점(Jump Point)의 복셀 인덱스
	const FIntVector NextTargetIndex = FoundPathIndices[CurrentPathIndex];

	// 'PathValidationLookAhead' 값만큼 앞의 경로 지점들을 순회합니다.
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

void UPathfindingComponent::TickComponent(float DeltaTime, ELevelTick TickType,
                                          FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	switch (CurrentState)
	{
	case EPathfindingState::Moving:
		{
			// --- 이동 중일 때의 로직 ---
			if (!IsPathAheadValid())
			{
				// 경로가 막혔다!
				UE_LOG(LogTemp, Warning, TEXT("Path Invalidated! Stopping movement and waiting to repath..."));
				CurrentState = EPathfindingState::PathInvalidated;
				FoundPath.Empty(); // 이동을 즉시 멈추기 위해 경로를 비움
				CurrentPathIndex = -1;

				// 쿨다운 타이머를 설정하여, 일정 시간 후에 OnRepathTimerFired 함수를 호출하도록 예약
				GetWorld()->GetTimerManager().SetTimer(
					RepathCooldownTimer,
					this,
					&UPathfindingComponent::OnRepathTimerFired,
					RepathCooldown,
					false // 반복하지 않음
				);
				return; // 이번 틱의 모든 로직 종료
			}

			// 경로가 유효하다면, 계속 이동
			AActor* Actor = GetOwner();
			if (!Actor || CurrentPathIndex < 0 || FoundPath.Num() == 0)
			{
				CurrentState = EPathfindingState::Idle;
				return;
			}
			
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

			if (CurrentPathIndex >= 0 && CurrentPathIndex < FoundPath.Num())
			{
				const FVector Direction = (FoundPath[CurrentPathIndex] - CurrentLocation).GetSafeNormal();
				const FVector NewLocation = CurrentLocation + Direction * MovementSpeed * DeltaTime;
				Actor->SetActorLocation(NewLocation);
			}
			break;
		}

	case EPathfindingState::PathInvalidated:
	case EPathfindingState::Idle:
	default:
		{
			// 이동 중이 아니거나 재탐색 대기 중일 때는 아무것도 하지 않음
			break;
		}
	}
}

bool UPathfindingComponent::FindPathToLocation(const FVector& TargetLocation)
{
	if (!PathfindingStrategy || !TargetVolume)
	{
		UE_LOG(LogTemp, Warning, TEXT("PathfindingComponent: Strategy or Volume is not set."));
		return false;
	}
	CurrentTargetLocation = TargetLocation;
	const FVector StartLocation = GetOwner()->GetActorLocation();

	// 월드 좌표를 복셀 인덱스로 변환
	const FIntVector StartIndex = TargetVolume->WorldToVoxelIndex(StartLocation);
	const FIntVector EndIndex = TargetVolume->WorldToVoxelIndex(TargetLocation);

	if (!TargetVolume->IsIndexValid(StartIndex) || !TargetVolume->IsIndexValid(EndIndex))
	{
		UE_LOG(LogTemp, Warning, TEXT("PathfindingComponent: Start or End index is out of bounds."));
		return false;
	}

	bool bPathFound = false;
	if (bUseFenwickTree && Radius >= 0)
	{
		bPathFound = PathfindingStrategy->FindPathWithStrategy(StartIndex, EndIndex, Radius, FoundPathIndices);
	}
	else
	{
		bPathFound = PathfindingStrategy->FindPathWithStrategy(StartIndex, EndIndex, FoundPathIndices);
	}

	FoundPath.Empty();
	CurrentPathIndex = -1; // 우선 이동 중지 상태로 초기화

	if (bPathFound && FoundPathIndices.Num() > 0)
	{
		UE_LOG(LogTemp, Log, TEXT("Path found with %d points. Starting movement."), FoundPathIndices.Num());
		CurrentState = EPathfindingState::Moving; 
		for (const FIntVector& Index : FoundPathIndices)
		{
			FoundPath.Add(TargetVolume->VoxelIndexToWorld(Index));
		}

		CurrentPathIndex = 0;

		// 디버그 드로잉
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
		UE_LOG(LogTemp, Warning, TEXT("Path not found."));
		FoundPathIndices.Empty();
		CurrentState = EPathfindingState::Idle;
	}

	return bPathFound;
}

void UPathfindingComponent::OnRepathTimerFired()
{
	FindPathToLocation(CurrentTargetLocation);
}