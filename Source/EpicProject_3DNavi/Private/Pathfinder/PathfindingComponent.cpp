// Fill out your copyright notice in the Description page of Project Settings.


#include "Pathfinder/PathfindingComponent.h"
#include "Pathfinder/PathfindingSubsystem.h"
#include "Pathfinder/Strategy/PathfindingStrategy.h"
#include "Pathfinder/VoxelVolume.h"

UPathfindingComponent::UPathfindingComponent()
{
	PrimaryComponentTick.bCanEverTick = true;

	// 변수 초기화
	CurrentPathIndex = -1; // 처음에는 이동하지 않음
	MovementSpeed = 500.f; // 기본 이동 속도
}

void UPathfindingComponent::BeginPlay()
{
	Super::BeginPlay();

	//UGameInstance* GameInstance = GEngine->GetWorld()->GetGameInstance();
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

void UPathfindingComponent::TickComponent(float DeltaTime, ELevelTick TickType,
                                          FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	AActor* Actor = GetOwner();
	// 1. 이동 중이 아니거나, 움직일 대상이 없거나, 경로가 없으면 아무것도 안 함
	if (CurrentPathIndex < 0 || Actor == nullptr || FoundPath.Num() == 0)
	{
		return;
	}

	// 2. 현재 위치와 목표 지점 설정

	const FVector CurrentLocation = Actor->GetActorLocation();
	const FVector TargetPoint = FoundPath[CurrentPathIndex];

	// 3. 목표 지점에 거의 도착했는지 확인
	if (FVector::DistSquared(CurrentLocation, TargetPoint) < FMath::Square(100.f)) // DistSquared가 더 빠름
	{
		// 도착했다면 다음 목표 지점으로 인덱스 증가
		CurrentPathIndex++;

		// 4. 경로의 끝에 도달했는지 확인
		if (CurrentPathIndex >= FoundPath.Num())
		{
			// 이동 완료: 정지
			CurrentPathIndex = -1;
			UE_LOG(LogTemp, Log, TEXT("Movement Finished."));
			return; // 이번 틱에서는 더 이상 움직이지 않음
		}
	}

	// 5. 아직 이동 중이라면 목표 방향으로 움직임
	if (CurrentPathIndex >= 0)
	{
		// 목표를 향하는 방향 벡터 계산 (크기를 1로 정규화)
		const FVector Direction = (FoundPath[CurrentPathIndex] - CurrentLocation).GetSafeNormal();

		// 이번 프레임에 움직일 위치 계산 (방향 * 속도 * 시간)
		const FVector NewLocation = CurrentLocation + Direction * MovementSpeed * DeltaTime;

		// MovableMeshComponent의 월드 위치를 업데이트
		Actor->SetActorLocation(NewLocation);
	}
}

bool UPathfindingComponent::FindPathToLocation(const FVector& TargetLocation)
{
	if (!PathfindingStrategy || !TargetVolume)
	{
		UE_LOG(LogTemp, Warning, TEXT("PathfindingComponent: Strategy or Volume is not set."));
		return false;
	}

	const FVector StartLocation = GetOwner()->GetActorLocation();

	// 월드 좌표를 복셀 인덱스로 변환
	const FIntVector StartIndex = TargetVolume->WorldToVoxelIndex(StartLocation);
	const FIntVector EndIndex = TargetVolume->WorldToVoxelIndex(TargetLocation);

	if (!TargetVolume->IsIndexValid(StartIndex) || !TargetVolume->IsIndexValid(EndIndex))
	{
		UE_LOG(LogTemp, Warning, TEXT("PathfindingComponent: Start or End index is out of bounds."));
		return false;
	}

	TArray<FIntVector> PathIndices;
	const bool bPathFound = PathfindingStrategy->FindPathWithStrategy(StartIndex, EndIndex, PathIndices);

	FoundPath.Empty();
	TArray<FVector> RawPath;
	CurrentPathIndex = -1; // 우선 이동 중지 상태로 초기화

	if (bPathFound && PathIndices.Num() > 0)
	{
		UE_LOG(LogTemp, Log, TEXT("Path found with %d points. Starting movement."), PathIndices.Num());
		for (const FIntVector& Index : PathIndices)
		{
			RawPath.Add(TargetVolume->VoxelIndexToWorld(Index));
		}
		const TArray<FVector> SmoothedPath = ApplyStringPulling(RawPath);
		FoundPath = RawPath;
		// ==================================================
		// 이동 시작 신호!
		// ==================================================
		// 첫 번째 지점(인덱스 0)을 목표로 설정해서 Tick에서 이동을 시작하게 함
		CurrentPathIndex = 0;

		// 디버그 드로잉
		if (bDrawDebugPath && FoundPath.Num() > 1)
		{
			for (int32 i = 0; i < RawPath.Num() - 1; ++i)
			{
				DrawDebugLine(GetWorld(), RawPath[i], RawPath[i + 1], FColor::White, false, 10.f, 0, 2.f);
			}
			// 부드러워진 경로 (녹색)
			for (int32 i = 0; i < FoundPath.Num() - 1; ++i)
			{
				DrawDebugLine(GetWorld(), FoundPath[i], FoundPath[i + 1], FColor::Green, false, 10.f, 0, 5.f);
			}
		}
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("Path not found."));
	}

	return bPathFound;
}

//To Do. 아래 코드 수정 필요함. 보정용 함수인데 대충짠 코드이기 때문에 무조건 수정하거나 없앨 것
TArray<FVector> UPathfindingComponent::ApplyStringPulling(const TArray<FVector>& RawPath) const
{
	// 경로에 점이 2개 미만이면 단순화할 필요가 없음
	if (RawPath.Num() < 2)
	{
		return RawPath;
	}

	TArray<FVector> SmoothedPath;
	SmoothedPath.Add(RawPath[0]); // 시작점은 항상 포함

	int32 CurrentIndex = 0;
	while (CurrentIndex < RawPath.Num() - 1)
	{
		int32 LookAheadIndex = CurrentIndex + 1;
		int32 LastVisibleIndex = CurrentIndex; // 현재까지 장애물 없이 도달 가능한 마지막 인덱스

		while (LookAheadIndex < RawPath.Num())
		{
			FHitResult HitResult;
			// CurrentIndex의 위치에서 LookAheadIndex의 위치까지 광선을 쏴서 장애물이 있는지 확인
			const bool bHit = GetWorld()->LineTraceSingleByChannel(
				HitResult,
				RawPath[CurrentIndex], // 시작점
				RawPath[LookAheadIndex], // 끝점
				ECC_Visibility // 충돌을 검사할 채널 (보통 ECC_Visibility나 ECC_WorldStatic을 사용)
			);
			
			if (bHit)
			{
				// 장애물에 부딪혔다면, 바로 이전 지점까지만 직선으로 이동 가능했다는 의미
				// 루프를 중단하고 LastVisibleIndex를 다음 웨이포인트로 사용
				break;
			}
			else
			{
				// 장애물이 없다면, 이 지점까지는 직선으로 올 수 있음. LastVisibleIndex를 업데이트하고 계속 탐색
				LastVisibleIndex = LookAheadIndex;
				LookAheadIndex++;
			}
		}

		// 장애물 없이 갈 수 있었던 가장 먼 지점을 새로운 경로에 추가
		// 만약 바로 다음 점조차 장애물에 막혔다면, LastVisibleIndex는 CurrentIndex와 같을 것이고,
		// 이 경우 바로 다음 점(CurrentIndex + 1)을 경로에 추가해야 함.
		if (LastVisibleIndex == CurrentIndex)
		{
			LastVisibleIndex = CurrentIndex + 1;
		}
		
		SmoothedPath.Add(RawPath[LastVisibleIndex]);
		CurrentIndex = LastVisibleIndex; // 새로운 시작점으로 설정
	}

	return SmoothedPath;
}