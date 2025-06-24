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
			UE_LOG(LogTemp, Error, TEXT("PathfindingComponent on Actor '%s': PathfindingStrategyClass is not set! Please assign a strategy in the Details panel."), *GetOwner()->GetName());
		}
	}

}

void UPathfindingComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// 1. 이동 중이 아니거나, 움직일 대상이 없거나, 경로가 없으면 아무것도 안 함
	if (CurrentPathIndex < 0 || !MovableMeshComponent || FoundPath.Num() == 0)
	{
		return;
	}

	// 2. 현재 위치와 목표 지점 설정
	const FVector CurrentLocation = MovableMeshComponent->GetComponentLocation();
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
		MovableMeshComponent->SetWorldLocation(NewLocation);
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
	CurrentPathIndex = -1; // 우선 이동 중지 상태로 초기화
	
	if (bPathFound && PathIndices.Num() > 0)
	{
		UE_LOG(LogTemp, Log, TEXT("Path found with %d points. Starting movement."), PathIndices.Num());
		for (const FIntVector& Index : PathIndices)
		{   
			FoundPath.Add(TargetVolume->VoxelIndexToWorld(Index));
		}

		// ==================================================
		// 이동 시작 신호!
		// ==================================================
		// 첫 번째 지점(인덱스 0)을 목표로 설정해서 Tick에서 이동을 시작하게 함
		CurrentPathIndex = 0;

		// 디버그 드로잉
		if (bDrawDebugPath && FoundPath.Num() > 1)
		{
			for (int32 i = 0; i < FoundPath.Num() - 1; ++i)
			{
				DrawDebugLine(GetWorld(), FoundPath[i], FoundPath[i + 1], FColor::Green, true, 10.f, 0, 5.f);
			}
		}
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("Path not found."));
	}

	return bPathFound;
}

