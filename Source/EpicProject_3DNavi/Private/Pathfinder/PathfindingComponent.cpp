// Fill out your copyright notice in the Description page of Project Settings.


#include "Pathfinder/PathfindingComponent.h"
#include "Pathfinder/PathfindingSubsystem.h"
#include "Pathfinder/Strategy/PathfindingStrategy.h"
#include "Pathfinder/VoxelVolume.h"

UPathfindingComponent::UPathfindingComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	
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
	if (bPathFound)
	{
		UE_LOG(LogTemp, Log, TEXT("Path found with %d points."), PathIndices.Num());
		// 인덱스 경로를 월드 좌표 경로로 변환
		for (const FIntVector& Index : PathIndices)
		{
			FoundPath.Add(TargetVolume->VoxelIndexToWorld(Index));
		}

		if (bDrawDebugPath && FoundPath.Num() > 1)
		{
			for (int32 i = 0; i < FoundPath.Num() - 1; ++i)
			{
				DrawDebugLine(GetWorld(), FoundPath[i], FoundPath[i + 1], FColor::Blue, true, 5.f, 0, 5.f);
			}
		}
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("Path not found."));
	}

	return bPathFound;
}

