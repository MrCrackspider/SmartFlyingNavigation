// Fill out your copyright notice in the Description page of Project Settings.
#include "EpicProject_3DNavi/Public/NaviController.h"
#include "EpicProject_3DNavi/Voxel/JPSPathFinding.h"
#include "EpicProject_3DNavi/Voxel/NavigationVolume.h"

// Sets default values
ANaviController::ANaviController()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void ANaviController::BeginPlay()
{
	Super::BeginPlay();

}

// Called every frame
void ANaviController::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	
	if (NaviVolume && GoalActor)
	{
		UVoxelGrid* VoxelGrid = NaviVolume->GetVoxelGrid();
		
		FVector StartLocation = GetActorLocation();
		FVector EndLocation = GoalActor->GetActorLocation();

		TArray<FVector> PathPoints = UJPSPathFinding::FindPathJPS(VoxelGrid, StartLocation, EndLocation);

		if (PathPoints.Num() > 1) // 점이 2개 이상 있어야 선을 그릴 수 있음
		{
			// UE_LOG(LogTemp, Error, TEXT("%d"), PathPoints.Num());

			for (int32 i = 0; i < PathPoints.Num(); ++i)
			{
				// 1. 각 지점에 박스 그리기
				DrawDebugBox(
					GetWorld(),
					PathPoints[i],
					FVector(25.f),
					FColor::Green,
					false, // false로 변경! Tick마다 새로 그릴 것이므로 유지할 필요 없음
					-1.f,  // -1 또는 0: 한 프레임만 표시
					0,
					5.f
				);

				// 2. 이전 지점과 현재 지점을 잇는 선 그리기 (첫 번째 점 제외)
				if (i > 0)
				{
					DrawDebugLine(
						GetWorld(),
						PathPoints[i - 1], // 이전 점
						PathPoints[i],     // 현재 점
						FColor::Yellow,    // 선 색상
						false,             // false
						-1.f,              // -1 또는 0
						0,
						3.f                // 선 두께
					);
				}
			}
		}
		else if (PathPoints.Num() == 1) // 점이 하나만 있을 경우
		{
			// 박스만 하나 그려줌
			DrawDebugBox(GetWorld(), PathPoints[0], FVector(25.f), FColor::Green, false, -1.f, 0, 5.f);
		}
	}
}

void ANaviController::MoveToLocation(FVector ToLocation)
{
	
}