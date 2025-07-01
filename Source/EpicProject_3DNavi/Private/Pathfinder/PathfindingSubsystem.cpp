// Fill out your copyright notice in the Description page of Project Settings.


#include "Pathfinder/PathfindingSubsystem.h"
#include "Pathfinder/VoxelVolume.h"
#include "Pathfinder/FenwickVoxelVolume.h"
#include "Engine/World.h"
#include "Kismet/GameplayStatics.h"
void UPathfindingSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);

	UWorld* World = GetWorld();
	if (World == nullptr)
	{
		UE_LOG(LogTemp, Error, TEXT("UPathfindingSubsystem::Initialize: Could not get a valid UWorld."));
		return;
	}

	TArray<AActor*> Actors;
	//UGameplayStatics::GetAllActorsOfClass(World, AVoxelVolume::StaticClass(), Actors);
	UGameplayStatics::GetAllActorsOfClass(World, AFenwickVoxelVolume::StaticClass(), Actors);
	Volumes.Empty();

	for (AActor* Actor : Actors)
	{
		AVoxelVolumeBase* VoxelVolume = Cast<AVoxelVolumeBase>(Actor);
		if (VoxelVolume)
		{
			Volumes.Add(VoxelVolume);
		}
	}
	if (Volumes.IsEmpty())
	{
		UE_LOG(LogTemp, Warning, TEXT("UPathfindingSubsystem::Initialize: No AVoxelVolume actors found in the world. Consider placing some in the level."));
	}
	else
	{
		UE_LOG(LogTemp, Log, TEXT("UPathfindingSubsystem: Initialization complete. %d VoxelVolume(s) registered."), Volumes.Num());
	}
}

void UPathfindingSubsystem::Deinitialize()
{
    ClearAllVolumes();
    Super::Deinitialize();
}

//// TO-DO: Implement
AVoxelVolumeBase* UPathfindingSubsystem::CreateVoxelVolume(const FIntVector& InMapSize, const uint32 InVoxelExtent)
{
    UWorld* World = GetWorld();
    if (!World)
    {
        UE_LOG(LogTemp, Error, TEXT("CreateVoxelVolume: Could not get a valid UWorld."));
        return nullptr;
    }

    FActorSpawnParameters SpawnParams;
    SpawnParams.Owner = nullptr; // Owner를 this로 설정하는 것이 일반적
    SpawnParams.Instigator = nullptr;
    SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

    const FVector SpawnLocation = FVector::ZeroVector;
    const FRotator SpawnRotation = FRotator::ZeroRotator;

	AVoxelVolumeBase* SpawnedVolume = World->SpawnActor<AVoxelVolumeBase>();
    if (SpawnedVolume)
    {
    	//SpawnedVolume->MapSize = InMapSize;
    	//SpawnedVolume->VoxelExtent = InVoxelExtent;
        Volumes.Add(SpawnedVolume);
        // Begin Test
        //CreateTestVolume(); // 0번 볼륨을 가져와서 벽을 세움.
        // End Test

    }

    return SpawnedVolume;

}

void UPathfindingSubsystem::DestroyVoxelVolume(AVoxelVolumeBase* VolumeToDestroy)
{
    if (IsValid(VolumeToDestroy))
    {
        Volumes.Remove(VolumeToDestroy);
        VolumeToDestroy->Destroy();
    }
}

void UPathfindingSubsystem::ClearAllVolumes()
{
    for (AVoxelVolumeBase* Volume : Volumes)
    {
        if (IsValid(Volume))
        {
            Volume->Destroy();
        }
    }
	Volumes.Empty();
}

bool UPathfindingSubsystem::BuildVoxelVolumes()
{
    if (Volumes.IsEmpty())
    {
        UE_LOG(LogTemp, Verbose, TEXT("UPathfindingSubsystem::BuildVoxelVolumes(): There are no volumes to build."));
        return true;
    }

	UE_LOG(LogTemp, Log, TEXT("UPathfindingSubsystem::BuildVoxelVolumes: Requesting rebuild of static obstacles for %d volume(s)."), Volumes.Num());

	bool bAllBuildsSuccessful = true; // 예시: 모든 빌드가 성공했는지 추적
	for (AVoxelVolumeBase* Volume : Volumes)
	{
		if (IsValid(Volume))
		{
			Volume->BuildObstacles(); 
		}
		else
		{
			bAllBuildsSuccessful = false;
		}
	}
	return bAllBuildsSuccessful;
}

void UPathfindingSubsystem::RenderVoxelVolume(const FLinearColor& WalkableColor, const FLinearColor& ObstacleColor) const
{
    UWorld* World = GetWorld();
    if (!World) return;

    for (const auto& VolumeToRender : Volumes)
    {
        if (!IsValid(VolumeToRender)) continue;

        const FIntVector MapSize = FIntVector(0,0,0);//임시데일밴드 = VolumeToRender->GetMapSize();
        const float VoxelSize =1;//임시 데일밴드= VolumeToRender->GetVoxelExtent();

        const FVector HalfBoxExtent(VoxelSize / 2.0f);

        const FTransform VolumeTransform = VolumeToRender->GetActorTransform();

        for (int32 z = 0; z < MapSize.Z; ++z)
        {
            for (int32 y = 0; y < MapSize.Y; ++y)
            {
                for (int32 x = 0; x < MapSize.X; ++x)
                {
                    const FIntVector CurrentIndex(x, y, z);

                    const FVector VoxelLocalCenter = FVector(CurrentIndex) * VoxelSize + HalfBoxExtent;
                    //const FVector VoxelWorldCenter = VolumeTransform.TransformPosition(VoxelLocalCenter);
                    const FVector VoxelWorldCenter = VolumeToRender->GetActorLocation() + VoxelLocalCenter;
                    const bool bIsWalkable=true ;//임시 데일밴드= VolumeToRender->IsVoxelWalkable(CurrentIndex);
                    const FColor DebugColor = (bIsWalkable ? WalkableColor : ObstacleColor).ToFColor(true);
                    const float Thickness = (bIsWalkable ? 1.f : 10.f);

                    if (!bIsWalkable)
                    {
                        DrawDebugBox(
                            World,
                            VoxelWorldCenter,
                            HalfBoxExtent,
                            VolumeTransform.GetRotation(),
                            DebugColor,
                            true,
                            -1.0f,
                            0,
                            Thickness
                        );
                    }
                }
            }
        }
    }
}

void UPathfindingSubsystem::CreateTestVolume()
{
	if (Volumes.IsEmpty() || !IsValid(Volumes[0]))
	{
		UE_LOG(LogTemp, Error, TEXT("CreateTestVolume: Cannot create test volume because no valid volumes exist."));
		return;
	}

	AVoxelVolumeBase* Vol = Volumes[0];
	const FIntVector MapSize = FIntVector(0,0,0);//임시 데일밴드Vol->Mapsize();

	// --- Helper 람다 함수: 맵을 특정 상태로 초기화 ---
	auto InitializeVolume = [&](bool bFillWithObstacles)
		{
			UE_LOG(LogTemp, Log, TEXT("Initializing Volume to be %s."), bFillWithObstacles ? TEXT("Full of Obstacles") : TEXT("Empty"));
			for (int32 z = 0; z < MapSize.Z; ++z)
			{
				for (int32 y = 0; y < MapSize.Y; ++y)
				{
					for (int32 x = 0; x < MapSize.X; ++x)
					{
						//Vol->SetStaticObstacle(FIntVector(x, y, z), bFillWithObstacles);
					}
				}
			}
		};

	//====================================================================================
	// 아래에서 원하는 맵 생성 코드 블록의 주석을 해제하여 사용하세요.
	// 한 번에 하나의 맵만 활성화하는 것을 권장합니다.
	//====================================================================================


	// 1. Staircase Map (계단 맵) - 테스트 완료
	// 목적: 직선 이동과 90도 코너에서의 기본적인 점프 및 이웃 탐색을 테스트합니다.
	// 시작 추천: (2,2,2), 도착 추천: (60,60,60)
	
	//{
	//	InitializeVolume(true); // 맵을 장애물로 가득 채움
	//	UE_LOG(LogTemp, Log, TEXT("creating map 1: staircase map"));
	//	// x축 길
	//	for (int32 x = 0; x < MapSize.X; ++x) 
	//	{
	//		Vol->SetStaticObstacle({ x, 0, 0 }, false);
	//		Vol->SetStaticObstacle({ 0, x, 0 }, false);
	//	}
	//	// y축 길
	//	for (int32 y = 0; y < MapSize.Y; ++y)
	//	{
	//		Vol->SetStaticObstacle({ MapSize.X - 1, y, 0 }, false);
	//		Vol->SetStaticObstacle({ y, MapSize.Y - 1, 0 }, false);
	//	}
	//	// z축 길
	//	for (int32 z = 0; z < MapSize.Z; ++z) Vol->SetStaticObstacle({ MapSize.X - 1, MapSize.Y - 1, z }, false);

	//	for (int32 x = 0; x < MapSize.X; ++x)
	//	{
	//		Vol->SetStaticObstacle({ x, x, 0 }, false);
	//	}
	//	for (int32 x = 0; x < MapSize.X; ++x)
	//	{
	//		Vol->SetStaticObstacle({ x, x, x }, false);
	//	}
	//}
	
	// 단순한 대각선 맵 - 테스트 완료
	//{
	//	InitializeVolume(true);
	//	for (int32 x = 0; x < MapSize.X; ++x)
	//	{
	//		Vol->SetStaticObstacle({ x, x, x }, false);
	//	}
	//}

	// 평면 대각선 맵 - 테스트 완료
	//{
	//	InitializeVolume(true);
	//	for (int32 x = 0; x < MapSize.X; ++x)
	//	{
	//		Vol->SetStaticObstacle({ x, x, 0 }, false);
	//	}
	//	for (int32 z = 0; z < MapSize.Z; ++z)
	//	{
	//		Vol->SetStaticObstacle({ MapSize.X - 1, MapSize.Y - 1, z }, false);
	//	}
	//}

	// 2. Obstacle Field Map (장애물 필드 맵)
	// 목적: 개방된 공간에서 장애물을 우회하는 경로를 찾을 때, 강제된 이웃(Forced Neighbors)을 올바르게 감지하는지 테스트합니다.
	// 시작 추천: (5,5,5), 도착 추천: (55,55,5)
	
	//{
	//	InitializeVolume(false); // 맵을 빈 공간으로 초기화
	//	UE_LOG(LogTemp, Log, TEXT("Creating Map 2: Obstacle Field Map"));
	//	for (int32 x = 10; x < MapSize.X - 10; x += 8)
	//	{
	//		for (int32 y = 10; y < MapSize.Y - 10; y += 8)
	//		{
	//			for (int32 z = 0; z < 10 + ((x + y) % 30); ++z)
	//			{
	//				Vol->SetStaticObstacle({ x, y, z }, true);
	//			}
	//		}
	//	}
	//	for (int32 y = 5; y < MapSize.Y - 5; ++y)
	//	{
	//		if (y < 28 || y > 36) // 중간에 틈을 만듦
	//		{
	//			Vol->SetStaticObstacle({32, y, 10}, true);
	//		}
	//	}
	//}
	
	// 3. Diagonal Test Map (대각선 테스트 맵)
	// 목적: 직선 경로가 막혔을 때, 평면 및 공간 대각선 이동을 올바르게 수행하는지 테스트합니다.
	// 시작 추천: (5,5,10), 도착 추천: (55,55,10)
	

	{
		InitializeVolume(false); // 맵을 빈 공간으로 초기화
	//	UE_LOG(LogTemp, Log, TEXT("Creating Map 3: Diagonal Test Map"));
	//	// XY 평면 대각선 이동을 유도하는 톱니 모양 벽
	//	for (int32 i = 10; i < MapSize.X - 10; ++i)
	//	{
	//		Vol->SetStaticObstacle({i, i + 1, 10}, true);
	//		Vol->SetStaticObstacle({i + 1, i, 10}, true);
	//	}
	//	// 공간 대각선 이동을 유도하는 코너 장애물
	//	Vol->SetStaticObstacle({50, 51, 51}, true);
	//	Vol->SetStaticObstacle({51, 50, 51}, true);
	//	Vol->SetStaticObstacle({51, 51, 50}, true);
	//	Vol->SetStaticObstacle({51, 51, 51}, true);
	}
	

	// 4. Simple Maze Map (간단한 미로 맵)
	// 목적: 좁고 구불구불한 길에서 알고리즘의 정확성을 테스트합니다. JPS가 비효율적인 환경에서의 동작을 확인합니다.
	//// 시작 추천: (1,1,10), 도착 추천: (62,62,20)
	
	//{
	//	InitializeVolume(false); // 맵을 빈 공간으로 초기화
	//	UE_LOG(LogTemp, Log, TEXT("Creating Map 4: Simple Maze Map"));
	//	for (int32 i = 2; i < MapSize.X - 2; ++i)
	//	{
	//		if (i % 4 != 0)
	//		{
	//			for (int32 y = 0; y < MapSize.Y - 10; ++y)
	//			{
	//				Vol->SetStaticObstacle({i, y, 10}, true);
	//			}
	//		}
	//	}
	//	for (int32 i = 2; i < MapSize.X - 2; ++i)
	//	{
	//		if ((i + 2) % 4 != 0)
	//		{
	//			for (int32 y = 10; y < MapSize.Y; ++y)
	//			{
	//				Vol->SetStaticObstacle({i, y, 20}, true);
	//			}
	//		}
	//	}
	//	for (int32 y = MapSize.Y - 15; y < MapSize.Y - 5; ++y)
	//	{
	//		Vol->SetStaticObstacle({4, y, 10}, false);
	//		Vol->SetStaticObstacle({4, y, 20}, false);
	//	}
	//}

	UE_LOG(LogTemp, Log, TEXT("Test volume creation finished."));
}
