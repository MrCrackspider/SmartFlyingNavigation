// Fill out your copyright notice in the Description page of Project Settings.


#include "NavigationVolume.h"
#include "Public/VoxelGrid.h"
#include "Engine/World.h"
#include "DrawDebugHelpers.h" // 디버깅 시각화를 위해 포함
#include "DynamicObstacle.h"

// Sets default values for this component's properties
ANavigationVolume::ANavigationVolume()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryActorTick.bCanEverTick = true;
	// ...
}


// Called when the game starts
void ANavigationVolume::BeginPlay()
{
	Super::BeginPlay();

}

void ANavigationVolume::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	
	BuildVoxelDataInVolume(); // 일정시간마다 호출하도록 수정 (ex 0.5초 또는 editor에서 수정)
}

void ANavigationVolume::GenerateVoxelGrid()
{
	if (!VoxelGrid)
	{
		VoxelGrid = NewObject<UVoxelGrid>(this);
		FVector Origin, BoxExtent;
		GetActorBounds(false, Origin, BoxExtent);
		VoxelGrid->Initialize(Origin, BoxExtent);
	}
}

void ANavigationVolume::BuildVoxelDataInVolume()
{
	GenerateVoxelGrid();

	VoxelGrid->VoxelMap.Empty();

	TArray<FOverlapResult> OverlappingActors;
	FCollisionShape VolumeShape = FCollisionShape::MakeBox(VoxelGrid->BoxExtent);
	FCollisionObjectQueryParams ObjectQueryParams;
	ObjectQueryParams.AddObjectTypesToQuery(ECC_WorldStatic);
	ObjectQueryParams.AddObjectTypesToQuery(ECC_PhysicsBody);

	FCollisionQueryParams CollisionParams;
	CollisionParams.AddIgnoredActor(this);

	GetWorld()->OverlapMultiByObjectType(
		OverlappingActors,
		VoxelGrid->VolumeOrigin,
		FQuat::Identity,
		ObjectQueryParams,
		VolumeShape,
		CollisionParams
	);

	const float VoxelSize = VoxelGrid->VoxelSize;
	const FVector VoxelHalfExtent = FVector(VoxelSize / 2.0f);
	FVoxelGridData SolidData;
	SolidData.bIsSolid = true;

	for (const FOverlapResult& Result : OverlappingActors)
	{		
		AActor* ObstacleActor = Result.GetActor();
		if (!ObstacleActor)
		{
			continue;
		}

		if (ObstacleActor->GetComponentByClass<UDynamicObstacle>() == nullptr)
		{
			continue;
		}

		UPrimitiveComponent* ObstacleComponent = Result.GetComponent();
		if (!ObstacleComponent)
		{
			continue;
		}
		
		FVector ObstacleOrigin, ObstacleExtent;
		ObstacleActor->GetActorBounds(false, ObstacleOrigin, ObstacleExtent);
		const FBox ObstacleBounds(ObstacleOrigin - ObstacleExtent, ObstacleOrigin + ObstacleExtent);

		FIntVector MinGridCoords = VoxelGrid->WorldToGridCoordinates(ObstacleBounds.Min);
		FIntVector MaxGridCoords = VoxelGrid->WorldToGridCoordinates(ObstacleBounds.Max);
		
		// 장애물에 대한 충돌 쿼리 파라미터를 설정합니다.
		FCollisionQueryParams ObstacleCheckParams;
		ObstacleCheckParams.AddIgnoredActor(this); // 네비게이션 볼륨은 무시
		// 필요하다면 다른 액터도 무시 목록에 추가할 수 있습니다.
		// ObstacleCheckParams.AddIgnoredActor(ObstacleActor); // 자기 자신과의 검사는 피해야 할 경우

		for (int32 x = MinGridCoords.X; x <= MaxGridCoords.X; ++x)
		{
			for (int32 y = MinGridCoords.Y; y <= MaxGridCoords.Y; ++y)
			{
				for (int32 z = MinGridCoords.Z; z <= MaxGridCoords.Z; ++z)
				{
					FIntVector CurrentGridCoords(x, y, z);
					FVector VoxelCenter = VoxelGrid->GridToWorldCoordinates(CurrentGridCoords);

					// *** 해결을 위한 핵심 로직 ***
					// 현재 복셀 위치에 복셀 크기의 박스를 만들어 장애물과 실제로 겹치는지 확인합니다.
					bool bIsOverlapping = ObstacleComponent->OverlapComponent(
						VoxelCenter,
						FQuat::Identity,
						FCollisionShape::MakeBox(VoxelHalfExtent)
					);

					// OverlapAnyTestByChannel을 사용하면 특정 액터의 컴포넌트와만 검사할 수도 있습니다.
					// bool bIsOverlapping = Result.GetComponent()->OverlapAnyTest(VoxelCenter, FQuat::Identity, FCollisionShape::MakeBox(VoxelHalfExtent));

					if (bIsOverlapping)
					{
						// 실제로 겹치는 경우에만 복셀을 Solid로 설정하고 디버그 박스를 그립니다.
						VoxelGrid->SetVoxel(CurrentGridCoords, SolidData);
						DrawDebugBox(GetWorld(), VoxelCenter, VoxelHalfExtent, FColor::Red, false, 0.0f);
					}
				}
			}
		}
	}

	// UE_LOG(LogTemp, Warning, TEXT("Voxelization complete! Found %d solid voxels."), VoxelGrid->VoxelMap.Num());
}

void ANavigationVolume::UpdateGridAtLocation(FVector Location, bool bIsSolid)
{
}
