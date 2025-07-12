// Fill out your copyright notice in the Description page of Project Settings.


#include "VoxelVolumeBase.h"

#include "Components/InstancedStaticMeshComponent.h"

// Sets default values
AVoxelVolumeBase::AVoxelVolumeBase()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	bDrawDebugBox = true; // Default to on, can be changed in editor
	if (!GetRootComponent())
	{
		SetRootComponent(CreateDefaultSubobject<USceneComponent>(TEXT("Root")));
	}
	DebugObstaclesComponent = CreateDefaultSubobject<UInstancedStaticMeshComponent>(TEXT("DebugObstaclesComponent"));
	DebugObstaclesComponent->SetupAttachment(RootComponent);
	
	// 생성자에서는 경로만 지정해 둡니다. (또는 에디터에서 할당)
	static const FSoftObjectPath DebugMeshPath(TEXT("/Script/Engine.StaticMesh'/Navigation3D/PathfinderPlugin/Debug/SM_DebugObstacle.SM_DebugObstacle'"));
	DebugMeshAsset = DebugMeshPath;
}

void AVoxelVolumeBase::BeginPlay()
{
	Super::BeginPlay();
	if (bDrawDebugBox)
	{
		GetWorldTimerManager().SetTimer(DebugUpdateTimer, this, &AVoxelVolumeBase::UpdateDebugObstacleVisualization, 0.8f, true, 0.5f);
	}
}

void AVoxelVolumeBase::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

void AVoxelVolumeBase::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);
#if WITH_EDITORONLY_DATA

	// 이 시점에서 메시를 로드하고 설정합니다.
	if (UStaticMesh* LoadedMesh = DebugMeshAsset.LoadSynchronous())
	{
		DebugObstaclesComponent->SetStaticMesh(LoadedMesh);
		DebugObstaclesComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("AYourActor::OnConstruction - Failed to load debug mesh."));
	}
#endif
}

void AVoxelVolumeBase::BakeStaticDataToChunks()
{
}

void AVoxelVolumeBase::GetDebugObstacleVoxelIndices(TArray<FIntVector>& OutObstacleIndices) const
{
}
void AVoxelVolumeBase::UpdateDebugObstacleVisualization()
{
}

void AVoxelVolumeBase::RegisterStreamingAgent(AActor* Agent)
{
}

void AVoxelVolumeBase::UnregisterStreamingAgent(AActor* Agent)
{
}

