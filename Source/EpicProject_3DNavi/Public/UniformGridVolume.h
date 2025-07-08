// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "VoxelVolumeBase.h"
#include "GameFramework/Actor.h"
#include "UniformGridVolume.generated.h"

UCLASS(Abstract) // <--- 바로 이 키워드!
class EPICPROJECT_3DNAVI_API AUniformGridVolume : public AVoxelVolumeBase
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AUniformGridVolume();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	virtual void OnConstruction(const FTransform& Transform) override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "VoxelVolume|UniformGrid")
	float VoxelExtent;
	const int CHUNK_SIZE = 64;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Visualization")
	TObjectPtr<UStaticMeshComponent> VolumeBoundsVisualizer;
	
	
	// --- 균일 격자 공통 프로퍼티가 여기에 위치! ---
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "VoxelVolume|UniformGrid")
	FIntVector MapSize;
	
	// --- 공통 기능 구현 ---
	virtual EVolumeDataStructure GetStructureType() const override { return EVolumeDataStructure::UniformGrid; }

	virtual void SetObstacle(const FIntVector& Index, bool bIsObstacle) PURE_VIRTUAL(AUniformGridVolume::SetObstacle, );
	
	// 좌표 변환은 VoxelSize와 MapSize를 기반으로 여기서 구현합니다.
	virtual FVector GetLocationFromIndex(const FIntVector& VoxelIndex) const override;
	virtual FIntVector GetIndexFromLocation(const FVector& WorldPosition) const override;
	FIntVector WorldToVoxelIndex(const FVector& WorldPosition) const;
	FVector VoxelIndexToWorld(const FIntVector& VoxelIndex) const;
	bool IsIndexValid(const FIntVector& Index) const;
	bool ClampVoxelIndicesToVolume(FIntVector& MinVoxelIndex,
	                               FIntVector& MaxVoxelIndex) const;
	bool GetOverlappingComponentsInVolume(const FCollisionObjectQueryParams& ObjectQueryParams,TArray<FOverlapResult>& OutResults) const;
	bool CheckVoxelOverlap(const FIntVector& VoxelIndex,
	                       UPrimitiveComponent* ComponentToTest) const;
	// 위치 기반 조회 함수는 인덱스 기반 조회 함수를 호출하도록 구현
	virtual bool IsLocationWalkable(const FVector& WorldPosition) const override
	{
		return IsVoxelWalkable(GetIndexFromLocation(WorldPosition));
	}
	virtual bool IsPathSegmentWalkable(const FIntVector& FromStartIndex, const FIntVector& ToStartIndex, int32 AgentRadiusInVoxels) const;

	virtual void UpdateDebugObstacleVisualization() override;
	// 균일 격자 자식들이 구현해야 할 새로운 순수 가상 함수
	virtual bool IsVoxelWalkable(const FIntVector& VoxelIndex) const PURE_VIRTUAL
	(AUniformGridVolume::IsVoxelWalkable,return false;);

	// --- JPS를 위해 추가된 인터페이스 ---
	virtual bool IsRegionWalkable(const FIntVector& CenterIndex, int32 AgentRadiusInVoxels) const PURE_VIRTUAL(AUniformGridVolume::IsRegionWalkable, return false;);
};

