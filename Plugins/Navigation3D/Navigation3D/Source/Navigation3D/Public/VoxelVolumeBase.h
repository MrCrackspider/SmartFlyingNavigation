// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
// VoxelVolumeBase.h (수정 후)

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "VoxelVolumeBase.generated.h"

class UBoxComponent;

UENUM(BlueprintType)
enum class EVolumeDataStructure : uint8
{
	NONE,
	UniformGrid,
	Octree,
	Other
};

// 자식 클래스가 핵심 기능을 구현해야만 사용할 수 있도록 '추상 클래스'로 선언합니다.
UCLASS(Abstract)
class NAVIGATION3D_API AVoxelVolumeBase : public AActor
{
	GENERATED_BODY()
	
public:	
	AVoxelVolumeBase();

protected:
	virtual void BeginPlay() override;
	virtual void OnConstruction(const FTransform& Transform) override;

public:	
	virtual void Tick(float DeltaTime) override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason);
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Voxel|VolumeSettings")
	float VoxelExtent;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Voxel|VolumeSettings")
	FIntVector MapSize;
	
	UPROPERTY()
	TObjectPtr<USceneComponent> PivotComponent;

	UPROPERTY()
	TObjectPtr<UBoxComponent> VolumeBox;


	virtual EVolumeDataStructure GetStructureType() const PURE_VIRTUAL(AVoxelVolumeBase::GetStructureType, return EVolumeDataStructure::NONE;);
	virtual bool IsRegionWalkable(const FIntVector& CenterIndex, int32 AgentRadiusInVoxels) const PURE_VIRTUAL(AVoxelVolumeBase::IsRegionWalkable, return false;);
	
	UFUNCTION(BlueprintPure, Category = "Voxel")
	virtual FVector VoxelIndexToWorld(const FIntVector& VoxelIndex) const;
    
	UFUNCTION(BlueprintPure, Category = "Voxel")
	virtual FIntVector WorldToVoxelIndex(const FVector& WorldPosition) const;

	//virtual bool IsLocationWalkable(const FVector& WorldPosition) const;
	virtual bool IsVoxelWalkable (const FIntVector& VoxelIndex) const;
	
	bool IsIndexValid(const FIntVector& Index) const;
	bool ClampVoxelIndicesToVolume(FIntVector& MinVoxelIndex, FIntVector& MaxVoxelIndex) const;
	bool ClampVoxelIndicesToVolume(const FIntVector& InMinVoxelIndex, const FIntVector& InMaxVoxelIndex, FIntVector& OutClampedMin, FIntVector& OutClampedMax) const;
	
	bool GetOverlappingComponentsInVolume(const FCollisionObjectQueryParams& ObjectQueryParams, TArray<FOverlapResult>& OutResults) const;
	bool CheckVoxelOverlap(const FIntVector& VoxelIndex, UPrimitiveComponent* ComponentToTest) const;
	
	//~======================================================================
	// 베이킹 및 스트리밍
	//~======================================================================
	
	UPROPERTY(EditAnywhere, Category="Voxel|Baking")
	TArray<AActor*> BakeExcludeActors;

#if WITH_EDITOR
	UFUNCTION(CallInEditor, Category = "Voxel|Baking")
	virtual void BakeStaticData();
#endif
	
	virtual void RegisterStreamingAgent(AActor* Agent);
	virtual void UnregisterStreamingAgent(AActor* Agent);

	//~======================================================================
	// 디버그 시각화
	//~======================================================================
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Voxel|Debug")
	bool bDrawDebugBox;
	
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Voxel|Debug", meta=(ClampMin = 1.f))
	float DebugLineWidth  = 4.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Voxel|Debug", meta=(ClampMin = 1.f))
	FColor DebugLineColor  = FColor::Green;
	
#if WITH_EDITORONLY_DATA
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Voxel|Debug")
	TSoftObjectPtr<UStaticMesh> DebugMeshAsset;
	
	UPROPERTY() TObjectPtr<UInstancedStaticMeshComponent> DebugSolidNodesComponent;
	UPROPERTY() TObjectPtr<UInstancedStaticMeshComponent> DebugEmptyNodesComponent;
	UPROPERTY() TObjectPtr<UInstancedStaticMeshComponent> DebugMixedNodesComponent;

	// 디버그용 머티리얼 (에디터에서 설정 가능)
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Voxel|Debug")
	TObjectPtr<UMaterialInterface> SolidNodeMaterial;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Voxel|Debug")
	TObjectPtr<UMaterialInterface> EmptyNodeMaterial;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Voxel|Debug")
	TObjectPtr<UMaterialInterface> MixedNodeMaterial;
	
#endif
	
	virtual void GetDebugObstacleVoxelIndices(TArray<FIntVector>& OutObstacleIndices) const;
	virtual void UpdateDebugObstacleVisualization();

protected:
	FTimerHandle DebugUpdateTimer;
	TSet<FIntVector> DisplayedVoxelIndices;
	bool bIsUpdatingVisualization = false;
};