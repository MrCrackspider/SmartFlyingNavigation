// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "VoxelVolumeBase.generated.h"

UENUM(BlueprintType)
enum class EVolumeDataStructure : uint8
{
	NONE,
	UniformGrid,
	Octree,
	Other
};



UCLASS()
class EPICPROJECT_3DNAVI_API AVoxelVolumeBase : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AVoxelVolumeBase();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	UPROPERTY(EditAnywhere, Category="Baking")
	TArray<AActor*> BakeExcludeActors;
#if WITH_EDITOR
	UFUNCTION(CallInEditor, Category="★ Bake Actions", meta=(DisplayName="Bake Static Data"))
	virtual void BakeStaticDataToChunks();

#endif
	
	// --- 모든 자식이 반드시 구현해야 하는 공통 인터페이스 ---

	virtual EVolumeDataStructure GetStructureType() const PURE_VIRTUAL(AVoxelVolumeBase::GetStructureType, return EVolumeDataStructure::NONE;);

	UFUNCTION(BlueprintPure, Category = "VoxelVolume")
	virtual FVector GetLocationFromIndex(const FIntVector& Index) const PURE_VIRTUAL(AVoxelVolumeBase::GetLocationFromIndex, return FVector::ZeroVector;);
    
	UFUNCTION(BlueprintPure, Category = "VoxelVolume")
	virtual FIntVector GetIndexFromLocation(const FVector& WorldPosition) const PURE_VIRTUAL(AVoxelVolumeBase::GetIndexFromLocation, return FIntVector::ZeroValue;);

	virtual bool IsLocationWalkable(const FVector& WorldPosition) const PURE_VIRTUAL(AVoxelVolumeBase::IsLocationWalkable, return false;);

	virtual void BuildObstacles() PURE_VIRTUAL(AVoxelVolumeBase::BuildObstacles,);
	
	virtual void GetDebugObstacleVoxelIndices(TArray<FIntVector>& OutObstacleIndices) const;
	
	virtual void UpdateDebugObstacleVisualization();

	virtual void RegisterStreamingAgent(AActor* Agent);

	virtual void UnregisterStreamingAgent(AActor* Agent) ;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug", meta = (DisplayPriority = "10"))
	bool bDrawDebugBox;

#if WITH_EDITORONLY_DATA
	TObjectPtr<UInstancedStaticMeshComponent> DebugObstaclesComponent;
#endif
	
	FTimerHandle DebugUpdateTimer;

};
