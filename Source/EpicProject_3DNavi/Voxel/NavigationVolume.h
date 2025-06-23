// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Volume.h"
#include "NavigationVolume.generated.h"

class UVoxelGrid;

UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class EPICPROJECT_3DNAVI_API ANavigationVolume : public AVolume
{
	GENERATED_BODY()

public:	
	// Sets default values for this component's properties
	ANavigationVolume();

protected:
	// Called when the game starts
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaSeconds) override;
	void GenerateVoxelGrid();

	//지정된 볼륨 내에서 복셀화를 수행하는 함수
	UFUNCTION(BlueprintCallable, Category = "Voxelization")
	void BuildVoxelDataInVolume();

	UVoxelGrid* GetVoxelGrid() const {return VoxelGrid;}

	void UpdateGridAtLocation (FVector Location, bool bIsSolid);

private:
	UPROPERTY()
	TObjectPtr<UVoxelGrid> VoxelGrid;
};
