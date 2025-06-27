// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "PathfindingComponent.generated.h"

class UPathfindingStrategy;
class UPathfindingSubsystem;
class AVoxelVolume;

UENUM(BlueprintType)
enum class EPathfindingState : uint8
{
	Idle,			// 대기 중
	Moving,			// 경로 따라 이동 중
	PathInvalidated	// 경로가 막혀 재탐색 대기 중
};

UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class EPICPROJECT_3DNAVI_API UPathfindingComponent : public UActorComponent
{
	GENERATED_BODY()

public:	
	UPathfindingComponent();
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	UFUNCTION(BlueprintCallable, Category = "Pathfinding")
	bool FindPathToLocation(const FVector& TargetLocation);

	UFUNCTION(BlueprintPure, Category = "Pathfinding")
	const TArray<FVector>& GetFoundPath() const { return FoundPath; }

	EPathfindingState CurrentState;
	FTimerHandle RepathCooldownTimer;

	// 재탐색 타이머가 만료되었을 때 호출될 함수
	void OnRepathTimerFired();
	
	// 이동 속도 (블루프린트에서 수정 가능)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pathfinding")
	float Radius = 0.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pathfinding")
	bool bUseFenwickTree;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pathfinding")
	float MovementSpeed = 500.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pathfinding|Dynamic")
	float RepathCooldown = 0.5f;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pathfinding|Dynamic")
	int32 PathValidationLookAhead = 5;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Pathfinding|Dynamic")
	FVector CurrentTargetLocation;
	
private:
	// 현재 이동 목표인 경로상의 인덱스 (-1은 이동 중이 아님을 의미)
	int32 CurrentPathIndex;
	bool IsPathAheadValid() const;
protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason);
protected:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pathfinding", meta = (AllowPrivateAccess = "true"))
	TSubclassOf<UPathfindingStrategy> PathfindingStrategyClass;
	UPROPERTY()
	TObjectPtr<UPathfindingStrategy> PathfindingStrategy;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pathfinding", meta = (AllowPrivateAccess = "true"))
	TObjectPtr<UPathfindingSubsystem> PathSubsystem;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pathfinding", meta = (AllowPrivateAccess = "true"))
	TObjectPtr<AVoxelVolume> TargetVolume;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Pathfinding", meta = (AllowPrivateAccess = "true"))
	TArray<FVector> FoundPath;
	
	TArray<FIntVector> FoundPathIndices; 
	/**
	* DEBUGGING
	*/
public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pathfinding|Debug")
	bool bDrawDebugPath = true;
};
