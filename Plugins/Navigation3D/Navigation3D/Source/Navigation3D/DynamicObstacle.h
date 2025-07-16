// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "DynamicObstacle.generated.h"

class AOctreeVoxelVolume;

UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class NAVIGATION3D_API UDynamicObstacleComponent  : public UActorComponent
{
	GENERATED_BODY()

public:
	UDynamicObstacleComponent();

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

public:
	// 이 컴포넌트가 마지막 업데이트 이후 움직였는지 확인합니다.
	bool IsDirty() const { return bIsDirty; }
	void ClearDirtyFlag() { bIsDirty = false; }

	// 이 컴포넌트의 이전/현재 경계 상자를 가져옵니다.
	const FBox& GetLastKnownBounds() const { return LastKnownBounds; }
	FBox GetCurrentBounds() const;
	
	// LastKnownBounds를 현재 위치로 업데이트합니다.
	void UpdateLastKnownBounds() { LastKnownBounds = GetCurrentBounds(); }

private:
	// 이 컴포넌트가 등록된 볼륨
	UPROPERTY()
	TWeakObjectPtr<AOctreeVoxelVolume> AssociatedVolume;

	// 마지막으로 업데이트된 위치/회전/크기
	FTransform LastKnownTransform;
	// 마지막으로 업데이트된 경계 상자
	FBox LastKnownBounds;
	
	// 마지막 업데이트 이후 변경되었는지 여부
	bool bIsDirty = true; 
};