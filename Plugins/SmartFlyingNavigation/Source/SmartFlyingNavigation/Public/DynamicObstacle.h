// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "DynamicObstacle.generated.h"

class AOctreeVoxelVolume;

UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class SMARTFLYINGNAVIGATION_API UDynamicObstacleComponent  : public UActorComponent
{
	GENERATED_BODY()

public:
	UDynamicObstacleComponent();

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

public:
	bool IsDirty() const { return bIsDirty; }
	void ClearDirtyFlag() { bIsDirty = false; }

	const FBox& GetLastKnownBounds() const { return LastKnownBounds; }
	FBox GetCurrentBounds() const;
	
	void UpdateLastKnownBounds() { LastKnownBounds = GetCurrentBounds(); }

private:
	UPROPERTY()
	TWeakObjectPtr<AOctreeVoxelVolume> AssociatedVolume;

	FTransform LastKnownTransform;
	FBox LastKnownBounds;
	
	bool bIsDirty = true; 
};