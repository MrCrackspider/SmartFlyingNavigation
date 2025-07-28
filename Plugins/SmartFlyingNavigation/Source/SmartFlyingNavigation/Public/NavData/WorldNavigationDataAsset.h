// WorldNavigationDataAsset.h

#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "WorldNavigationDataAsset.generated.h"

class UOctreeDataAsset;

USTRUCT(BlueprintType)
struct FVolumeSpatialInfo
{
    GENERATED_BODY()

    UPROPERTY(VisibleAnywhere, Category = "Navigation")
    FBox Bounds;

    UPROPERTY(VisibleAnywhere, Category = "Navigation")
    TSoftObjectPtr<UOctreeDataAsset> NavDataAsset;
};

UCLASS()
class SMARTFLYINGNAVIGATION_API UWorldNavigationDataAsset : public UDataAsset
{
    GENERATED_BODY()

public:
    UPROPERTY(VisibleAnywhere, Category = "Navigation")
    TArray<FVolumeSpatialInfo> VolumeInfos;
};