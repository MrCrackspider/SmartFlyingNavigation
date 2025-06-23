#pragma once

#include "CoreMinimal.h"
#include "VoxelData.generated.h"

// 복셀에 저장될 데이터 구조체
USTRUCT(BlueprintType)
struct FVoxelGridData
{
	GENERATED_BODY()

	// 가장 기본적인 정보: 장애물인가?
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	bool bIsSolid = false;

	bool IsWalkable()
	{
		return !bIsSolid;
	}
	// 추가 정보 (예시)
	// UPROPERTY()
	// TEnumAsByte<EPhysicalSurface> SurfaceType;
};