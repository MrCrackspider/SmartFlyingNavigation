#pragma once

#include "CoreMinimal.h"
#include "VoxelData.h"
#include "VoxelGrid.generated.h"
 
UCLASS()
class UVoxelGrid : public UObject
{
	GENERATED_BODY()

public:
	TMap<FIntVector, FVoxelGridData> VoxelMap;

	//복셀 하나의 크기 (월드 유닛)
	float VoxelSize = 100.0f;
	FVector GridOrigin = FVector::ZeroVector;
	FVector VolumeOrigin = FVector::ZeroVector;
	FVector BoxExtent = FVector::OneVector;
	FBox VolumeBounds;
	
	void Initialize(FVector InVolumeOrigin, FVector InBoxExtent, float InVoxleSize = 100.f);
	
	// 월드 좌표를 복셀 그리드 좌표로 변환하는 헬퍼 함수
	FIntVector WorldToGridCoordinates(const FVector& WorldPosition) const;

	FVector GridToWorldCoordinates(const FIntVector& GridPos) const;

	// 그리드 좌표에 복셀 데이터를 설정하는 함수
	void SetVoxel(const FIntVector& GridCoords, const FVoxelGridData& Data);

	bool GetVoxel(const FIntVector& GridCoords, FVoxelGridData& OutData) const;

	bool IsInVolume(const FVector& WorldPosition) const;
};
