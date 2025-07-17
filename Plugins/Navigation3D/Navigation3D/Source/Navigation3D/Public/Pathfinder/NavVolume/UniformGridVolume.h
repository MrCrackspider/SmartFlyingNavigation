// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Pathfinder/NavVolume/VoxelVolumeBase.h"
#include "GameFramework/Actor.h"
#include "UniformGridVolume.generated.h"

UCLASS(Abstract) // AFenwickVoxelVolume 같은 구체적인 자식이 구현할 것이므로 추상 클래스로 유지
class NAVIGATION3D_API AUniformGridVolume : public AVoxelVolumeBase
{
	GENERATED_BODY()

public:
	AUniformGridVolume();

	//~======================================================================
	// AVoxelVolumeBase 인터페이스 구현 (이 클래스는 아직 추상이므로, 자식이 구현)
	//~======================================================================
	virtual EVolumeDataStructure GetStructureType() const override { return EVolumeDataStructure::UniformGrid; }
	
	// IsVoxelWalkable, IsRegionWalkable은 여전히 순수 가상 함수로 남아,
	// AFenwickVoxelVolume 같은 최종 자식 클래스가 구현하도록 강제합니다.
};