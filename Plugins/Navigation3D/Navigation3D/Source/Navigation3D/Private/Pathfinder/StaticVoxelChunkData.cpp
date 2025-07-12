// Fill out your copyright notice in the Description page of Project Settings.


#include "Pathfinder/StaticVoxelChunkData.h"
#include "Serialization/Archive.h"

void UStaticVoxelChunkData::Serialize(FStructuredArchive::FRecord Record)
{
	Super::Serialize(Record);
	Record << SA_VALUE(TEXT("ObstacleBitmasks_X"), ObstacleBitmasks_X);
}
