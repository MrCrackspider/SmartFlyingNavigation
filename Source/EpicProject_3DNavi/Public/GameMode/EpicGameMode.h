// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameMode.h"
#include "EpicGameMode.generated.h"

/**
 * 
 */
UCLASS()
class EPICPROJECT_3DNAVI_API AEpicGameMode : public AGameMode
{
	GENERATED_BODY()

	virtual void BeginPlay() override;
	
};
