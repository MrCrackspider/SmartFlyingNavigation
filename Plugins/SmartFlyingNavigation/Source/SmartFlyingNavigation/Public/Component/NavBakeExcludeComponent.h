// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "NavBakeExcludeComponent.generated.h"

/**
 * Add this component to any actor to exclude it from navigation baking.
 * This is a simpler, more discoverable alternative to the "NoNavBake" actor tag.
 *
 * Usage: Select an actor → Add Component → NavBakeExclude
 */
UCLASS(ClassGroup = (Navigation), meta = (BlueprintSpawnableComponent, DisplayName = "Nav Bake Exclude"))
class SMARTFLYINGNAVIGATION_API UNavBakeExcludeComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UNavBakeExcludeComponent()
	{
		bIsEditorOnly = true;
		PrimaryComponentTick.bCanEverTick = false;
	}
};
