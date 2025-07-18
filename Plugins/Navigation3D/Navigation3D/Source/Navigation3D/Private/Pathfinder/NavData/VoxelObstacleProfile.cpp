// Copyright © 2025 TeamHamDev. All Rights Reserved.

#include "Pathfinder/NavData/VoxelObstacleProfile.h"
#include "Components/PrimitiveComponent.h"

bool UVoxelObstacleProfile::DoesActorMatch(const AActor* InActor, const UPrimitiveComponent* InPrimitiveComponent) const
{
	// 1. Basic Validity Check: A null actor can never be a match.
	if (!InActor)
	{
		return false;
	}

	// 2. Check if any criteria are actually set.
	const bool bHasAnyCriteria = CollisionChannels.Num() > 0 ||
								 ActorTags.Num() > 0 ||
								 MarkerComponentClass != nullptr;
	if (!bHasAnyCriteria)
	{
		// If no rules are set, MatchAny should fail, and MatchAll should succeed.
		return Logic == EObstacleFilterLogic::MatchAll;
	}

	// 3. Branch based on the filter logic.
	if (Logic == EObstacleFilterLogic::MatchAny)
	{
		// OR Logic: Return true as soon as any condition is met.

		// Check Collision Channels
		if (InPrimitiveComponent && CollisionChannels.Num() > 0)
		{
			const ECollisionChannel ActorChannel = InPrimitiveComponent->GetCollisionObjectType();
			if (CollisionChannels.Contains(ActorChannel))
			{
				return true;
			}
		}

		// Check Actor Tags
		if (ActorTags.Num() > 0)
		{
			for (const FName& Tag : ActorTags)
			{
				if (InActor->ActorHasTag(Tag))
				{
					return true;
				}
			}
		}

		// Check for Marker Component
		if (MarkerComponentClass && InActor->FindComponentByClass(MarkerComponentClass))
		{
			return true;
		}
		
		// None of the checks passed.
		return false;
	}
	else // Logic == EObstacleFilterLogic::MatchAll
	{
		// AND Logic: Return false as soon as any condition fails.
		// Only perform checks if the corresponding criterion is set.

		// Check Collision Channels
		if (CollisionChannels.Num() > 0)
		{
			if (!InPrimitiveComponent || !CollisionChannels.Contains(InPrimitiveComponent->GetCollisionObjectType()))
			{
				return false;
			}
		}

		// Check Actor Tags
		if (ActorTags.Num() > 0)
		{
			bool bFoundAllTags = true;
			for (const FName& Tag : ActorTags)
			{
				if (!InActor->ActorHasTag(Tag))
				{
					bFoundAllTags = false;
					break;
				}
			}
			if (!bFoundAllTags)
			{
				return false;
			}
		}

		// Check for Marker Component
		if (MarkerComponentClass && !InActor->FindComponentByClass(MarkerComponentClass))
		{
			return false;
		}
		
		// All active checks have passed.
		return true;
	}
}