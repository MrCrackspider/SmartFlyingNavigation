// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

struct FAgentThreadData
{
    uint32 AgentId;
    FVector Position;
    FVector UpVector;
    FVector CurrentVelocity;
    FVector PreferredVelocity;
    float AvoidanceRadius;
    float MovementSpeed;
};

/** Thread-safe snapshot of a dynamic obstacle for ORCA constraint generation. */
struct FDynamicObstacleThreadData
{
    FVector Position;
    FVector Velocity;
    float Radius;
};

struct FThreadAvoidanceResult
{
    uint32 AgentId;
    TArray<FPlane> AgentConstraints;
    TArray<FPlane> VoxelConstraints;
    FVector PreferredVelocity;
    FVector ActualPreferredVelocity;
};
