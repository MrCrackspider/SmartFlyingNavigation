// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Stats/Stats.h"

/**
 * Plugin stat group. View on-screen with `stat SmartFlyingNav` (PC and Android;
 * on device use the 4-finger tap console or `adb shell "am broadcast ..."` / UnrealAndroidConsole).
 * One-shot text dump (logcat-friendly): `sfn.DumpNavStats`.
 */
DECLARE_STATS_GROUP(TEXT("SmartFlyingNav"), STATGROUP_SmartFlyingNav, STATCAT_Advanced);

// Worker-thread costs.
DECLARE_CYCLE_STAT_EXTERN(TEXT("FindPath (worker)"), STAT_SFN_FindPath, STATGROUP_SmartFlyingNav, SMARTFLYINGNAVIGATION_API);
DECLARE_CYCLE_STAT_EXTERN(TEXT("Avoidance Solve (worker)"), STAT_SFN_AvoidanceSolve, STATGROUP_SmartFlyingNav, SMARTFLYINGNAVIGATION_API);

// Per-frame counters.
DECLARE_DWORD_COUNTER_STAT_EXTERN(TEXT("Path Requests"), STAT_SFN_PathRequests, STATGROUP_SmartFlyingNav, SMARTFLYINGNAVIGATION_API);

// Live gauges.
DECLARE_DWORD_ACCUMULATOR_STAT_EXTERN(TEXT("Registered Agents"), STAT_SFN_Agents, STATGROUP_SmartFlyingNav, SMARTFLYINGNAVIGATION_API);
DECLARE_DWORD_ACCUMULATOR_STAT_EXTERN(TEXT("Dynamic Obstacles"), STAT_SFN_Obstacles, STATGROUP_SmartFlyingNav, SMARTFLYINGNAVIGATION_API);

// Memory.
DECLARE_MEMORY_STAT_EXTERN(TEXT("Loaded Nav Chunk Memory"), STAT_SFN_ChunkMemory, STATGROUP_SmartFlyingNav, SMARTFLYINGNAVIGATION_API);
