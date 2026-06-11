// Copyright © 2025 TeamHamDev. All Rights Reserved.

#include "SmartFlyingNavigation.h"
#include "NavCore/SmartFlyingNavStats.h"

DEFINE_STAT(STAT_SFN_FindPath);
DEFINE_STAT(STAT_SFN_AvoidanceSolve);
DEFINE_STAT(STAT_SFN_PathRequests);
DEFINE_STAT(STAT_SFN_Agents);
DEFINE_STAT(STAT_SFN_Obstacles);
DEFINE_STAT(STAT_SFN_ChunkMemory);

#define LOCTEXT_NAMESPACE "FSmartFlyingNavigationModule"

void FSmartFlyingNavigationModule::StartupModule()
{
}

void FSmartFlyingNavigationModule::ShutdownModule()
{
}

#undef LOCTEXT_NAMESPACE
	
IMPLEMENT_MODULE(FSmartFlyingNavigationModule, SmartFlyingNavigation)