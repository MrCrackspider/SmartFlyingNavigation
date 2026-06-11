// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

#if !UE_BUILD_SHIPPING

#include "Widgets/SCompoundWidget.h"

/**
 * On-device debug panel (touch-friendly Slate, no UMG asset required).
 * Spawned by UPathfindingSubsystem when `sfn.ShowDebugPanel` is enabled.
 *
 *  [ Pause / Resume ]  — toggles game pause (UI keeps working while paused)
 *  [ Path Lines ]      — toggles sfn.DrawPathDebug at runtime
 *  [ Dump Stats ]      — runs sfn.DumpNavStats (visible via adb logcat)
 */
class SNavDebugPanel : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SNavDebugPanel) {}
		SLATE_ARGUMENT(TWeakObjectPtr<UWorld>, World)
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);

private:
	FReply OnPauseClicked();
	FReply OnPathLinesClicked();
	FReply OnStatNavClicked();
	FReply OnDumpStatsClicked();

	FText GetPauseLabel() const;
	FText GetPathLinesLabel() const;

	bool IsGamePaused() const;
	bool ArePathLinesEnabled() const;

	TWeakObjectPtr<UWorld> World;
};

#endif // !UE_BUILD_SHIPPING
