// Copyright © 2025 TeamHamDev. All Rights Reserved.

#include "Debug/SNavDebugPanel.h"

#if !UE_BUILD_SHIPPING

#include "Engine/World.h"
#include "Engine/Engine.h"
#include "GameFramework/PlayerController.h"
#include "Kismet/GameplayStatics.h"
#include "HAL/IConsoleManager.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Text/STextBlock.h"

#define LOCTEXT_NAMESPACE "SNavDebugPanel"

namespace
{
	/** Runs a console command in the panel's world. */
	void ExecConsoleCommand(UWorld* World, const TCHAR* Command)
	{
		if (GEngine && World)
		{
			GEngine->Exec(World, Command);
		}
	}

	TSharedRef<SWidget> MakePanelButton(const TAttribute<FText>& Label, const FOnClicked& OnClicked)
	{
		return SNew(SButton)
			.OnClicked(OnClicked)
			.ContentPadding(FMargin(18.f, 12.f))
			[
				SNew(STextBlock)
				.Text(Label)
				.Font(FCoreStyle::GetDefaultFontStyle("Bold", 16))
			];
	}
}

void SNavDebugPanel::Construct(const FArguments& InArgs)
{
	World = InArgs._World;

	ChildSlot
	[
		SNew(SVerticalBox)

		+ SVerticalBox::Slot().AutoHeight().Padding(4.f)
		[
			MakePanelButton(
				TAttribute<FText>::CreateSP(this, &SNavDebugPanel::GetPauseLabel),
				FOnClicked::CreateSP(this, &SNavDebugPanel::OnPauseClicked))
		]

		+ SVerticalBox::Slot().AutoHeight().Padding(4.f)
		[
			MakePanelButton(
				TAttribute<FText>::CreateSP(this, &SNavDebugPanel::GetPathLinesLabel),
				FOnClicked::CreateSP(this, &SNavDebugPanel::OnPathLinesClicked))
		]

		+ SVerticalBox::Slot().AutoHeight().Padding(4.f)
		[
			MakePanelButton(
				LOCTEXT("StatNavButton", "stat SmartFlyingNav"),
				FOnClicked::CreateSP(this, &SNavDebugPanel::OnStatNavClicked))
		]

		+ SVerticalBox::Slot().AutoHeight().Padding(4.f)
		[
			MakePanelButton(
				LOCTEXT("DumpStatsButton", "Dump Stats (logcat)"),
				FOnClicked::CreateSP(this, &SNavDebugPanel::OnDumpStatsClicked))
		]
	];
}

bool SNavDebugPanel::IsGamePaused() const
{
	return World.IsValid() && World->IsPaused();
}

bool SNavDebugPanel::ArePathLinesEnabled() const
{
	static IConsoleVariable* DrawCVar = IConsoleManager::Get().FindConsoleVariable(TEXT("sfn.DrawPathDebug"));
	return DrawCVar && DrawCVar->GetInt() == 1;
}

FReply SNavDebugPanel::OnPauseClicked()
{
	if (UWorld* WorldPtr = World.Get())
	{
		UGameplayStatics::SetGamePaused(WorldPtr, !WorldPtr->IsPaused());
	}
	return FReply::Handled();
}

FReply SNavDebugPanel::OnPathLinesClicked()
{
	static IConsoleVariable* DrawCVar = IConsoleManager::Get().FindConsoleVariable(TEXT("sfn.DrawPathDebug"));
	if (DrawCVar)
	{
		DrawCVar->Set(DrawCVar->GetInt() == 1 ? 0 : 1, ECVF_SetByConsole);
	}
	return FReply::Handled();
}

FReply SNavDebugPanel::OnStatNavClicked()
{
	ExecConsoleCommand(World.Get(), TEXT("stat SmartFlyingNav"));
	return FReply::Handled();
}

FReply SNavDebugPanel::OnDumpStatsClicked()
{
	ExecConsoleCommand(World.Get(), TEXT("sfn.DumpNavStats"));
	return FReply::Handled();
}

FText SNavDebugPanel::GetPauseLabel() const
{
	return IsGamePaused()
		? LOCTEXT("ResumeButton", "▶ Resume")
		: LOCTEXT("PauseButton", "❚❚ Pause");
}

FText SNavDebugPanel::GetPathLinesLabel() const
{
	return ArePathLinesEnabled()
		? LOCTEXT("PathLinesOn", "Path Lines: ON")
		: LOCTEXT("PathLinesOff", "Path Lines: OFF");
}

#undef LOCTEXT_NAMESPACE

#endif // !UE_BUILD_SHIPPING
