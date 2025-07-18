#pragma once

#include "CoreMinimal.h"
#include "Pathfinder/PathfindingComponent.h"
#include "Widgets/SCompoundWidget.h"

class AOctreeVoxelVolume;
class UPathfindingSubsystem;

/**
 * UI widget for configuring pathfinding-related data such as agents, octree, and performance settings.
 */
class NAVIGATION3D_API SPathfindingTuner : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SPathfindingTuner) {}
	SLATE_END_ARGS()

	/** Constructs the widget layout */
	void Construct(const FArguments& InArgs);

private:
	//----------------------------------------
	// UI Section Builders
	//----------------------------------------

	TSharedRef<SWidget> BuildPerformanceSettingsSection();
	TSharedRef<SWidget> BuildOctreeVolumeSettingsSection();
	TSharedRef<SWidget> BuildBakedDataSettingsSection();
	FText GetBakedDataFullPathAsText() const;
	TSharedRef<SWidget> BuildAgentSettingsSection();

	//----------------------------------------
	// Baked Octree Data Section
	//----------------------------------------

	FText GetBakedDataSaveDirectoryAsText() const;
	void OnBakedDataSaveDirectoryTextCommitted(const FText& InText, ETextCommit::Type CommitInfo);
	FReply OnBrowseButtonClicked();
	void SetBakedDataSaveDirectory(const FString& NewPath);
	FString GetBakedOctreeDataPath() const;
	void OnBakedOctreeDataChanged(const FAssetData& AssetData);

	FDirectoryPath BakedDataSaveDirectory;

	//----------------------------------------
	// Octree Settings
	//----------------------------------------
	TArray<AOctreeVoxelVolume*> GetSelectedVolumes() const;
	bool IsOctreeSettingsEnabled() const;
	TOptional<int32> GetMaxOctreeDepth() const;
	void OnMaxOctreeDepthCommitted(int32 NewValue, ETextCommit::Type CommitType);
	TOptional<float> GetVoxelSize() const;
	void OnVoxelSizeCommitted(float NewValue, ETextCommit::Type CommitType);
	void OnStaticObjectFilterChanged(const FAssetData& AssetData);
	FString GetStaticObjectFilterPath() const;
	FReply OnBakeButtonClicked();

	bool bIsOctreeSettingsCategoryExpanded = true;
	void OnOctreeSettingsCategoryExpansionChanged(bool bIsExpanded);
	void UpdateOctreeSettingsCache();
	FString GetSelectedOctreeVolumePath() const;

	TOptional<int32> CachedMaxOctreeDepth;
	bool bIsMaxOctreeDepthMixed = false;

	/** MapSize 드롭다운에 표시될 옵션 목록입니다. */
	TArray<TSharedPtr<int32>> MapSizeOptions;

	/** UI의 드롭다운에 현재 선택된 MapSize를 텍스트로 표시하는 함수입니다. */
	FText GetSelectedMapSizeText() const;

	/** 사용자가 드롭다운에서 새로운 MapSize를 선택했을 때 호출될 함수입니다. */
	void OnMapSizeSelectionChanged(TSharedPtr<int32> NewSelection, ESelectInfo::Type SelectType);
	//----------------------------------------
	// Agent Settings
	//----------------------------------------

	bool IsAgentSettingsEnabled() const;
	FText GetSelectedAgentName() const;
	TArray<AActor*> GetSelectedAgents() const;
	bool IsAgentSettingsCategoryExpanded() const;
	void OnAgentSettingsCategoryExpansionChanged(bool bIsExpanded);

	TOptional<float> GetAgentRadius() const;
	void OnAgentRadiusCommitted(float NewValue, ETextCommit::Type CommitType);

	TOptional<float> GetMovementSpeed() const;
	void OnMovementSpeedCommitted(float NewValue, ETextCommit::Type CommitType);

	FString GetTargetActorPath() const;
	void OnTargetActorChanged(const FAssetData& AssetData);

	bool bIsAgentSettingsCategoryExpanded = true;
	TOptional<float> CachedAgentRadiusValue;
	TOptional<float> CachedMovementSpeedValue;
	TWeakObjectPtr<AActor> CachedTargetActorValue;
	bool bIsAgentRadiusMixed=false;
	bool bIsMovementSpeedMixed=false;
	bool bIsTargetActorMixed=false;

	void UpdateAgentSettingsCache();

	//----------------------------------------
	// Performance Settings
	//----------------------------------------

	TOptional<int32> GetCurrentNumThreads() const;
	void OnNumThreadsCommitted(int32 NewValue, ETextCommit::Type CommitInfo);

	bool IsPerformanceCategoryExpanded() const;
	void OnPerformanceCategoryExpansionChanged(bool bIsExpanded);

	bool bIsPerformanceCategoryExpanded = true;

	//----------------------------------------
	// Selection and Editor Interaction
	//----------------------------------------

	TArray<TWeakObjectPtr<AActor>> LastSelectedActors;
	void OnActorSelectionChanged();
	FDelegateHandle OnActorSelectionChangedHandle;
	FTSTicker::FDelegateHandle ActorSelectionTickerHandle;

	//----------------------------------------
	// Subsystem
	//----------------------------------------

	TWeakObjectPtr<UPathfindingSubsystem> PathfindingSubsystem;

	//----------------------------------------
	// Error Message
	//----------------------------------------
	void ShowNoOctreeSelectedWarning() const;
	void ShowNoAgentSelectedWarning() const;
	FText GetSelectedVolumeDisplayText() const;
};
