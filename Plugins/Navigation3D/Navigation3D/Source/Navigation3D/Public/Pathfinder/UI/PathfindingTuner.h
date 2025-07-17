#pragma once

#include "CoreMinimal.h"
#include "Pathfinder/PathfindingComponent.h"
#include "Widgets/SCompoundWidget.h"

class AOctreeVoxelVolume;
class UPathfindingSubsystem;

class NAVIGATION3D_API SPathfindingTuner : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SPathfindingTuner) {}
	SLATE_END_ARGS()

	/** 위젯을 구성하는 함수입니다. */
	void Construct(const FArguments& InArgs);

private:
	TSharedRef<SWidget> BuildPerformanceSettingsSection();
	TSharedRef<SWidget> BuildOctreeVolumeSettingsSection();
	FText GetBakedDataSaveDirectoryAsText() const;
	void OnBakedDataSaveDirectoryTextCommitted(const FText& InText,
	                                           ETextCommit::Type CommitInfo);
	FReply OnBrowseButtonClicked();
	void SetBakedDataSaveDirectory(const FString& NewPath);
	TSharedRef<SWidget> BuildBakedDataSettingsSection(); // <--- 이 함수
	TSharedRef<SWidget> BuildAgentSettingsSection();
	FText GetAgentRadiusToolTipText() const;


	FString GetBakedOctreeDataPath() const;
	void OnBakedOctreeDataChanged(const FAssetData& AssetData);
	

	// --- Agent Settings UI Callbacks ---
	// 에이전트 설정 섹션의 확장 상태 관리

	FText GetSelectedAgentName() const; //

	EVisibility GetAgentRadiusHelpIconVisibility() const;
	FText GetMovementSpeedToolTipText() const;
	EVisibility GetMovementSpeedHelpIconVisibility() const;
	FText GetTargetActorToolTipText() const;
	EVisibility GetTargetActorHelpIconVisibility() const;
	bool IsAgentSettingsCategoryExpanded() const;
	void OnAgentSettingsCategoryExpansionChanged(bool bIsExpanded);

	// Radius
	TOptional<float> GetAgentRadius() const;
	void OnAgentRadiusCommitted(float NewValue, ETextCommit::Type CommitType);

	// Movement Speed
	TOptional<float> GetMovementSpeed() const;
	void OnMovementSpeedCommitted(float NewValue, ETextCommit::Type CommitType);

	// Target Actor
	FString GetTargetActorPath() const;
	void OnTargetActorChanged(const FAssetData& AssetData);

	bool bIsAgentSettingsCategoryExpanded = true; // <--- 에이전트 설정 섹션 확장 상태
	
	TWeakObjectPtr<UPathfindingComponent> CachedPathfindingComponent; // <--- 선택된 액
	// --- 데이터 바인딩을 위한 멤버 함수들 ---

	/** SNumericEntryBox의 현재 값을 제공합니다. */
	TOptional<int32> GetCurrentNumThreads() const;

	/** SNumericEntryBox의 값이 확정될 때 호출됩니다. */
	void OnNumThreadsChanged(int32 NewValue);

	void OnNumThreadsCommitted(int32 NewValue, ETextCommit::Type CommitInfo);
	
	/** '성능 설정' 카테고리가 펼쳐진 상태인지 리턴합니다. */
	bool IsPerformanceCategoryExpanded() const;

	/** '성능 설정' 카테고리의 펼침 상태가 변경될 때 호출됩니다. */
	void OnPerformanceCategoryExpansionChanged(bool bIsExpanded);
	
	/** 길찾기 서브시스템에 대한 약한 포인터입니다. */
	TWeakObjectPtr<UPathfindingSubsystem> PathfindingSubsystem;

	/** '성능 설정' 카테고리의 확장 상태를 저장하는 변수입니다. */
	bool bIsPerformanceCategoryExpanded = true;


	
	/** 현재 선택된 AOctreeVoxelVolume 액터를 캐시합니다. */
	TWeakObjectPtr<AOctreeVoxelVolume> CachedOctreeVolume;

	/** 현재 선택된 AOctreeVoxelVolume 액터를 가져오는 헬퍼 함수입니다. */
	TWeakObjectPtr<AOctreeVoxelVolume> GetSelectedOctreeVolume() const;

	// MaxOctreeDepth 프로퍼티 바인딩
	TOptional<int32> GetMaxOctreeDepth() const;
	void OnMaxOctreeDepthCommitted(int32 NewValue, ETextCommit::Type CommitType); // 값 확정 시

	// VoxelSize 프로퍼티 바인딩
	TOptional<float> GetVoxelSize() const;
	void OnVoxelSizeCommitted(float NewValue, ETextCommit::Type CommitType); // 값 확정 시

	// StaticObjectFilter 프로퍼티 바인딩
	void OnStaticObjectFilterChanged(const FAssetData& AssetData);
	FString GetStaticObjectFilterPath() const; // 현재 필터 에셋 경로를 반환
	
	// Bake 버튼 클릭 핸들러
	FReply OnBakeButtonClicked();

	// '옥트리 볼륨 설정' 카테고리의 확장 상태를 저장하는 변수입니다.
	bool bIsOctreeSettingsCategoryExpanded = true; 
	// '옥트리 볼륨 설정' 카테고리의 펼침 상태가 변경될 때 호출됩니다.
	void OnOctreeSettingsCategoryExpansionChanged(bool bIsExpanded);
	void ShowNoOctreeSelectedWarning() const;
	void ShowNoAgentSelectedWarning() const;

	FString GetSelectedOctreeVolumePath() const;

	void OnActorSelectionChanged(); 

	FDelegateHandle OnActorSelectionChangedHandle; // <<<<<< 이 멤버 변수가 누락되었습니다!


	TArray<TWeakObjectPtr<AActor>> LastSelectedActors; // 선택된 모든 액터

	// 캐시된 혼합 값 상태 (성능 최적화를 위해 OnActorSelectionChanged에서 계산)
	TOptional<float> CachedAgentRadiusValue;
	TOptional<float> CachedMovementSpeedValue;
	TWeakObjectPtr<AActor> CachedTargetActorValue; // TargetActor는 혼합 값 표현이 복잡하므로 일단 단일 값만 캐시하거나, FText로 "Mixed" 표시
	// ... 다른 속성들도 유사하게 캐시 ...
	bool bIsAgentRadiusMixed;
	bool bIsMovementSpeedMixed;
	bool bIsTargetActorMixed;
	
	FTSTicker::FDelegateHandle ActorSelectionTickerHandle;


	FDirectoryPath BakedDataSaveDirectory; 
	
};