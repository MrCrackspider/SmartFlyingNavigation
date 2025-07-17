#pragma once

#include "CoreMinimal.h"
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
	void OnMaxOctreeDepthChanged(int32 NewValue); // 값 변경 중
	void OnMaxOctreeDepthCommitted(int32 NewValue, ETextCommit::Type CommitType); // 값 확정 시

	// VoxelSize 프로퍼티 바인딩
	TOptional<float> GetVoxelSize() const;
	void OnVoxelSizeChanged(float NewValue); // 값 변경 중
	void OnVoxelSizeCommitted(float NewValue, ETextCommit::Type CommitType); // 값 확정 시

	// StaticObjectFilter 프로퍼티 바인딩
	void OnStaticObjectFilterChanged(const FAssetData& AssetData);
	FString GetStaticObjectFilterPath() const; // 현재 필터 에셋 경로를 반환

	// DynamicObjectFilter 프로퍼티 바인딩
	void OnDynamicObjectFilterChanged(const FAssetData& AssetData);
	FString GetDynamicObjectFilterPath() const; // 현재 필터 에셋 경로를 반환

	// Bake 버튼 클릭 핸들러
	FReply OnBakeButtonClicked();

	// '옥트리 볼륨 설정' 카테고리의 확장 상태를 저장하는 변수입니다.
	bool bIsOctreeSettingsCategoryExpanded = true; 
	// '옥트리 볼륨 설정' 카테고리의 펼침 상태가 변경될 때 호출됩니다.
	void OnOctreeSettingsCategoryExpansionChanged(bool bIsExpanded);

	FString GetSelectedOctreeVolumePath() const;

	void OnActorSelectionChanged(AActor* SelectedActor);

	FDelegateHandle OnActorSelectionChangedHandle; // <<<<<< 이 멤버 변수가 누락되었습니다!


	TWeakObjectPtr<AActor> LastSelectedActor;
	FTSTicker::FDelegateHandle ActorSelectionTickerHandle;
};