#pragma once

#include "CoreMinimal.h"
#include "Widgets/SCompoundWidget.h"

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

	/** '성능 설정' 카테고리가 펼쳐진 상태인지 리턴합니다. */
	bool IsPerformanceCategoryExpanded() const;

	/** '성능 설정' 카테고리의 펼침 상태가 변경될 때 호출됩니다. */
	void OnPerformanceCategoryExpansionChanged(bool bIsExpanded);

	/** 길찾기 서브시스템에 대한 약한 포인터입니다. */
	TWeakObjectPtr<UPathfindingSubsystem> PathfindingSubsystem;

	/** '성능 설정' 카테고리의 확장 상태를 저장하는 변수입니다. */
	bool bIsPerformanceCategoryExpanded = true;
};