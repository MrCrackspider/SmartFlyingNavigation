// Fill out your copyright notice in the Description page of Project Settings.

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
	// UI 데이터 바인딩을 위한 함수들
	TOptional<int32> GetCurrentNumThreads() const;
	/** 값이 변경될 때마다 호출될 콜백입니다. (시그니처 변경) */
	void OnNumThreadsChanged(int32 NewValue);
private:

	/** 서브시스템이 유효한지 (PIE가 실행 중인지) 확인합니다. */
	bool IsSubsystemValid() const;

	/** 길찾기 서브시스템에 대한 약한 포인터입니다. */
	TWeakObjectPtr<UPathfindingSubsystem> PathfindingSubsystem;
};
