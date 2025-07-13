#include "Pathfinder/UI/PathfindingTuner.h"

#include "Widgets/SBoxPanel.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SNumericEntryBox.h"
#include "Widgets/Layout/SExpandableArea.h"
#include "Pathfinder/PathfindingSubsystem.h"
#include "Styling/AppStyle.h"

void SPathfindingTuner::Construct(const FArguments& InArgs)
{
	PathfindingSubsystem = GetMutableDefault<UPathfindingSubsystem>();

	// bIsPerformanceCategoryExpanded의 초기값에 따라 위젯이 처음 보일 때 접혀있을지 결정합니다.
	const bool bInitiallyCollapsed = !bIsPerformanceCategoryExpanded;

	ChildSlot
		[
			SNew(SVerticalBox)

				// '성능 설정' 카테고리
				+ SVerticalBox::Slot()
				.AutoHeight()
				.Padding(2)
				[
					SNew(SExpandableArea)
						// Header: 항상 보이는 카테고리 제목 부분
						.HeaderContent()
						[
							SNew(STextBlock)
								.Text(FText::FromString(TEXT("성능 설정 (Performance)")))
								.Font(FAppStyle::Get().GetFontStyle("DetailsView.CategoryFontStyle"))
								.ShadowOffset(FVector2D(1.0f, 1.0f))
						]
						// Body: 펼쳤을 때만 보이는 실제 설정 위젯 부분
						.BodyContent()
						[
							// Body 안에 세로 박스를 하나 더 넣어 여러 설정을 추가하기 쉽게 합니다.
							SNew(SVerticalBox)

								// 작업 스레드 개수 설정 UI
								+ SVerticalBox::Slot()
								.AutoHeight()
								.Padding(8.0f, 5.0f) // 카테고리 내용이므로 안쪽으로 좀 더 여백을 줍니다.
								[
									SNew(SHorizontalBox)
										.ToolTipText(FText::FromString(TEXT("길찾기 연산에 사용할 워커 스레드의 개수를 설정합니다. \n값을 높이면 동시에 더 많은 길찾기 요청을 처리할 수 있지만, CPU 사용량이 증가합니다.")))

										// 왼쪽: 설명 라벨
										+ SHorizontalBox::Slot()
										.FillWidth(0.4f) // 라벨이 차지할 공간
										.VAlign(VAlign_Center)
										.Padding(0, 0, 10, 0)
										[
											SNew(STextBlock)
												.Text(FText::FromString(TEXT("작업 스레드 개수")))
												.Font(FAppStyle::GetFontStyle("PropertyWindow.NormalFont"))
										]

										// 오른쪽: 슬라이더가 포함된 숫자 입력창
										+ SHorizontalBox::Slot()
										.FillWidth(0.6f) // 입력창이 차지할 공간
										.VAlign(VAlign_Center)
										[
											SNew(SNumericEntryBox<int32>)
												.Value(this, &SPathfindingTuner::GetCurrentNumThreads)
												.OnValueCommitted_Lambda([this](int32 NewValue, ETextCommit::Type CommitType) {
												this->OnNumThreadsChanged(NewValue);
													})
												.MinValue(1)
												.MaxValue(64)
												.MinSliderValue(1)
												.MaxSliderValue(16)
												.SliderExponent(1.0f) // 선형적인 슬라이더
												.Delta(1)
										]
								]
							// (나중에 이 카테고리에 다른 설정을 추가하려면 여기에 +SVerticalBox::Slot() ... )
						]

					// [수정된 부분]
					// IsExpanded 대신 InitiallyCollapsed를 사용하고, bIsPerformanceCategoryExpanded의 반대 값을 전달합니다.
					.InitiallyCollapsed(bInitiallyCollapsed)
						.OnAreaExpansionChanged_Raw(this, &SPathfindingTuner::OnPerformanceCategoryExpansionChanged)
				]

			// (나중에 다른 카테고리를 추가하려면 여기에 +SVerticalBox::Slot() ... )
		];
}
TOptional<int32> SPathfindingTuner::GetCurrentNumThreads() const
{
	if (PathfindingSubsystem.IsValid())
	{
		return PathfindingSubsystem->GetNumWorkerThreads();
	}
	// 서브시스템이 유효하지 않으면 값을 표시하지 않도록 None을 리턴하는 것이 더 안전합니다.
	return TOptional<int32>();
}

void SPathfindingTuner::OnNumThreadsChanged(int32 NewValue)
{
	if (PathfindingSubsystem.IsValid())
	{
		UE_LOG(LogTemp, Log, TEXT("PathfindingTuner: Changing NumWorkerThreads from %d to %d."), PathfindingSubsystem->GetNumWorkerThreads(), NewValue);

		PathfindingSubsystem->SetNumWorkerThreads(NewValue);
		PathfindingSubsystem->SaveConfig(); // 변경 사항을 .ini 파일에 저장

		UE_LOG(LogTemp, Log, TEXT("PathfindingTuner: Value changed and config saved. New value is: %d."), PathfindingSubsystem->GetNumWorkerThreads());
	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT("PathfindingTuner: PathfindingSubsystem is NOT valid. Cannot change value."));
	}
}

bool SPathfindingTuner::IsPerformanceCategoryExpanded() const
{
	return bIsPerformanceCategoryExpanded;
}

void SPathfindingTuner::OnPerformanceCategoryExpansionChanged(bool bIsExpanded)
{
	bIsPerformanceCategoryExpanded = bIsExpanded;
}