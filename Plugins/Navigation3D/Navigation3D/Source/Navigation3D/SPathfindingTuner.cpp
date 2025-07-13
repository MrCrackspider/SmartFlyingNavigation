#include "SPathfindingTuner.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SNumericEntryBox.h"
#include "Editor/EditorEngine.h"
#include "Pathfinder/PathfindingSubsystem.h" // 실제 경로에 맞게 수정
#include "Styling/AppStyle.h" // FAppStyle을 사용하기 위한 헤더
#include "UnrealEd.h"

void SPathfindingTuner::Construct(const FArguments& InArgs)
{
    // 서브시스템의 기본 객체(CDO)에 대한 참조를 얻습니다.
    PathfindingSubsystem = GetMutableDefault<UPathfindingSubsystem>();

    ChildSlot
    [
        SNew(SVerticalBox)

        // ... (다른 위젯들)

        + SVerticalBox::Slot()
        .AutoHeight()
        .Padding(5)
        [
            SNew(SHorizontalBox)
            // .IsEnabled은 제거하여 항상 활성화
            
            + SHorizontalBox::Slot()
            // ... (STextBlock 부분)

            + SHorizontalBox::Slot()
            .FillWidth(1.0f)
            .VAlign(VAlign_Center)
            [
                // ====================================================================
                // ▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼ 이 부분을 수정합니다 ▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼
                // ====================================================================
                SNew(SNumericEntryBox<int32>)
                // 현재 코드에 있는 함수 이름으로 바인딩합니다.
                .Value(this, &SPathfindingTuner::GetCurrentNumThreads)
                
                // OnValueCommitted를 람다로 감싸서 호출합니다.
                .OnValueCommitted_Lambda([this](int32 NewValue, ETextCommit::Type CommitType) {
                    // 람다 함수 본문에서 현재 코드에 있는 멤버 함수를 직접 호출합니다.
                    this->OnNumThreadsChanged(NewValue);
                })
                .MinValue(1)
                .MaxValue(64)
                .MinSliderValue(1)
                .MaxSliderValue(32)
                // ====================================================================
                // ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲
            ]
        ]
        
        // ... (나머지 위젯들)
    ];
}


TOptional<int32> SPathfindingTuner::GetCurrentNumThreads() const
{
    // 1. 함수가 호출될 때마다 로그를 남깁니다.
    UE_LOG(LogTemp, Log, TEXT("SPathfindingTuner: GetCurrentNumThreads() CALLED."));

    if (PathfindingSubsystem.IsValid())
    {
        // 2. CDO에서 읽어온 값을 로그로 남깁니다.
        const int32 CurrentValue = PathfindingSubsystem->GetNumWorkerThreads();
        UE_LOG(LogTemp, Log, TEXT("SPathfindingTuner: Reading value from CDO: %d"), CurrentValue);
        return CurrentValue;
    }
    else
    {
        // 3. CDO가 유효하지 않은 경우를 로그로 남깁니다.
        UE_LOG(LogTemp, Warning, TEXT("SPathfindingTuner: SubsystemCDO is NOT valid. Returning 0."));
        return 0;
    }
}

// 변경된 시그니처에 맞춰 함수 구현부를 수정합니다.
void SPathfindingTuner::OnNumThreadsChanged(int32 NewValue)
{
    // 1. 함수가 호출될 때 받은 값을 로그로 남깁니다.
    UE_LOG(LogTemp, Log, TEXT("SPathfindingTuner: OnNumThreadsChanged() CALLED with NewValue: %d"), NewValue);

    if (PathfindingSubsystem.IsValid())
    {
        // 2. 값을 변경하기 전의 CDO 값을 로그로 남깁니다.
        UE_LOG(LogTemp, Log, TEXT("SPathfindingTuner: Changing CDO value from %d to %d."), PathfindingSubsystem->GetNumWorkerThreads(), NewValue);
        
        PathfindingSubsystem->SetNumWorkerThreads(NewValue);
        PathfindingSubsystem->SaveConfig();

        // 3. 값을 변경한 후의 CDO 값을 다시 읽어서 확인합니다.
        UE_LOG(LogTemp, Log, TEXT("SPathfindingTuner: Value changed. New CDO value is: %d. Config saved."),  PathfindingSubsystem->GetNumWorkerThreads());
    }
    else
    {
        // 4. CDO가 유효하지 않은 경우를 로그로 남깁니다.
        UE_LOG(LogTemp, Error, TEXT("SPathfindingTuner: SubsystemCDO is NOT valid. Cannot change value."));
    }
}

bool SPathfindingTuner::IsSubsystemValid() const
{
    return PathfindingSubsystem.IsValid();
}