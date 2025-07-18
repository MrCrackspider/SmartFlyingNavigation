#include "Pathfinder/UI/PathfindingTuner.h"

#include "Widgets/SBoxPanel.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SNumericEntryBox.h"
#include "Widgets/Layout/SExpandableArea.h"
#include "Pathfinder/Subsystem/PathfindingSubsystem.h"
#include "Styling/AppStyle.h"
// AOctreeVoxelVolume 관련 추가 헤더
#include "Editor.h" // GEditor 사용
#include "Editor/EditorEngine.h" // UEditorEngine 사용
#include "Widgets/Layout/SBorder.h" // SBorder 사용
#include "IDetailsView.h" // IDetailsView 사용 (필요하다면)
#include "PropertyEditorModule.h" // FPropertyEditorModule 사용 (필요하다면)
#include "Internationalization/Text.h"
#include "Widgets/Input/SFilePathPicker.h"
#include "DesktopPlatformModule.h" // 이제 오류 없이 포함됩니다.
#include "IDesktopPlatform.h" 

// AOctreeVoxelVolume 클래스 헤더 (당신의 프로젝트 경로에 맞게 수정)
#include "Pathfinder/NavData/VoxelObstacleFilter.h"
#include "PropertyCustomizationHelpers.h"
#include "Selection.h"
#include "Pathfinder/NavVolume/OctreeVoxelVolume.h"
#include "Widgets/Layout/SUniformGridPanel.h"
// 필터 데이터 에셋 클래스 헤더 (당신의 실제 클래스 이름으로 변경)

#define LOCTEXT_NAMESPACE "PathfindingTuner" 

void SPathfindingTuner::Construct(const FArguments& InArgs)
{
    PathfindingSubsystem = GEngine->GetEngineSubsystem<UPathfindingSubsystem>();

	MapSizeOptions.Add(MakeShared<int32>(128));
	MapSizeOptions.Add(MakeShared<int32>(256));
	MapSizeOptions.Add(MakeShared<int32>(512));
	
    // --- 액터 선택 감지 로직 (FTSTicker 기반의 다중 선택 폴링으로 변경) ---
    if (GEditor)
    {
       TWeakPtr<SPathfindingTuner> WeakThisPtr = SharedThis(this);
       // FTSTicker를 사용하여 주기적으로 액터 선택 변경을 감지합니다.
       ActorSelectionTickerHandle = FTSTicker::GetCoreTicker().AddTicker(

          FTickerDelegate::CreateLambda([WeakThisPtr](float DeltaTime)
          {
             if (TSharedPtr<SPathfindingTuner> Pinned = WeakThisPtr.Pin())
             {
                if (!GEditor)
                {
                   return true; // 에디터가 유효하지 않으면 틱 중지
                }

                // 현재 선택된 액터 목록을 가져옵니다.
                TArray<UObject*> CurrentSelectedObjects;
                if (USelection* Sel = GEditor->GetSelectedActors())
                {
                   Sel->GetSelectedObjects(CurrentSelectedObjects);
                }

                // AActor 타입 중 AOctreeVoxelVolume 또는 UPathfindingComponent를 가진 액터만 필터링합니다.
                TArray<TWeakObjectPtr<AActor>> FilteredSelectedActors;
                for (UObject* Obj : CurrentSelectedObjects)
                {
                   if (AActor* Actor = Cast<AActor>(Obj))
                   {
                      if (Actor->IsA(AOctreeVoxelVolume::StaticClass()) || Actor->FindComponentByClass<
                         UPathfindingComponent>())
                      {
                         FilteredSelectedActors.Add(Actor);
                      }
                   }
                }

                // 이전 필터링된 선택 목록과 현재 필터링된 선택 목록을 비교하여 변경 여부를 확인합니다.
                bool bSelectionReallyChanged = false;
                if (FilteredSelectedActors.Num() != Pinned->LastSelectedActors.Num())
                {
                   bSelectionReallyChanged = true;
                }
                else
                {
                   // 개수가 같다면, 각 요소가 동일한지 확인합니다. (Set 비교)
                   TSet<TWeakObjectPtr<AActor>> LastSelectedSet(Pinned->LastSelectedActors);
                   for (TWeakObjectPtr<AActor> CurrentActorPtr : FilteredSelectedActors)
                   {
                      if (!LastSelectedSet.Contains(CurrentActorPtr))
                      {
                         bSelectionReallyChanged = true;
                         break;
                      }
                   }
                   if (!bSelectionReallyChanged) // 역방향으로도 확인 (완벽한 Set 비교)
                   {
                      TSet<TWeakObjectPtr<AActor>> CurrentSelectedSet(FilteredSelectedActors);
                      for (TWeakObjectPtr<AActor> LastActorPtr : Pinned->LastSelectedActors)
                      {
                         if (!CurrentSelectedSet.Contains(LastActorPtr))
                         {
                            bSelectionReallyChanged = true;
                            break;
                         }
                      }
                   }
                }
                if (bSelectionReallyChanged)
                {
                   // 선택이 변경되었으면 LastSelectedActors를 업데이트하고 내부 로직을 호출합니다.
                   Pinned->LastSelectedActors = FilteredSelectedActors; // 새로운 선택 목록으로 업데이트
                   Pinned->OnActorSelectionChanged(); // UI 업데이트 및 속성 집계 로직 호출
                }

                return true; // 계속 틱을 돌립니다.
             }
             return false;
          })
       );


       // 위젯 초기화 시 현재 선택 상태를 한 번 호출하여 UI를 초기화합니다.
       // FTSTicker 람다가 첫 틱에서 이 작업을 수행할 것이므로, 여기서는 명시적으로 호출하지 않아도 됩니다.
       // 하지만 즉시 UI를 업데이트하려면 아래처럼 LastSelectedActors를 채우고 OnSelectionChangedInternal을 호출합니다.
       TArray<UObject*> InitialSelectedObjects;
       USelection* EditorSelection = GEditor->GetSelectedActors();
       if (EditorSelection)
       {
          EditorSelection->GetSelectedObjects(InitialSelectedObjects);
       }

       LastSelectedActors.Empty(); // 초기화
       for (UObject* Obj : InitialSelectedObjects)
       {
          if (AActor* Actor = Cast<AActor>(Obj))
          {
             if (Actor->IsA(AOctreeVoxelVolume::StaticClass()) || Actor->FindComponentByClass<
                UPathfindingComponent>())
             {
                LastSelectedActors.Add(Actor);
             }
          }
       }
       OnActorSelectionChanged(); // 초기 UI 업데이트
    }


    ChildSlot
    [


       SNew(SVerticalBox)

       // --- 1. '성능 설정' 카테고리 ---
       + SVerticalBox::Slot()
       .AutoHeight()
       .Padding(2)
       [
          BuildPerformanceSettingsSection() // 함수 호출로 분리
       ]

       // --- 2. '옥트리 볼륨 설정' 카테고리 ---
       + SVerticalBox::Slot()
       .AutoHeight()
       .Padding(2)
       [
          BuildOctreeVolumeSettingsSection() // 함수 호출로 분리
       ]


       // --- 3. '베이크된 데이터 설정' 카테고리 (새로 추가) ---
       + SVerticalBox::Slot()
       .AutoHeight()
       .Padding(2)
       [
          BuildBakedDataSettingsSection() // <--- 이 함수 호출 추가
       ]


       // --- 4. '에이전트 설정' 카테고리 (새로 추가) ---
       + SVerticalBox::Slot()
       .AutoHeight()
       .Padding(2)
       [
          BuildAgentSettingsSection() // <--- 이 함수 호출 추가
       ]

       // Bake 버튼
       + SVerticalBox::Slot()
       .AutoHeight()
       .Padding(8.0f, 15.0f)
       [
          SNew(SButton)
          .Text(LOCTEXT("BakeButtonText", "Bake Octree Data"))
          .OnClicked(this, &SPathfindingTuner::OnBakeButtonClicked)
       ]
    ];
}
// --- UI Section Builder Implementations ---

TSharedRef<SWidget> SPathfindingTuner::BuildPerformanceSettingsSection()
{
    const bool bInitiallyCollapsedPerformance = !bIsPerformanceCategoryExpanded;

    return SNew(SExpandableArea)
        .HeaderContent()
        [
            SNew(STextBlock)
                .Text(FText::FromString(TEXT("성능 설정 (Performance)")))
                .Font(FAppStyle::Get().GetFontStyle("DetailsView.CategoryFontStyle"))
                .ShadowOffset(FVector2D(1.0f, 1.0f))
        ]
        .BodyContent()
        [
            SNew(SVerticalBox)
                // 작업 스레드 개수 설정 UI
                + SVerticalBox::Slot()
                .AutoHeight()
                .Padding(8.0f, 5.0f)
                [
                    SNew(SHorizontalBox)
                        .ToolTipText(FText::FromString(TEXT("길찾기 연산에 사용할 워커 스레드의 개수를 설정합니다. \n값을 높이면 동시에 더 많은 길찾기 요청을 처리할 수 있지만, CPU 사용량이 증가합니다.")))
                        + SHorizontalBox::Slot()
                        .FillWidth(0.4f)
                        .VAlign(VAlign_Center)
                        .Padding(0, 0, 10, 0)
                        [
                            SNew(STextBlock)
                                .Text(FText::FromString(TEXT("작업 스레드 개수")))
                                .Font(FAppStyle::GetFontStyle("PropertyWindow.NormalFont"))
                        ]
                        + SHorizontalBox::Slot()
                        .FillWidth(0.6f)
                        .VAlign(VAlign_Center)
                        [
                            SNew(SNumericEntryBox<int32>)
                                .Value(this, &SPathfindingTuner::GetCurrentNumThreads)
                                .OnValueCommitted(this, &SPathfindingTuner::OnNumThreadsCommitted)
                                .MinValue(1)
                                .MaxValue(64)
                                .MinSliderValue(1)
                                .MaxSliderValue(16)
                                .SliderExponent(1.0f)
                                .Delta(1)
                        ]
                ]
        ]
    .InitiallyCollapsed(bInitiallyCollapsedPerformance)
    .OnAreaExpansionChanged_Raw(this, &SPathfindingTuner::OnPerformanceCategoryExpansionChanged);
}

TSharedRef<SWidget> SPathfindingTuner::BuildOctreeVolumeSettingsSection()
{
    const bool bInitiallyCollapsedOctreeSettings = !bIsOctreeSettingsCategoryExpanded;

    return SNew(SExpandableArea)
    	.IsEnabled(this, &SPathfindingTuner::IsOctreeSettingsEnabled)
        .HeaderContent()
        [
            SNew(STextBlock)
                .Text(LOCTEXT("OctreeSettingsCategory", "옥트리 볼륨 설정 (Octree Volume Settings)"))
                .Font(FAppStyle::Get().GetFontStyle("DetailsView.CategoryFontStyle"))
                .ShadowOffset(FVector2D(1.0f, 1.0f))
        ]
        .BodyContent()
        [
        	SNew(SVerticalBox)
		// 타겟 볼륨 액터 '표시'
		+ SVerticalBox::Slot()
		.AutoHeight()
		.Padding(8.0f, 5.0f)
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot()
			.FillWidth(0.4f)
			.VAlign(VAlign_Center)
			[
				SNew(STextBlock)
				// 레이블 텍스트를 좀 더 명확하게 바꿉니다.
				.Text(LOCTEXT("SelectedVolumeLabel", "Selected Volume(s):"))
				.Font(FAppStyle::GetFontStyle("PropertyWindow.NormalFont"))
			]
			+ SHorizontalBox::Slot()
			.FillWidth(0.6f)
			.VAlign(VAlign_Center)
			[
				// SObjectPropertyEntryBox 대신 STextBlock을 사용합니다.
				SNew(STextBlock)
				// 이 함수는 선택 상태에 따라 다른 텍스트를 반환합니다. (이전 답변 참고)
				.Text(this, &SPathfindingTuner::GetSelectedVolumeDisplayText)
				.Font(FAppStyle::GetFontStyle("PropertyWindow.BoldFont")) // 중요한 정보이므로 굵게 표시
			]
		]

        	+ SVerticalBox::Slot()
			.AutoHeight()
			.Padding(8.0f, 5.0f)
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot()
				.FillWidth(0.4f)
				.VAlign(VAlign_Center)
				[
					SNew(STextBlock)
					.Text(LOCTEXT("BakedDataSaveDirectoryLabel", "Baked Data Save Directory:"))
					.Font(FAppStyle::GetFontStyle("PropertyWindow.NormalFont"))
				]
				+ SHorizontalBox::Slot()
				.FillWidth(0.6f)
				[
					SNew(SHorizontalBox)
					// 경로 텍스트 박스
					+ SHorizontalBox::Slot()
					.FillWidth(1.0f)
					.VAlign(VAlign_Center)
					[
						SNew(SEditableTextBox)
						.Text(this, &SPathfindingTuner::GetBakedDataSaveDirectoryAsText)
						.OnTextCommitted(this, &SPathfindingTuner::OnBakedDataSaveDirectoryTextCommitted)
						.HintText(LOCTEXT("DirectoryPathHint", "Path to save directory..."))
					]
					// '...' 찾아보기 버튼
					+ SHorizontalBox::Slot()
					.AutoWidth()
					.Padding(4.0f, 0.0f, 0.0f, 0.0f)
					.VAlign(VAlign_Center)
					[
						SNew(SButton)
						.Text(FText::FromString(TEXT("...")))
						.ToolTipText(LOCTEXT("BakedDataSaveDirectoryBrowseTooltip", "Choose a directory from this computer"))
						.OnClicked(this, &SPathfindingTuner::OnBrowseButtonClicked)
					]
				]
			]
                // MaxOctreeDepth 설정
                + SVerticalBox::Slot()
                .AutoHeight()
                .Padding(8.0f, 5.0f)
                [
                    SNew(SHorizontalBox)
                    + SHorizontalBox::Slot()
                    .FillWidth(0.4f)
                    .VAlign(VAlign_Center)
                    [
                        SNew(STextBlock)
                        .Text(LOCTEXT("MaxOctreeDepthLabel", "Max Octree Depth:"))
                        .Font(FAppStyle::GetFontStyle("PropertyWindow.NormalFont"))
                    ]
                    + SHorizontalBox::Slot()
                    .FillWidth(0.6f)
                    [
                        SNew(SNumericEntryBox<int32>)
                        .Value(this, &SPathfindingTuner::GetMaxOctreeDepth)
                        .OnValueCommitted(this, &SPathfindingTuner::OnMaxOctreeDepthCommitted)
                        .MinValue(0)
                        .MaxValue(10)
                        .MinSliderValue(0)
                        .MaxSliderValue(8)
                        .Delta(1)
                    ]
                ]
                // VoxelSize 설정
                + SVerticalBox::Slot()
                .AutoHeight()
                .Padding(8.0f, 5.0f)
                [
                    SNew(SHorizontalBox)
                    + SHorizontalBox::Slot()
                    .FillWidth(0.4f)
                    .VAlign(VAlign_Center)
                    [
                        SNew(STextBlock)
                        .Text(LOCTEXT("VoxelSizeLabel", "Voxel Size:"))
                        .Font(FAppStyle::GetFontStyle("PropertyWindow.NormalFont"))
                    ]
                    + SHorizontalBox::Slot()
                    .FillWidth(0.6f)
                    [
                        SNew(SNumericEntryBox<float>)
                        .Value(this, &SPathfindingTuner::GetVoxelSize)
                        .OnValueCommitted(this, &SPathfindingTuner::OnVoxelSizeCommitted)
                        .MinValue(1.0f)
                        .MaxValue(1000.0f)
                        .MinSliderValue(10.0f)
                        .MaxSliderValue(500.0f)
                        .Delta(1.0f)
                    ]
	                
                ]


        	+ SVerticalBox::Slot()
		 .AutoHeight()
		 .Padding(8.0f, 5.0f)
		 [
			 SNew(SHorizontalBox)
			 + SHorizontalBox::Slot()
			 .FillWidth(0.4f)
			 .VAlign(VAlign_Center)
			 [
				 SNew(STextBlock)
				 .Text(LOCTEXT("MapSizeLabel", "Map Size:"))
				 .Font(FAppStyle::GetFontStyle("PropertyWindow.NormalFont"))
			 ]
			 + SHorizontalBox::Slot()
			 .FillWidth(0.6f)
			 [
				 SNew(SComboBox<TSharedPtr<int32>>)
				 .OptionsSource(&MapSizeOptions)
				 .OnSelectionChanged(this, &SPathfindingTuner::OnMapSizeSelectionChanged)
				 .OnGenerateWidget_Lambda([](TSharedPtr<int32> Item)
				 {
					 return SNew(STextBlock).Text(FText::AsNumber(*Item));
				 })
				 .Content()
				 [
					 SNew(STextBlock)
					 .Text(this, &SPathfindingTuner::GetSelectedMapSizeText)
				 ]
			 ]
		 ]
        ]
    	
    .InitiallyCollapsed(bInitiallyCollapsedOctreeSettings)
    .OnAreaExpansionChanged_Raw(this, &SPathfindingTuner::OnOctreeSettingsCategoryExpansionChanged);
}

FText SPathfindingTuner::GetBakedDataSaveDirectoryAsText() const
{
	// 이전에 만든 헬퍼 함수를 사용해 선택된 볼륨 목록을 가져옵니다.
	const TArray<AOctreeVoxelVolume*> SelectedVolumes = GetSelectedVolumes();

	if (SelectedVolumes.IsEmpty())
	{
		// 선택된 볼륨이 없으면 빈 텍스트를 반환합니다.
		return FText::GetEmpty();
	}

	// 첫 번째 볼륨의 경로를 '기준'으로 삼습니다.
	const FString FirstPath = SelectedVolumes[0]->BakedDataSaveDirectory.Path;

	// 두 번째 볼륨부터 순회하며 기준 경로와 다른 것이 있는지 확인합니다.
	for (int32 i = 1; i < SelectedVolumes.Num(); ++i)
	{
		if (SelectedVolumes[i]->BakedDataSaveDirectory.Path != FirstPath)
		{
			// 경로가 하나라도 다르면, 빈 텍스트를 반환합니다.
			// 이렇게 하면 SEditableTextBox의 HintText("Multiple Values" 등으로 설정 가능)가 표시됩니다.
			return FText::GetEmpty();
		}
	}

	// 모든 볼륨의 경로가 동일하면, 그 경로를 반환합니다.
	return FText::FromString(FirstPath);
}

/** LastSelectedActors에서 AOctreeVoxelVolume 타입의 액터만 반환합니다. */
TArray<AOctreeVoxelVolume*> SPathfindingTuner::GetSelectedVolumes() const
{
	TArray<AOctreeVoxelVolume*> SelectedVolumes;
	for (const TWeakObjectPtr<AActor>& ActorPtr : LastSelectedActors)
	{
		if (AOctreeVoxelVolume* Volume = Cast<AOctreeVoxelVolume>(ActorPtr.Get()))
		{
			SelectedVolumes.Add(Volume);
		}
	}
	return SelectedVolumes;
}

void SPathfindingTuner::OnBakedDataSaveDirectoryTextCommitted(const FText& InText, ETextCommit::Type CommitInfo)
{
	if (CommitInfo == ETextCommit::OnEnter || CommitInfo == ETextCommit::OnUserMovedFocus)
	{
		SetBakedDataSaveDirectory(InText.ToString());
	}
}

// In SPathfindingTuner.cpp

FReply SPathfindingTuner::OnBrowseButtonClicked()
{
	IDesktopPlatform* DesktopPlatform = FDesktopPlatformModule::Get();
	if (DesktopPlatform)
	{
		const void* ParentWindowHandle = FSlateApplication::Get().GetActiveTopLevelWindow()->GetNativeWindow()->GetOSWindowHandle();
		FString SelectedDirectory;
		const FString DefaultPath = FPaths::ProjectContentDir();
		const FString Title = TEXT("Select Save Directory");

		if (DesktopPlatform->OpenDirectoryDialog(ParentWindowHandle, Title, DefaultPath, SelectedDirectory))
		{
			// --- [수정된 최종 유효성 검사 로직] ---

			// 1. Content 디렉토리의 절대 경로를 가져옵니다.
			// FPaths::ConvertRelativePathToFull은 상대 경로를 절대 경로로 변환해 줍니다.
			const FString AbsoluteContentDir = FPaths::ConvertRelativePathToFull(FPaths::ProjectContentDir());

			// 2. 두 경로 모두 슬래시(/)를 사용하도록 표준화하여 비교의 일관성을 높입니다.
			FString StandardizedSelectedDir = SelectedDirectory;
			FPaths::MakeStandardFilename(StandardizedSelectedDir);

			FString StandardizedContentDir = AbsoluteContentDir;
			FPaths::MakeStandardFilename(StandardizedContentDir);

			// 3. 이제 두 절대 경로를 비교합니다.
			if (!StandardizedSelectedDir.StartsWith(StandardizedContentDir))
			{
				// 선택된 경로가 Content 폴더 내에 있지 않은 경우
				FMessageDialog::Open(EAppMsgType::Ok, LOCTEXT("InvalidPathMsg", "Please select a directory inside the project's Content folder."));
				return FReply::Handled(); // 경로 설정을 거부하고 함수 종료
			}
			// --- [유효성 검사 로직 끝] ---

			// 유효성 검사를 통과했으므로, 상대 경로로 변환하여 저장합니다.
			FString RelativePath = SelectedDirectory;
			FPaths::MakePathRelativeTo(RelativePath, *AbsoluteContentDir); // 기준 경로도 절대 경로 사용
			SetBakedDataSaveDirectory(RelativePath);
		}
	}
	return FReply::Handled();
}

// SPathfindingTuner.cpp

void SPathfindingTuner::SetBakedDataSaveDirectory(const FString& NewPath)
{
	// 1. 헬퍼 함수를 사용해 선택된 모든 유효한 볼륨을 가져옵니다.
	const TArray<AOctreeVoxelVolume*> SelectedVolumes = GetSelectedVolumes();

	// 2. 변경할 대상이 있는지 확인합니다.
	if (SelectedVolumes.IsEmpty())
	{
		ShowNoOctreeSelectedWarning();
		return;
	}

	// 3. Undo/Redo를 위해 트랜잭션을 시작합니다. (루프 바깥에!)
	const FScopedTransaction Transaction(LOCTEXT("SetSaveDirectoryTransaction_Setter", "Set Baked Data Save Directory"));

	// 4. 선택된 모든 볼륨을 순회합니다.
	for (AOctreeVoxelVolume* Volume : SelectedVolumes)
	{
		if (Volume)
		{
			// 5. 각 볼륨에 대해 Modify()를 호출하고 값을 설정합니다.
			Volume->Modify();
			Volume->BakedDataSaveDirectory.Path = NewPath;
			Volume->MarkPackageDirty();
		}
	}
    
	// 6. (중요) UI를 새로고침하여 변경 사항을 즉시 반영합니다.
	// 이 함수가 호출되었다는 것은 값이 변경되었다는 의미이므로,
	// UI의 텍스트 박스도 갱신되어야 합니다.
	Invalidate(EInvalidateWidgetReason::LayoutAndVolatility);
}

TSharedRef<SWidget> SPathfindingTuner::BuildBakedDataSettingsSection()
{
    return SNew(SExpandableArea)
        .IsEnabled(this, &SPathfindingTuner::IsOctreeSettingsEnabled)
        .HeaderContent()
        [
            SNew(STextBlock)
                .Text(LOCTEXT("BakedDataSettingsCategory", "베이크된 데이터 설정 (Baked Data Settings)"))
                .Font(FAppStyle::Get().GetFontStyle("DetailsView.CategoryFontStyle"))
                .ShadowOffset(FVector2D(1.0f, 1.0f))
        ]
        .BodyContent()
        [
            SNew(SVerticalBox)
            // BakedOctreeData 에셋 선택
            + SVerticalBox::Slot()
            .AutoHeight()
            .Padding(8.0f, 5.0f)
            [
                SNew(SHorizontalBox)
                + SHorizontalBox::Slot()
                .FillWidth(0.4f)
                .VAlign(VAlign_Center)
                [
                    SNew(STextBlock)
                    .Text(LOCTEXT("BakedOctreeDataLabel", "Baked Octree Data Asset:"))
                    .Font(FAppStyle::GetFontStyle("PropertyWindow.NormalFont"))
                ]
                + SHorizontalBox::Slot()
                .FillWidth(0.6f)
                [
                    SNew(SVerticalBox)

                    // 1. 기존의 오브젝트 선택 박스
                    + SVerticalBox::Slot()
                    .AutoHeight()
                    [
                        SNew(SObjectPropertyEntryBox)
                        .AllowedClass(UOctreeDataAsset::StaticClass())
                        .ObjectPath(this, &SPathfindingTuner::GetBakedOctreeDataPath)
                        .OnObjectChanged(this, &SPathfindingTuner::OnBakedOctreeDataChanged)
                    ]

                    // ▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼
                    // 2. 선택된 에셋의 디렉토리 경로를 '레이블과 함께' 보여주는 부분 (수정됨)
                    // ▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼
                    + SVerticalBox::Slot()
                    .AutoHeight()
                    .Padding(0, 4, 0, 0) // 위에 살짝 공간을 줍니다.
                    [
                        SNew(SHorizontalBox)

                        // "Path:" 레이블
                        + SHorizontalBox::Slot()
                        .AutoWidth() // 레이블의 크기만큼만 공간을 차지합니다.
                        .VAlign(VAlign_Center)
                        .Padding(0, 0, 5, 0) // 오른쪽 여백을 줘서 경로와 분리합니다.
                        [
                            SNew(STextBlock)
                            .Text(LOCTEXT("PathLabel", "Path:"))
                            .Font(FAppStyle::GetFontStyle("PropertyWindow.NormalFont")) // 일반 폰트 사용
                            .ColorAndOpacity(FSlateColor(FLinearColor(0.7f, 0.7f, 0.7f))) // 약간 밝은 회색
                        ]

                        // 실제 경로 텍스트
                        + SHorizontalBox::Slot()
                        .FillWidth(1.0f) // 남은 공간을 모두 차지합니다.
                        .VAlign(VAlign_Center)
                        [
                            SNew(STextBlock)
                            .Text(this, &SPathfindingTuner::GetBakedDataFullPathAsText) // C++ 함수는 그대로 사용
                            .Font(FAppStyle::GetFontStyle("PropertyWindow.ItalicFont"))
                            .ColorAndOpacity(FSlateColor(FLinearColor(0.5f, 0.5f, 0.5f)))
                        ]
                    ]
                    // ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲
                    // 수정 끝
                    // ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲
                ]
            ]
        ];
}

// SPathfindingTuner.cpp (소스 파일에 구현)
FText SPathfindingTuner::GetBakedDataFullPathAsText() const
{
	const TArray<AOctreeVoxelVolume*> SelectedVolumes = GetSelectedVolumes();
	if (SelectedVolumes.IsEmpty())
	{
		return FText::GetEmpty();
	}

	const UOctreeDataAsset* FirstDataAsset = SelectedVolumes[0]->GetBakedOctreeData();

	for (int32 i = 1; i < SelectedVolumes.Num(); ++i)
	{
		if (SelectedVolumes[i]->GetBakedOctreeData() != FirstDataAsset)
		{
			return LOCTEXT("MultipleValues", "Multiple Values");
		}
	}

	if (FirstDataAsset)
	{
		// ▼▼▼ 핵심 변경 부분 ▼▼▼
		// 전체 경로에서 파일 이름을 제외한 디렉토리 경로만 가져옵니다.
		const FString FullPath = FirstDataAsset->GetPathName();
		const FString DirectoryPath = FPaths::GetPath(FullPath);
		return FText::FromString(DirectoryPath);
		// ▲▲▲ 핵심 변경 끝 ▲▲▲
	}

	return LOCTEXT("NonePath", "None");
}

TSharedRef<SWidget> SPathfindingTuner::BuildAgentSettingsSection()
{
      const bool bInitiallyCollapsedAgent = !bIsAgentSettingsCategoryExpanded;

    return SNew(SExpandableArea)
    	.IsEnabled(this, &SPathfindingTuner::IsAgentSettingsEnabled)
        .HeaderContent()
        [
            SNew(STextBlock)
                .Text(LOCTEXT("AgentSettingsCategory", "에이전트 설정 (Agent Settings)"))
                .Font(FAppStyle::Get().GetFontStyle("DetailsView.CategoryFontStyle"))
                .ShadowOffset(FVector2D(1.0f, 1.0f))
        ]
        .BodyContent()
        [
            SNew(SVerticalBox)
           // --- 0. 선택된 에이전트 정보 표시 (새로 추가) ---
      + SVerticalBox::Slot()
      .AutoHeight()
           .Padding(8.0f, 5.0f) // 다른 설정 항목과 패딩 통일
         [
            SNew(SHorizontalBox) // 수평 박스 추가
            + SHorizontalBox::Slot()
            .FillWidth(0.4f) // 레이블이 차지할 공간
            .VAlign(VAlign_Center)
            [
               SNew(STextBlock)
               .Text(LOCTEXT("SelectedActorLabel", "Selected Actor:")) // 고정 레이블
               .Font(FAppStyle::GetFontStyle("PropertyWindow.NormalFont")) // 일반 폰트 사용
            ]
            + SHorizontalBox::Slot()
            .FillWidth(0.6f) // 액터 이름이 차지할 공간
            .VAlign(VAlign_Center)
            [
               SNew(STextBlock)
               .Text(this, &SPathfindingTuner::GetSelectedAgentName) // 바인딩된 액터 이름
               .Font(FAppStyle::GetFontStyle("PropertyWindow.NormalFont")) // 일반 폰트 사용
               .ColorAndOpacity(FLinearColor(0.8f, 0.8f, 1.0f, 1.0f)) // 색상 유지
               .ShadowOffset(FVector2D(1.0f, 1.0f))
            ]
      ]
                // Radius 설정
                + SVerticalBox::Slot()
                .AutoHeight()
                .Padding(8.0f, 5.0f)
                [
                    SNew(SHorizontalBox)
                    + SHorizontalBox::Slot()
                    .FillWidth(0.4f)
                    .VAlign(VAlign_Center)
                    [
                        SNew(STextBlock)
                        .Text(LOCTEXT("AgentRadiusLabel", "Agent Radius:"))
                        .Font(FAppStyle::GetFontStyle("PropertyWindow.NormalFont"))
                    ]
                    + SHorizontalBox::Slot()
                    .FillWidth(0.6f)
                    [
                        SNew(SNumericEntryBox<float>)
                        .Value(this, &SPathfindingTuner::GetAgentRadius)
                        .OnValueCommitted(this, &SPathfindingTuner::OnAgentRadiusCommitted)
                        .MinValue(0.1f)
                        .MaxValue(1000.0f)
                        .MinSliderValue(10.0f)
                        .MaxSliderValue(200.0f)
                        .Delta(1.0f)
                    ]
                ]

                // Movement Speed 설정
                + SVerticalBox::Slot()
                .AutoHeight()
                .Padding(8.0f, 5.0f)
                [
                    SNew(SHorizontalBox)
                    + SHorizontalBox::Slot()
                    .FillWidth(0.4f)
                    .VAlign(VAlign_Center)
                    [
                        SNew(STextBlock)
                        .Text(LOCTEXT("MovementSpeedLabel", "Movement Speed:"))
                        .Font(FAppStyle::GetFontStyle("PropertyWindow.NormalFont"))
                    ]
                    + SHorizontalBox::Slot()
                    .FillWidth(0.6f)
                    [
                        SNew(SNumericEntryBox<float>)
                        .Value(this, &SPathfindingTuner::GetMovementSpeed)
                        .OnValueCommitted(this, &SPathfindingTuner::OnMovementSpeedCommitted)
                        .MinValue(1.0f)
                        .MaxValue(1000.0f)
                        .MinSliderValue(100.0f)
                        .MaxSliderValue(500.0f)
                        .Delta(10.0f)
                    ]
                ]

                // Target Actor 선택
                + SVerticalBox::Slot()
                .AutoHeight()
                .Padding(8.0f, 5.0f)
                [
                    SNew(SHorizontalBox)
                    + SHorizontalBox::Slot()
                    .FillWidth(0.4f)
                    .VAlign(VAlign_Center)
                    [
                        SNew(STextBlock)
                        .Text(LOCTEXT("TargetActorLabel", "Target Actor:"))
                        .Font(FAppStyle::GetFontStyle("PropertyWindow.NormalFont"))
                    ]
                    + SHorizontalBox::Slot()
                    .FillWidth(0.6f)
                    [
                        SNew(SObjectPropertyEntryBox)
                        .AllowedClass(AActor::StaticClass()) // 모든 액터 클래스 허용
                        .ObjectPath(this, &SPathfindingTuner::GetTargetActorPath) 
                        .OnObjectChanged(this, &SPathfindingTuner::OnTargetActorChanged)
                       .DisplayBrowse(false)
                    
                    ]
                ]
        ]
    .InitiallyCollapsed(bInitiallyCollapsedAgent)
    .OnAreaExpansionChanged_Raw(this, &SPathfindingTuner::OnAgentSettingsCategoryExpansionChanged);
} 

bool SPathfindingTuner::IsAgentSettingsCategoryExpanded() const
{
    return bIsAgentSettingsCategoryExpanded;
}

void SPathfindingTuner::OnAgentSettingsCategoryExpansionChanged(bool bIsExpanded)
{
    bIsAgentSettingsCategoryExpanded = bIsExpanded;
}

// Radius
TOptional<float> SPathfindingTuner::GetAgentRadius() const
{
   // 만약 AgentRadius 값이 혼합 상태라면 (여러 액터의 값이 다르면)
   if (bIsAgentRadiusMixed)
   {
      return TOptional<float>(); // UI에 "---" (혼합 값)을 표시하도록 빈 Optional 반환
   }
   // 혼합 상태가 아니거나 단일 액터만 선택된 경우, 캐시된 값을 반환
   return CachedAgentRadiusValue; 
}

// SPathfindingTuner.cpp (OnAgentRadiusCommitted 함수)

// SPathfindingTuner.cpp
void SPathfindingTuner::OnAgentRadiusCommitted(float NewValue, ETextCommit::Type CommitType)
{

	// IsAgentSettingsEnabled() 덕분에 이 함수가 호출될 때는
	// GEditor가 유효하고, PathfindingComponent를 가진 액터가 최소 하나 이상 선택된 상태임이 보장됩니다.
    
	GEditor->BeginTransaction(LOCTEXT("SetAgentRadius", "Set Agent Radius"));
    
	for (const TWeakObjectPtr<AActor>& ActorPtr : LastSelectedActors)
	{
		if (AActor* Actor = ActorPtr.Get())
		{
			if (UPathfindingComponent* Comp = Actor->FindComponentByClass<UPathfindingComponent>())
			{
				Comp->Modify();
				Comp->AgentRadius = NewValue;
			}
		}
	}
    
	GEditor->EndTransaction();

	UE_LOG(LogTemp, Log, TEXT("AgentRadius committed to: %f"), NewValue);
    
	// UI 캐시된 값 업데이트 및 혼합 상태 해제
	CachedAgentRadiusValue = NewValue;
	bIsAgentRadiusMixed = false;
	Invalidate(EInvalidateWidgetReason::LayoutAndVolatility);
}

// Movement Speed
TOptional<float> SPathfindingTuner::GetMovementSpeed() const
{
   if (bIsMovementSpeedMixed)
   {
      return TOptional<float>();
   }
   return CachedMovementSpeedValue;
}

// SPathfindingTuner.cpp (OnMovementSpeedCommitted 함수)

void SPathfindingTuner::OnMovementSpeedCommitted(float NewValue, ETextCommit::Type CommitType)
{
   if (GEditor && LastSelectedActors.Num() > 0)
   {
      GEditor->BeginTransaction(LOCTEXT("SetMovementSpeed", "Set Movement Speed"));
      int32 ModifiedCount = 0;
      for (TWeakObjectPtr<AActor> ActorPtr : LastSelectedActors)
      {
         if (AActor* Actor = ActorPtr.Get())
         {
            if (UPathfindingComponent* Comp = Actor->FindComponentByClass<UPathfindingComponent>())
            {
               Comp->Modify();
               Comp->MovementSpeed = NewValue;
               Actor->MarkPackageDirty();
               ModifiedCount++;
            }
         }
      }
      GEditor->EndTransaction();

      if (ModifiedCount > 0)
      {
         UE_LOG(LogTemp, Warning, TEXT("MovementSpeed committed to: %f for %d selected agents."), NewValue, ModifiedCount);
      }
      else
      {
         UE_LOG(LogTemp, Warning, TEXT("OnMovementSpeedCommitted: No PathfindingComponent found among selected actors."));
      }

      CachedMovementSpeedValue = NewValue;
      bIsMovementSpeedMixed = false;
      Invalidate(EInvalidateWidgetReason::LayoutAndVolatility);
   }
   else
   {
      UE_LOG(LogTemp, Warning, TEXT("OnMovementSpeedCommitted: No agents selected or GEditor is not valid. Cannot set MovementSpeed."));
   }
}

// Target Actor
FString SPathfindingTuner::GetTargetActorPath() const
{
   // 만약 TargetActor 값이 혼합 상태라면
   if (bIsTargetActorMixed)
   {
      return FString("---"); // UI에 "---" (혼합 값)을 표시하도록 빈 문자열 반환
   }

   // 혼합 상태가 아니거나 단일 액터만 선택된 경우, 캐시된 값을 반환
   if (AActor* TargetActor = CachedTargetActorValue.Get()) // TWeakObjectPtr에서 실제 포인터 가져와 유효성 검사
   {
      return TargetActor->GetPathName(); // 액터의 경로 반환
   }
    
   return FString(); // 유효한 액터가 없으면 빈 문자열 반환
}

// SPathfindingTuner.cpp (OnTargetActorChanged 함수)

void SPathfindingTuner::OnTargetActorChanged(const FAssetData& AssetData)
{
    // GEditor가 유효하고, 선택된 액터가 하나라도 있는지 확인합니다.
    if (GEditor && LastSelectedActors.Num() > 0) // <--- CachedPathfindingComponent.IsValid() 대신 LastSelectedActors.Num() > 0 사용
    {
        GEditor->BeginTransaction(LOCTEXT("SetTargetActor", "Set Target Actor"));
        
        // AssetData에서 새로운 TargetActor를 가져옵니다.
        AActor* NewTargetActor = Cast<AActor>(AssetData.GetAsset());
        int32 ModifiedCount = 0; // 몇 개의 에이전트가 수정되었는지 카운트

        // 선택된 모든 액터를 순회합니다.
        for (TWeakObjectPtr<AActor> ActorPtr : LastSelectedActors)
        {
            if (AActor* Actor = ActorPtr.Get()) // 액터 포인터가 유효한지 확인
            {
                // 액터에서 UPathfindingComponent를 찾습니다.
                UPathfindingComponent* Comp = Actor->FindComponentByClass<UPathfindingComponent>();
                if (Comp) // 컴포넌트가 유효한지 확인
                {
                    Comp->Modify(); // 컴포넌트 수정 가능 상태로 만듭니다.
                    Comp->TargetActor = NewTargetActor; // 새로운 TargetActor 할당
                    Actor->MarkPackageDirty(); // 액터의 패키지를 더티로 표시하여 저장 가능하게 함
                    ModifiedCount++;
                }
            }
        }
        GEditor->EndTransaction();

        // 로그 메시지 출력
        if (ModifiedCount > 0)
        {
            UE_LOG(LogTemp, Warning, TEXT("TargetActor committed to: %s for %d selected agents."), 
                NewTargetActor ? *NewTargetActor->GetActorLabel() : TEXT("None"), ModifiedCount);
        }
        else
        {
            UE_LOG(LogTemp, Warning, TEXT("OnTargetActorChanged: No PathfindingComponent found among selected actors."));
        }

        // UI 캐시된 값 업데이트 및 혼합 상태 해제
        CachedTargetActorValue = NewTargetActor;
        bIsTargetActorMixed = false;
    }
    else
    {
        // 선택된 액터가 없거나 GEditor가 유효하지 않을 때 경고
        UE_LOG(LogTemp, Warning, TEXT("OnTargetActorChanged: No agents selected or GEditor is not valid. Cannot set TargetActor."));
        // ShowNoAgentSelectedWarning(); // 이 함수는 제거하거나 실제 경고 메시지 박스 등으로 대체
    }
   
    Invalidate(EInvalidateWidgetReason::LayoutAndVolatility); // UI 새로고침
}


// SPathfindingTuner.cpp

FString SPathfindingTuner::GetBakedOctreeDataPath() const
{
	// 1. 선택된 모든 볼륨을 가져옵니다.
	const TArray<AOctreeVoxelVolume*> SelectedVolumes = GetSelectedVolumes();

	if (SelectedVolumes.IsEmpty())
	{
		return FString(); // 선택된 볼륨이 없으면 빈 경로 반환
	}

	// 2. 첫 번째 볼륨의 데이터 에셋을 '기준'으로 삼습니다.
	const UOctreeDataAsset* FirstDataAsset = SelectedVolumes[0]->GetBakedOctreeData();

	// 3. 나머지 볼륨들을 순회하며 기준 에셋과 다른 것이 있는지 확인합니다.
	for (int32 i = 1; i < SelectedVolumes.Num(); ++i)
	{
		if (SelectedVolumes[i]->GetBakedOctreeData() != FirstDataAsset)
		{
			// 4. 데이터 에셋이 하나라도 다르면, 빈 경로를 반환하여 "혼합 값" 상태를 만듭니다.
			return FString();
		}
	}

	// 5. 모든 볼륨의 데이터 에셋이 동일하면, 그 에셋의 경로를 반환합니다.
	if (FirstDataAsset)
	{
		return FirstDataAsset->GetPathName();
	}

	// 모든 에셋이 비어있는(None) 경우, 빈 경로를 반환합니다.
	return FString();
}

// SPathfindingTuner.cpp

void SPathfindingTuner::OnBakedOctreeDataChanged(const FAssetData& AssetData)
{
	// 1. 선택된 모든 볼륨을 가져옵니다.
	const TArray<AOctreeVoxelVolume*> SelectedVolumes = GetSelectedVolumes();
	if (SelectedVolumes.IsEmpty())
	{
		ShowNoOctreeSelectedWarning();
		return; // 변경할 대상이 없으면 종료
	}

	// 2. AssetData에서 새로운 데이터 에셋 오브젝트를 가져옵니다.
	UOctreeDataAsset* NewDataAsset = Cast<UOctreeDataAsset>(AssetData.GetAsset());

	// 3. Undo/Redo를 위해 트랜잭션을 시작합니다.
	const FScopedTransaction Transaction(LOCTEXT("SetBakedDataTransaction", "Set Baked Octree Data"));

	// 4. 선택된 모든 볼륨을 순회하며 값을 변경합니다.
	for (AOctreeVoxelVolume* Volume : SelectedVolumes)
	{
		if (Volume)
		{
			Volume->Modify();
			// AOctreeVoxelVolume에 SetBakedOctreeData 함수가 있다고 가정합니다.
			Volume->SetBakedOctreeData(NewDataAsset);
			Volume->MarkPackageDirty();
		}
	}
    
	// UI를 새로고침하여 변경 사항을 즉시 반영합니다.
	Invalidate(EInvalidateWidgetReason::LayoutAndVolatility);
}

FText SPathfindingTuner::GetSelectedMapSizeText() const
{
	const TArray<AOctreeVoxelVolume*> SelectedVolumes = GetSelectedVolumes();
	if (SelectedVolumes.IsEmpty())
	{
		return LOCTEXT("NoVolumeSelected", "---");
	}

	// 첫 번째 볼륨의 MapSize를 기준으로 삼습니다.
	const FIntVector FirstMapSize = SelectedVolumes[0]->MapSize;

	// 기준 MapSize가 균일한지 (X=Y=Z) 확인합니다.
	if (FirstMapSize.X != FirstMapSize.Y || FirstMapSize.Y != FirstMapSize.Z)
	{
		return LOCTEXT("CustomMapSize", "Custom Size");
	}

	// 나머지 볼륨들을 순회하며 기준과 다른 값이 있는지 확인합니다.
	for (int32 i = 1; i < SelectedVolumes.Num(); ++i)
	{
		if (SelectedVolumes[i]->MapSize != FirstMapSize)
		{
			return LOCTEXT("MultipleMapSizes", "Multiple Values");
		}
	}

	// 모든 값이 동일하면, 그 값을 텍스트로 반환합니다.
	return FText::AsNumber(FirstMapSize.X);
}

void SPathfindingTuner::OnMapSizeSelectionChanged(
	TSharedPtr<int32> NewSelection, ESelectInfo::Type SelectType)
{
	// 사용자가 직접 선택했을 때만 실행합니다.
	if (SelectType == ESelectInfo::Type::OnMouseClick || SelectType == ESelectInfo::Type::OnKeyPress)
	{
		if (!NewSelection.IsValid()) return;

		const int32 NewSize = *NewSelection;
		const FIntVector NewMapSize(NewSize, NewSize, NewSize);

		const TArray<AOctreeVoxelVolume*> SelectedVolumes = GetSelectedVolumes();
		if (SelectedVolumes.IsEmpty()) return;

		const FScopedTransaction Transaction(LOCTEXT("SetMapSizeTransaction", "Set Map Size"));

		for (AOctreeVoxelVolume* Volume : SelectedVolumes)
		{
			if (Volume)
			{
				Volume->Modify();
				Volume->MapSize = NewMapSize;
				// MapSize는 액터의 외형에 영향을 주므로, OnConstruction을 다시 실행해야 합니다.
				Volume->RerunConstructionScripts();
			}
		}
	}
}

bool SPathfindingTuner::IsAgentSettingsEnabled() const
{
	// 선택된 액터가 없으면 비활성화
	if (LastSelectedActors.Num() == 0)
	{
		return false;
	}

	// 선택된 액터 중에 PathfindingComponent를 가진 것이 하나라도 있는지 확인
	for (const TWeakObjectPtr<AActor>& ActorPtr : LastSelectedActors)
	{
		if (AActor* Actor = ActorPtr.Get())
		{
			// OctreeVolume은 에이전트가 아니므로 무시
			if (Actor->IsA<AOctreeVoxelVolume>())
			{
				continue;
			}
            
			// PathfindingComponent가 발견되면 즉시 true를 반환
			if (Actor->FindComponentByClass<UPathfindingComponent>())
			{
				return true;
			}
		}
	}

	// 루프가 끝날 때까지 PathfindingComponent를 찾지 못했다면 비활성화
	return false;
}

FText SPathfindingTuner::GetSelectedAgentName() const
{
	// 1. 헬퍼 함수를 호출하여 선택된 에이전트 목록을 가져옵니다.
	const TArray<AActor*> SelectedAgents = GetSelectedAgents();
	const int32 NumSelected = SelectedAgents.Num();

	if (NumSelected == 0)
	{
		// 2. 선택된 에이전트가 없을 경우
		return LOCTEXT("NoAgentSelected", "No Agent Selected");
	}
	else if (NumSelected == 1)
	{
		// 3. 하나의 에이전트만 선택된 경우: 해당 에이전트의 이름을 반환
		// GetActorLabel()이 에디터에서 보이는 이름이라 더 좋습니다.
		return FText::FromString(SelectedAgents[0]->GetActorLabel());
	}
	else // NumSelected > 1
	{
		// 4. 여러 에이전트가 선택된 경우: 개수를 요약해서 반환
		return FText::Format(LOCTEXT("MultipleAgentsSelected", "{0} Agents Selected"), FText::AsNumber(NumSelected));
	}
}

TArray<AActor*> SPathfindingTuner::GetSelectedAgents() const
{
	TArray<AActor*> SelectedAgents;
	for (const TWeakObjectPtr<AActor>& ActorPtr : LastSelectedActors)
	{
		if (AActor* Actor = ActorPtr.Get())
		{
			if (Actor->FindComponentByClass<UPathfindingComponent>())
			{
				SelectedAgents.Add(Actor);
			}
		}
	}
	return SelectedAgents;
}

// --- Performance Settings Callbacks ---
TOptional<int32> SPathfindingTuner::GetCurrentNumThreads() const
{
   if (PathfindingSubsystem.IsValid())
   {
      return PathfindingSubsystem->GetNumWorkerThreads();
   }
   return TOptional<int32>();
}


void SPathfindingTuner::OnNumThreadsCommitted(int32 NewValue, ETextCommit::Type CommitInfo)
{
   if (PathfindingSubsystem.IsValid())
   {
      PathfindingSubsystem->SetNumWorkerThreads(NewValue);
      PathfindingSubsystem->SaveConfig();
      UE_LOG(LogTemp, Warning, TEXT("PathfindingTuner: Num Threads Committed to: %d. Config Saved."), NewValue);
      Invalidate(EInvalidateWidgetReason::LayoutAndVolatility);
   }
   else
   {
      UE_LOG(LogTemp, Error, TEXT("PathfindingTuner: PathfindingSubsystem is NOT valid. Cannot commit value."));
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

// --- Octree Volume Settings Callbacks ---

bool SPathfindingTuner::IsOctreeSettingsEnabled() const
{
	// 1. 선택된 액터가 하나도 없으면 비활성화합니다.
	if (LastSelectedActors.Num() == 0)
	{
		return false;
	}

	// 2. 선택된 모든 액터가 AOctreeVoxelVolume인지 확인합니다.
	for (const TWeakObjectPtr<AActor>& ActorPtr : LastSelectedActors)
	{
		// 포인터가 유효하지 않거나, AOctreeVoxelVolume이 아닌 다른 클래스가 섞여 있으면 비활성화합니다.
		if (!ActorPtr.IsValid() || !ActorPtr->IsA<AOctreeVoxelVolume>())
		{
			return false;
		}
	}

	// 3. 루프를 무사히 통과했다면, 선택된 모든 액터가 AOctreeVoxelVolume이라는 의미이므로 활성화합니다.
	return true;
}

// 액터 선택 변경 시 호출되는 함수
void SPathfindingTuner::OnActorSelectionChanged()
{
	// 1. GEditor를 통해 현재 선택된 액터 목록을 직접 가져옵니다.
	LastSelectedActors.Empty();
	TArray<UObject*> SelectedObjects;
	GEditor->GetSelectedActors()->GetSelectedObjects(SelectedObjects);

	for (UObject* SelectedObject : SelectedObjects)
	{
		if (AActor* Actor = Cast<AActor>(SelectedObject))
		{
			LastSelectedActors.Add(Actor);
		}
	}

	// 2. 값 계산 함수 호출 (이 부분은 이전 설명과 동일)
	UpdateAgentSettingsCache();
	UpdateOctreeSettingsCache();

	// 3. UI 새로고침 (이 부분도 동일)
	Invalidate(EInvalidateWidgetReason::LayoutAndVolatility);
}

// SPathfindingTuner.cpp

void SPathfindingTuner::UpdateOctreeSettingsCache()
{
	// 먼저, 옥트리 섹션이 활성화될 조건인지 확인합니다.
	// (IsOctreeSettingsEnabled() 함수는 이미 만들어 두었습니다)
	if (!IsOctreeSettingsEnabled())
	{
		// 비활성화 상태라면, 모든 관련 값을 초기화하고 끝냅니다.
		bIsMaxOctreeDepthMixed = false;
		CachedMaxOctreeDepth.Reset();
		return;
	}

	// 이제, 선택된 모든 옥트리의 값을 비교합니다.
	TOptional<int32> FirstMaxDepth;

	// 혼합 상태를 false로 초기화
	bIsMaxOctreeDepthMixed = false;

	for (const TWeakObjectPtr<AActor>& ActorPtr : LastSelectedActors)
	{
		// 액터를 AOctreeVoxelVolume으로 변환합니다.
		if (AOctreeVoxelVolume* Volume = Cast<AOctreeVoxelVolume>(ActorPtr.Get()))
		{
			// 첫 번째로 발견된 옥트리의 값을 '기준'으로 삼습니다.
			if (!FirstMaxDepth.IsSet())
			{
				FirstMaxDepth = Volume->GetMaxOctreeDepth();
			}
			// 두 번째 이후 옥트리부터는 '기준' 값과 비교합니다.
			else
			{
				// MaxOctreeDepth 값이 기준과 다른지 확인
				if (FirstMaxDepth.GetValue() != Volume->GetMaxOctreeDepth())
				{
					bIsMaxOctreeDepthMixed = true;
				}
			}
		}
	}

	// 비교가 끝났으면, 최종적으로 UI에 표시될 값을 캐시 변수에 저장합니다.
	CachedMaxOctreeDepth = bIsMaxOctreeDepthMixed ? TOptional<int32>() : FirstMaxDepth;
}

void SPathfindingTuner::UpdateAgentSettingsCache()
{
	// 먼저, 에이전트 섹션이 활성화될 조건인지 확인합니다.
	// (IsAgentSettingsEnabled() 함수는 이미 만들어 두었습니다)
	if (!IsAgentSettingsEnabled())
	{
		// 비활성화 상태라면, 모든 관련 값을 초기화하고 끝냅니다.
		bIsAgentRadiusMixed = false;
		bIsMovementSpeedMixed = false;
		CachedAgentRadiusValue.Reset();
		CachedMovementSpeedValue.Reset();
		return;
	}

	// 이제, 선택된 모든 에이전트의 값을 비교합니다.
	TOptional<float> FirstRadius;
	TOptional<float> FirstSpeed;

	// 혼합 상태를 false로 초기화
	bIsAgentRadiusMixed = false;
	bIsMovementSpeedMixed = false;

	for (const TWeakObjectPtr<AActor>& ActorPtr : LastSelectedActors)
	{
		// 액터에서 PathfindingComponent를 찾습니다.
		if (UPathfindingComponent* Comp = ActorPtr.IsValid() ? ActorPtr->FindComponentByClass<UPathfindingComponent>() : nullptr)
		{
			// 첫 번째로 발견된 에이전트의 값을 '기준'으로 삼습니다.
			if (!FirstRadius.IsSet())
			{
				FirstRadius = Comp->AgentRadius;
				FirstSpeed = Comp->MovementSpeed;
			}
			// 두 번째 이후 에이전트부터는 '기준' 값과 비교합니다.
			else
			{
				// Radius 값이 기준과 다른지 확인
				if (FirstRadius.GetValue() != Comp->AgentRadius)
				{
					bIsAgentRadiusMixed = true;
				}
				// Speed 값이 기준과 다른지 확인
				if (FirstSpeed.GetValue() != Comp->MovementSpeed)
				{
					bIsMovementSpeedMixed = true;
				}
			}
		}
	}

	// 비교가 끝났으면, 최종적으로 UI에 표시될 값을 캐시 변수에 저장합니다.
	// 만약 값이 혼합된 상태(bIs...Mixed == true)이면, 캐시 변수를 비웁니다(Reset).
	CachedAgentRadiusValue = bIsAgentRadiusMixed ? TOptional<float>() : FirstRadius;
	CachedMovementSpeedValue = bIsMovementSpeedMixed ? TOptional<float>() : FirstSpeed;
}

// MaxOctreeDepth
TOptional<int32> SPathfindingTuner::GetMaxOctreeDepth() const
{
	// 1. 선택된 '모든' 볼륨을 가져온다.
	const TArray<AOctreeVoxelVolume*> SelectedVolumes = GetSelectedVolumes();

	if (SelectedVolumes.IsEmpty())
	{
		return TOptional<int32>(); // 선택된 것이 없으면 값을 표시하지 않음
	}

	// 2. 첫 번째 볼륨의 값을 '기준'으로 삼는다.
	const int32 FirstDepth = SelectedVolumes[0]->GetMaxOctreeDepth();

	// 3. 나머지 볼륨들을 순회하며 기준과 다른 값이 있는지 확인한다.
	for (int32 i = 1; i < SelectedVolumes.Num(); ++i)
	{
		if (SelectedVolumes[i]->GetMaxOctreeDepth() != FirstDepth)
		{
			// 4. 하나라도 다르면 "혼합 값" 상태이므로, 빈 Optional을 반환한다.
			return TOptional<int32>();
		}
	}

	// 5. 모든 값이 동일하면, 그 값을 반환한다.
	return FirstDepth;
}

void SPathfindingTuner::OnMaxOctreeDepthCommitted(int32 NewValue, ETextCommit::Type CommitType)
{
	// 사용자가 입력을 확정했을 때만 로직을 실행합니다.
	if (CommitType != ETextCommit::OnEnter && CommitType != ETextCommit::OnUserMovedFocus)
	{
		return;
	}

	// 변경할 대상이 있는지 먼저 확인합니다.
	bool bHasValidTarget = false;
	for (const TWeakObjectPtr<AActor>& ActorPtr : LastSelectedActors)
	{
		if (Cast<AOctreeVoxelVolume>(ActorPtr.Get()))
		{
			bHasValidTarget = true;
			break;
		}
	}

	if (!bHasValidTarget)
	{
		ShowNoOctreeSelectedWarning();
		UE_LOG(LogTemp, Warning, TEXT("OnMaxOctreeDepthCommitted: No valid OctreeVoxelVolume selected."));
		return;
	}

	// FScopedTransaction을 사용하면 Begin/EndTransaction을 자동으로 관리해주고 더 안전합니다.
	const FScopedTransaction Transaction(LOCTEXT("SetMaxOctreeDepthTransaction", "Set Max Octree Depth"));

	// 선택된 모든 액터를 순회합니다.
	for (const TWeakObjectPtr<AActor>& ActorPtr : LastSelectedActors)
	{
		// 액터가 유효하고, AOctreeVoxelVolume 타입인지 확인합니다.
		if (AOctreeVoxelVolume* Volume = Cast<AOctreeVoxelVolume>(ActorPtr.Get()))
		{
			// Undo/Redo를 위해 Modify()를 호출합니다.
			Volume->Modify();

			// 새로운 값으로 속성을 설정합니다.
			Volume->SetMaxOctreeDepth(NewValue);

			// 변경 사항을 저장하기 위해 패키지를 더티(dirty) 상태로 만듭니다.
			Volume->MarkPackageDirty();
		}
	}
    
	// 루프가 끝난 후, 모든 액터의 값이 변경되었으므로 UI 캐시를 업데이트합니다.
	UpdateOctreeSettingsCache();
	Invalidate(EInvalidateWidgetReason::LayoutAndVolatility);

	UE_LOG(LogTemp, Log, TEXT("Committed MaxOctreeDepth for all selected OctreeVoxelVolumes to: %d"), NewValue);
}

// SPathfindingTuner.cpp

TOptional<float> SPathfindingTuner::GetVoxelSize() const
{
	// 1. 선택된 모든 볼륨을 가져옵니다.
	const TArray<AOctreeVoxelVolume*> SelectedVolumes = GetSelectedVolumes();

	if (SelectedVolumes.IsEmpty())
	{
		return TOptional<float>(); // 선택된 것이 없으면 값을 표시하지 않음
	}

	// 2. 첫 번째 볼륨의 VoxelExtent를 '기준'으로 삼습니다.
	const float FirstVoxelSize = SelectedVolumes[0]->VoxelExtent;

	// 3. 나머지 볼륨들을 순회하며 기준과 다른 값이 있는지 확인합니다.
	for (int32 i = 1; i < SelectedVolumes.Num(); ++i)
	{
		if (!FMath::IsNearlyEqual(SelectedVolumes[i]->VoxelExtent, FirstVoxelSize))
		{
			// 4. 하나라도 다르면 "혼합 값" 상태이므로, 빈 Optional을 반환합니다.
			// float 비교는 IsNearlyEqual을 사용하는 것이 안전합니다.
			return TOptional<float>();
		}
	}

	// 5. 모든 값이 동일하면, 그 값을 반환합니다.
	return FirstVoxelSize;
}

void SPathfindingTuner::OnVoxelSizeCommitted(float NewValue, ETextCommit::Type CommitType)
{
	if (CommitType != ETextCommit::OnEnter && CommitType != ETextCommit::OnUserMovedFocus)
	{
		return;
	}

	// 1. 선택된 모든 볼륨을 가져옵니다.
	const TArray<AOctreeVoxelVolume*> SelectedVolumes = GetSelectedVolumes();
	if (SelectedVolumes.IsEmpty())
	{
		ShowNoOctreeSelectedWarning();
		return;
	}

	// 2. Undo/Redo를 위해 트랜잭션을 시작합니다.
	const FScopedTransaction Transaction(LOCTEXT("SetVoxelSizeTransaction", "Set Voxel Size"));

	// 3. 모든 볼륨을 순회하며 값을 변경합니다.
	for (AOctreeVoxelVolume* Volume : SelectedVolumes)
	{
		if (Volume)
		{
			Volume->Modify();
			Volume->VoxelExtent = NewValue;
			Volume->MarkPackageDirty();
			Volume->RerunConstructionScripts();
		}
	}
    
	// UI를 새로고침하여 변경 사항을 즉시 반영합니다.
	Invalidate(EInvalidateWidgetReason::LayoutAndVolatility);
}

// SPathfindingTuner.cpp

void SPathfindingTuner::OnStaticObjectFilterChanged(const FAssetData& AssetData)
{
	// 1. 선택된 모든 볼륨을 가져옵니다.
	const TArray<AOctreeVoxelVolume*> SelectedVolumes = GetSelectedVolumes();
	if (SelectedVolumes.IsEmpty())
	{
		return; // 변경할 대상이 없으면 종료
	}

	// 2. AssetData에서 새로운 필터 오브젝트를 가져옵니다.
	UVoxelObstacleFilter* NewFilter = Cast<UVoxelObstacleFilter>(AssetData.GetAsset());

	// 3. Undo/Redo를 위해 트랜잭션을 시작합니다.
	const FScopedTransaction Transaction(LOCTEXT("SetStaticFilterTransaction", "Set Static Obstacle Filter"));

	// 4. 선택된 모든 볼륨을 순회하며 값을 변경합니다.
	for (AOctreeVoxelVolume* Volume : SelectedVolumes)
	{
		if (Volume)
		{
			Volume->Modify();
			// AOctreeVoxelVolume에 SetStaticObstacleFilter 함수가 있다고 가정합니다.
			Volume->SetStaticObstacleFilter(NewFilter);
			Volume->MarkPackageDirty();
		}
	}
    
	// UI를 새로고침하여 변경 사항을 즉시 반영합니다.
	Invalidate(EInvalidateWidgetReason::LayoutAndVolatility);
}

FString SPathfindingTuner::GetStaticObjectFilterPath() const
{
	// 1. 선택된 모든 볼륨을 가져옵니다.
	const TArray<AOctreeVoxelVolume*> SelectedVolumes = GetSelectedVolumes();

	if (SelectedVolumes.IsEmpty())
	{
		return FString(); // 선택된 볼륨이 없으면 빈 경로 반환
	}

	// 2. 첫 번째 볼륨의 필터를 '기준'으로 삼습니다.
	// GetStaticObstacleFilter()가 UVoxelObstacleFilter*를 반환한다고 가정합니다.
	const UVoxelObstacleFilter* FirstFilter = SelectedVolumes[0]->GetStaticObstacleFilter();

	// 3. 나머지 볼륨들을 순회하며 기준 필터와 다른 것이 있는지 확인합니다.
	for (int32 i = 1; i < SelectedVolumes.Num(); ++i)
	{
		if (SelectedVolumes[i]->GetStaticObstacleFilter() != FirstFilter)
		{
			// 4. 필터가 하나라도 다르면, 빈 경로를 반환하여 "혼합 값" 상태를 만듭니다.
			return FString();
		}
	}

	// 5. 모든 볼륨의 필터가 동일하면, 그 필터의 경로를 반환합니다.
	if (FirstFilter)
	{
		return FirstFilter->GetPathName();
	}

	return FString(); // 모든 필터가 비어있는(None) 경우
}

// DynamicObjectFilter (제거됨)

FReply SPathfindingTuner::OnBakeButtonClicked()
{
	const TArray<AOctreeVoxelVolume*> SelectedVolumes = GetSelectedVolumes();
	if (SelectedVolumes.IsEmpty())
	{
		ShowNoOctreeSelectedWarning();
		return FReply::Handled();
	}

	// [사전 검사] - 옵션 2와 동일
	for (AOctreeVoxelVolume* Volume : SelectedVolumes)
	{
		if (Volume->BakedDataSaveDirectory.Path.IsEmpty())
		{
			FText ErrorMsg = FText::Format(LOCTEXT("BakePathNotSetForOne", "Cannot start baking. 'Baked Data Save Directory' is not set for {0}."), FText::FromString(Volume->GetActorLabel()));
			FMessageDialog::Open(EAppMsgType::Ok, ErrorMsg);
			return FReply::Handled();
		}
	}
    
	// [다중 선택 확인]
	if (SelectedVolumes.Num() > 1)
	{
		FText ConfirmationMessage = FText::Format(
			LOCTEXT("MultiBakeConfirmation", "You have selected {0} volumes to bake.\nThis process could take a long time and may cause the editor to become unresponsive.\n\nDo you want to continue?"),
			FText::AsNumber(SelectedVolumes.Num())
		);

		EAppReturnType::Type UserResponse = FMessageDialog::Open(EAppMsgType::YesNo, ConfirmationMessage);

		if (UserResponse != EAppReturnType::Yes)
		{
			// 사용자가 'No'를 선택하면 작업을 취소합니다.
			return FReply::Handled();
		}
	}
    
	// (필요하다면 필터 미설정 경고도 여기에 통합할 수 있습니다)

	// [베이크 실행]
	const FScopedTransaction Transaction(LOCTEXT("BakeOctreeDataTransaction", "Bake Octree Data"));
	for (AOctreeVoxelVolume* Volume : SelectedVolumes)
	{
		Volume->Modify();
		Volume->BakeStaticData();
	}

	return FReply::Handled();
}

void SPathfindingTuner::OnOctreeSettingsCategoryExpansionChanged(bool bIsExpanded)
{
   bIsOctreeSettingsCategoryExpanded = bIsExpanded;
}

void SPathfindingTuner::ShowNoOctreeSelectedWarning() const
{
   FMessageDialog::Open(
      EAppMsgType::Ok, 
      LOCTEXT("NoOctreeSelected", 
         "No Octree Volume selected.\nPlease select a valid Octree Volume to proceed.")
   );
}

void SPathfindingTuner::ShowNoAgentSelectedWarning() const
{
   FMessageDialog::Open(
      EAppMsgType::Ok, 
      LOCTEXT("NoAgentSelected", 
         "No agent assigned.\nPlease select a valid pathfinding agent.")
   );
}

FText SPathfindingTuner::GetSelectedVolumeDisplayText() const
{
    // LastSelectedActors에서 AOctreeVoxelVolume 타입만 필터링하여 개수를 셉니다.
    TArray<AOctreeVoxelVolume*> SelectedVolumes;
    for (const TWeakObjectPtr<AActor>& ActorPtr : LastSelectedActors)
    {
        if (AOctreeVoxelVolume* Volume = Cast<AOctreeVoxelVolume>(ActorPtr.Get()))
        {
            SelectedVolumes.Add(Volume);
        }
    }

    const int32 NumSelected = SelectedVolumes.Num();

    if (NumSelected == 0)
    {
        // 1. 선택된 볼륨이 없을 경우
        return LOCTEXT("NoVolumeSelected", "None");
    }
    else if (NumSelected == 1)
    {
        // 2. 하나의 볼륨만 선택된 경우: 해당 볼륨의 이름을 반환
        return FText::FromString(SelectedVolumes[0]->GetActorLabel()); // GetActorLabel()이 더 사용자 친화적일 수 있습니다.
    }
    else // NumSelected > 1
    {
        // 3. 여러 볼륨이 선택된 경우: 개수를 요약해서 반환
        return FText::Format(LOCTEXT("MultipleVolumesSelected", "{0} Volumes Selected"), FText::AsNumber(NumSelected));
    }
}