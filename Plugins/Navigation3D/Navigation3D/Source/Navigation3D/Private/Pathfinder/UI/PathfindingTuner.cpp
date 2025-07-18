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
				.ToolTipText(FText::FromString(
					TEXT("길찾기 연산에 사용할 워커 스레드의 개수를 설정합니다. \n값을 높이면 동시에 더 많은 길찾기 요청을 처리할 수 있지만, CPU 사용량이 증가합니다.")))
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
					.OnValueChanged(this, &SPathfindingTuner::OnNumThreadsChanged)
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
			// 타겟 볼륨 액터 선택
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
					.Text(LOCTEXT("TargetVolumeLabel", "Target Octree Volume:"))
					.Font(FAppStyle::GetFontStyle("PropertyWindow.NormalFont"))
				]
				+ SHorizontalBox::Slot()
				.FillWidth(0.6f)
				[
					SNew(SObjectPropertyEntryBox)
					.AllowedClass(AOctreeVoxelVolume::StaticClass())
					.ObjectPath(this, &SPathfindingTuner::GetSelectedOctreeVolumePath)
					.OnObjectChanged_Lambda([this](const FAssetData& AssetData)
					{
						if (UObject* Object = AssetData.GetAsset())
						{
							CachedOctreeVolume = Cast<AOctreeVoxelVolume>(Object);
						}
						else
						{
							CachedOctreeVolume = nullptr;
						}
						Invalidate(EInvalidateWidgetReason::LayoutAndVolatility);
					})
					.DisplayBrowse(false)

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
						.ToolTipText(LOCTEXT("BakedDataSaveDirectoryBrowseTooltip",
						                     "Choose a directory from this computer"))
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
			// StaticObjectFilter 설정


		]
		.InitiallyCollapsed(bInitiallyCollapsedOctreeSettings)
		.OnAreaExpansionChanged_Raw(this, &SPathfindingTuner::OnOctreeSettingsCategoryExpansionChanged);
}

FText SPathfindingTuner::GetBakedDataSaveDirectoryAsText() const
{
	if (CachedOctreeVolume.IsValid())
	{
		return FText::FromString(CachedOctreeVolume->BakedDataSaveDirectory.Path);
	}
	return FText::GetEmpty();
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
		const void* ParentWindowHandle = FSlateApplication::Get().GetActiveTopLevelWindow()->GetNativeWindow()->
		                                                          GetOSWindowHandle();
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
				FMessageDialog::Open(EAppMsgType::Ok,
				                     LOCTEXT("InvalidPathMsg",
				                             "Please select a directory inside the project's Content folder."));
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

void SPathfindingTuner::SetBakedDataSaveDirectory(const FString& NewPath)
{
	if (CachedOctreeVolume.IsValid())
	{
		CachedOctreeVolume->Modify();
		CachedOctreeVolume->BakedDataSaveDirectory.Path = NewPath;
	}
}

TSharedRef<SWidget> SPathfindingTuner::BuildBakedDataSettingsSection()
{
	return SNew(SExpandableArea) // 또는 SBorder 등 다른 레이아웃 위젯
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
					SNew(SObjectPropertyEntryBox)
					                             .AllowedClass(UOctreeDataAsset::StaticClass())
 // UOctreeDataAsset 클래스만 허용
					                             .ObjectPath(this, &SPathfindingTuner::GetBakedOctreeDataPath)
					                             .OnObjectChanged(this, &SPathfindingTuner::OnBakedOctreeDataChanged)
					                             .DisplayUseSelected(false) // 돋보기 아이콘 숨김
					                             .DisplayBrowse(false) // 폴더 아이콘 숨김
				]
			]
		];
}

TSharedRef<SWidget> SPathfindingTuner::BuildAgentSettingsSection()
{
	const bool bInitiallyCollapsedAgent = !bIsAgentSettingsCategoryExpanded;

	return SNew(SExpandableArea)
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

void SPathfindingTuner::OnAgentRadiusCommitted(float NewValue, ETextCommit::Type CommitType)
{
	// GEditor가 유효하고, 선택된 액터가 하나라도 있는지 확인합니다.
	if (GEditor && LastSelectedActors.Num() > 0)
	{
		GEditor->BeginTransaction(LOCTEXT("SetAgentRadius", "Set Agent Radius"));
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
					Comp->AgentRadius = NewValue; // 새로운 값 할당
					Actor->MarkPackageDirty(); // 액터의 패키지를 더티로 표시하여 저장 가능하게 함
					ModifiedCount++;
				}
			}
		}
		GEditor->EndTransaction();

		// 로그 메시지 출력
		if (ModifiedCount > 0)
		{
			UE_LOG(LogTemp, Warning, TEXT("AgentRadius committed to: %f for %d selected agents."), NewValue,
			       ModifiedCount);
		}
		else
		{
			UE_LOG(LogTemp, Warning,
			       TEXT("OnAgentRadiusCommitted: No PathfindingComponent found among selected actors."));
		}

		// UI 캐시된 값 업데이트 및 혼합 상태 해제
		CachedAgentRadiusValue = NewValue;
		bIsAgentRadiusMixed = false;
		Invalidate(EInvalidateWidgetReason::LayoutAndVolatility); // UI 새로고침
	}
	else
	{
		// 선택된 액터가 없거나 GEditor가 유효하지 않을 때 경고
		UE_LOG(LogTemp, Warning,
		       TEXT("OnAgentRadiusCommitted: No agents selected or GEditor is not valid. Cannot set AgentRadius."));
		ShowNoAgentSelectedWarning(); // 이 함수는 제거하거나 실제 경고 메시지 박스 등으로 대체
	}
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
			UE_LOG(LogTemp, Warning, TEXT("MovementSpeed committed to: %f for %d selected agents."), NewValue,
			       ModifiedCount);
		}
		else
		{
			UE_LOG(LogTemp, Warning,
			       TEXT("OnMovementSpeedCommitted: No PathfindingComponent found among selected actors."));
		}

		CachedMovementSpeedValue = NewValue;
		bIsMovementSpeedMixed = false;
		Invalidate(EInvalidateWidgetReason::LayoutAndVolatility);
	}
	else
	{
		UE_LOG(LogTemp, Warning,
		       TEXT("OnMovementSpeedCommitted: No agents selected or GEditor is not valid. Cannot set MovementSpeed."));
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
	if (GEditor && LastSelectedActors.Num() > 0)
	// <--- CachedPathfindingComponent.IsValid() 대신 LastSelectedActors.Num() > 0 사용
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
			UE_LOG(LogTemp, Warning,
			       TEXT("OnTargetActorChanged: No PathfindingComponent found among selected actors."));
		}

		// UI 캐시된 값 업데이트 및 혼합 상태 해제
		CachedTargetActorValue = NewTargetActor;
		bIsTargetActorMixed = false;
	}
	else
	{
		// 선택된 액터가 없거나 GEditor가 유효하지 않을 때 경고
		UE_LOG(LogTemp, Warning,
		       TEXT("OnTargetActorChanged: No agents selected or GEditor is not valid. Cannot set TargetActor."));
		// ShowNoAgentSelectedWarning(); // 이 함수는 제거하거나 실제 경고 메시지 박스 등으로 대체
	}

	Invalidate(EInvalidateWidgetReason::LayoutAndVolatility); // UI 새로고침
}


FString SPathfindingTuner::GetBakedOctreeDataPath() const
{
	if (AOctreeVoxelVolume* OctreeVolume = GetSelectedOctreeVolume().Get())
	{
		if (UOctreeDataAsset* BakedData = OctreeVolume->GetBakedOctreeData()) // GetBakedOctreeData() 결과도 널 체크
		{
			return BakedData->GetPathName();
		}
	}
	return FString();
}

void SPathfindingTuner::OnBakedOctreeDataChanged(const FAssetData& AssetData)
{
	if (AOctreeVoxelVolume* OctreeVolume = GetSelectedOctreeVolume().Get())
	{
		if (GEditor)
		{
			UObject* AssetObject = AssetData.GetAsset();
			UOctreeDataAsset* NewBakedData = Cast<UOctreeDataAsset>(AssetObject);

			if (!NewBakedData && AssetObject)
			{
				UE_LOG(LogTemp, Warning,
				       TEXT("OnBakedOctreeDataChanged: Selected asset is not a UOctreeDataAsset. Ignoring."));
				return; // 타입이 맞지 않으면 함수 종료
			}

			GEditor->BeginTransaction(LOCTEXT("SetBakedOctreeData", "Set Baked Octree Data"));
			OctreeVolume->Modify();

			// AssetData에서 UOctreeDataAsset을 가져와 할당합니다.
			OctreeVolume->SetBakedOctreeData(NewBakedData);

			GEditor->EndTransaction();
			OctreeVolume->MarkPackageDirty(); // 액터의 패키지를 더티로 표시하여 저장 가능하게 함

			UE_LOG(LogTemp, Warning, TEXT("OctreeVoxelVolume '%s' BakedOctreeData changed to: %s"),
			       *OctreeVolume->GetName(),
			       OctreeVolume->GetBakedOctreeData() ? *OctreeVolume->GetBakedOctreeData()->GetName() : TEXT("None"));
		}
	}
	else
	{
		ShowNoOctreeSelectedWarning();
	}
	Invalidate(EInvalidateWidgetReason::LayoutAndVolatility); // UI 업데이트
}

FText SPathfindingTuner::GetSelectedAgentName() const
{
	if (LastSelectedActors.Num() == 0)
	{
		return LOCTEXT("NoAgentSelected", "No Agent or Volume Selected");
	}
	else if (LastSelectedActors.Num() > 1)
	{
		return FText::Format(
			LOCTEXT("MultipleActorsSelected", "Multiple Actors Selected ({0})"),
			FText::AsNumber(LastSelectedActors.Num()));
	}
	else // LastSelectedActors.Num() == 1
	{
		if (AActor* SelectedActor = LastSelectedActors[0].Get())
		{
			if (SelectedActor->FindComponentByClass<UPathfindingComponent>())
			{
				return FText::Format(
					LOCTEXT("SelectedAgentFormat", "Selected Agent: {0}"),
					FText::FromString(SelectedActor->GetActorLabel()));
			}
			else if (SelectedActor->IsA(AOctreeVoxelVolume::StaticClass()))
			{
				return FText::Format(
					LOCTEXT("SelectedVolumeFormat", "Selected Volume: {0}"),
					FText::FromString(SelectedActor->GetActorLabel()));
			}
			else
			{
				return FText::Format(
					LOCTEXT("SelectedActorGenericFormat", "Selected Actor: {0}"),
					FText::FromString(SelectedActor->GetActorLabel()));
			}
		}
		else
		{
			return LOCTEXT("InvalidActorSelected", "Invalid Actor Selected");
		}
	}
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

void SPathfindingTuner::OnNumThreadsChanged(int32 NewValue)
{
	// 값이 변경되는 동안에는 아무것도 하지 않거나, UI에만 임시 반영할 수 있습니다.
	// 실제 값 변경 및 저장은 OnNumThreadsCommitted에서 처리합니다.
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

// 현재 선택된 옥트리 볼륨을 가져오는 함수
TWeakObjectPtr<AOctreeVoxelVolume> SPathfindingTuner::GetSelectedOctreeVolume() const
{
	return CachedOctreeVolume;
}

// Target Octree Volume 필드의 ObjectPath 바인딩을 위한 함수
FString SPathfindingTuner::GetSelectedOctreeVolumePath() const
{
	if (AOctreeVoxelVolume* OctreeVolume = GetSelectedOctreeVolume().Get())
	{
		return OctreeVolume->GetPathName();
	}
	return FString();
}

// 액터 선택 변경 시 호출되는 함수
void SPathfindingTuner::OnActorSelectionChanged()
{
	// 1. CachedOctreeVolume 업데이트 (첫 번째 OctreeVoxelVolume만 캐시)
	CachedOctreeVolume = nullptr;
	for (TWeakObjectPtr<AActor> ActorPtr : LastSelectedActors)
	{
		if (AActor* Actor = ActorPtr.Get())
		{
			if (Actor->IsA(AOctreeVoxelVolume::StaticClass()))
			{
				CachedOctreeVolume = Cast<AOctreeVoxelVolume>(Actor);
				break;
			}
		}
	}

	// 2. CachedPathfindingComponent 업데이트 (첫 번째 PathfindingComponent를 가진 액터의 컴포넌트만 캐시)
	CachedPathfindingComponent = nullptr;
	for (TWeakObjectPtr<AActor> ActorPtr : LastSelectedActors)
	{
		if (AActor* Actor = ActorPtr.Get())
		{
			UPathfindingComponent* FoundComponent = Actor->FindComponentByClass<UPathfindingComponent>();
			if (FoundComponent)
			{
				CachedPathfindingComponent = FoundComponent;
				break;
			}
		}
	}

	// 3. 다중 선택 속성 값 집계 및 캐싱 (옵션 3 핵심 로직)
	CachedAgentRadiusValue.Reset();
	CachedMovementSpeedValue.Reset();
	CachedTargetActorValue = nullptr;
	bIsAgentRadiusMixed = false;
	bIsMovementSpeedMixed = false;
	bIsTargetActorMixed = false;

	if (LastSelectedActors.Num() > 0)
	{
		bool bFirstAgentFound = false;
		float FirstRadius = 0.0f;
		float FirstSpeed = 0.0f;
		AActor* FirstTargetActor = nullptr;

		for (TWeakObjectPtr<AActor> ActorPtr : LastSelectedActors)
		{
			if (AActor* Actor = ActorPtr.Get())
			{
				if (UPathfindingComponent* Comp = Actor->FindComponentByClass<UPathfindingComponent>())
				{
					if (!bFirstAgentFound)
					{
						FirstRadius = Comp->AgentRadius;
						FirstSpeed = Comp->MovementSpeed;
						FirstTargetActor = Comp->TargetActor.Get();
						bFirstAgentFound = true;
					}
					else
					{
						if (!FMath::IsNearlyEqual(Comp->AgentRadius, FirstRadius))
						{
							bIsAgentRadiusMixed = true;
						}
						if (!FMath::IsNearlyEqual(Comp->MovementSpeed, FirstSpeed))
						{
							bIsMovementSpeedMixed = true;
						}
						if (Comp->TargetActor.Get() != FirstTargetActor)
						{
							bIsTargetActorMixed = true;
						}
						if (bIsAgentRadiusMixed && bIsMovementSpeedMixed && bIsTargetActorMixed)
						{
							break;
						}
					}
				}
			}
		}

		if (bFirstAgentFound)
		{
			if (!bIsAgentRadiusMixed) CachedAgentRadiusValue = FirstRadius;
			if (!bIsMovementSpeedMixed) CachedMovementSpeedValue = FirstSpeed;
			if (!bIsTargetActorMixed) CachedTargetActorValue = FirstTargetActor;
		}
	}

	// 4. UI 업데이트 요청
	Invalidate(EInvalidateWidgetReason::LayoutAndVolatility);
}

// MaxOctreeDepth
TOptional<int32> SPathfindingTuner::GetMaxOctreeDepth() const
{
	if (AOctreeVoxelVolume* OctreeVolume = GetSelectedOctreeVolume().Get())
	{
		return OctreeVolume->GetMaxOctreeDepth(); // AOctreeVoxelVolume의 GetMaxOctreeDepth() 함수를 호출한다고 가정
	}
	return TOptional<int32>(); // 유효한 값을 반환할 수 없을 때
}

void SPathfindingTuner::OnMaxOctreeDepthCommitted(int32 NewValue, ETextCommit::Type CommitType)
{
	if (CachedOctreeVolume.IsValid())
	{
		if (GEditor)
		{
			GEditor->BeginTransaction(LOCTEXT("SetMaxOctreeDepth", "Set Max Octree Depth"));
			CachedOctreeVolume->Modify();
			CachedOctreeVolume->SetMaxOctreeDepth(NewValue); // AOctreeVoxelVolume의 SetMaxOctreeDepth 함수를 호출한다고 가정
			GEditor->EndTransaction();
			CachedOctreeVolume->MarkPackageDirty();
			UE_LOG(LogTemp, Warning, TEXT("OctreeVoxelVolume '%s' MaxOctreeDepth committed to: %d"),
			       *CachedOctreeVolume->GetName(), NewValue);
		}
		else
		{
			UE_LOG(LogTemp, Error, TEXT("OnMaxOctreeDepthCommitted: GEditor is not valid."));
		}
	}
	else
	{
		ShowNoOctreeSelectedWarning();
		UE_LOG(LogTemp, Warning,
		       TEXT("OnMaxOctreeDepthCommitted: No OctreeVoxelVolume selected or cached. Cannot set MaxOctreeDepth."));
	}
}

// VoxelSize
TOptional<float> SPathfindingTuner::GetVoxelSize() const
{
	if (AOctreeVoxelVolume* OctreeVolume = GetSelectedOctreeVolume().Get())
	{
		return OctreeVolume->VoxelExtent;
	}
	return TOptional<float>();
}

void SPathfindingTuner::OnVoxelSizeCommitted(float NewValue, ETextCommit::Type CommitType)
{
	if (AOctreeVoxelVolume* OctreeVolume = GetSelectedOctreeVolume().Get())
	{
		GEditor->BeginTransaction(LOCTEXT("SetVoxelSize", "Set Voxel Size"));
		OctreeVolume->Modify();
		OctreeVolume->VoxelExtent = NewValue;
		GEditor->EndTransaction();
		OctreeVolume->MarkPackageDirty();
	}
	else
	{
		ShowNoOctreeSelectedWarning();
	}
}

// StaticObjectFilter
void SPathfindingTuner::OnStaticObjectFilterChanged(const FAssetData& AssetData)
{
	if (AOctreeVoxelVolume* OctreeVolume = GetSelectedOctreeVolume().Get())
	{
		GEditor->BeginTransaction(LOCTEXT("SetStaticObjectFilter", "Set Static Object Filter"));
		OctreeVolume->Modify();
		OctreeVolume->SetStaticObstacleFilter(Cast<UVoxelObstacleFilter>(AssetData.GetAsset())); // 실제 필터 클래스 타입으로 변경
		GEditor->EndTransaction();
		OctreeVolume->MarkPackageDirty();
	}
}

FString SPathfindingTuner::GetStaticObjectFilterPath() const
{
	if (AOctreeVoxelVolume* OctreeVolume = GetSelectedOctreeVolume().Get())
	{
		if (UVoxelObstacleFilter* Filter = OctreeVolume->GetStaticObstacleFilter())
		// AOctreeVoxelVolume의 GetStaticObstacleFilter() 함수를 호출한다고 가정
		{
			return Filter->GetPathName();
		}
	}
	return FString();
}

// DynamicObjectFilter (제거됨)

// Bake 버튼 클릭 핸들러
FReply SPathfindingTuner::OnBakeButtonClicked()
{
	AOctreeVoxelVolume* OctreeVolume = GetSelectedOctreeVolume().Get();
	if (!OctreeVolume)
	{
		ShowNoOctreeSelectedWarning();
		UE_LOG(LogTemp, Warning, TEXT("No OctreeVoxelVolume selected or found to bake."));
		return FReply::Handled();
	}

	// --- [추가된 예외 처리 로직 시작] ---
	// 베이크 데이터 저장 경로가 설정되었는지 확인합니다.
	if (OctreeVolume->BakedDataSaveDirectory.Path.IsEmpty())
	{
		// 경로가 비어있으면 사용자에게 경고 메시지를 띄웁니다.
		FMessageDialog::Open(EAppMsgType::Ok,
		                     LOCTEXT("BakePathNotSetWarning",
		                             "Cannot start baking.\n\nPlease set the 'Baked Data Save Directory' in the Octree Volume Settings first."));

		UE_LOG(LogTemp, Error, TEXT("Baking failed: 'Baked Data Save Directory' is not set for %s."),
		       *OctreeVolume->GetName());

		// 베이크 프로세스를 중단합니다.
		return FReply::Handled();
	}

	if (!OctreeVolume->GetStaticObstacleFilter())
	{
		EAppReturnType::Type UserResponse = EAppReturnType::Cancel;

		TSharedRef<SWindow> ConfirmationWindow = SNew(SWindow)
			.Title(LOCTEXT("BakingConfirmationTitle", "Baking Confirmation"))
			.SizingRule(ESizingRule::Autosized)
			.SupportsMaximize(false)
			.SupportsMinimize(false);

		TSharedPtr<SButton> YesButton;

		// --- [UI 개선 부분 시작] ---
		ConfirmationWindow->SetContent(
			SNew(SBorder)
			             .BorderImage(FAppStyle::GetBrush("Menu.Background")) // 창 배경 스타일
			             .Padding(FMargin(10.0f))
			[
				SNew(SHorizontalBox)

				// 1. 경고 아이콘 추가
				+ SHorizontalBox::Slot()
				.AutoWidth()
				.VAlign(VAlign_Top)
				.Padding(0, 0, 12, 0)
				[
					SNew(SImage)
					.Image(FAppStyle::GetBrush("MessageLog.Warning")) // 표준 경고 아이콘
				]

				// 2. 텍스트와 버튼을 담을 수직 박스
				+ SHorizontalBox::Slot()
				.FillWidth(1.0f)
				[
					SNew(SVerticalBox)

					// 메시지 텍스트
					+ SVerticalBox::Slot()
					.AutoHeight()
					.Padding(0, 0, 0, 12)
					[
						SNew(STextBlock)
						.Text(LOCTEXT("FilterNotSetMessage",
						              "The 'Static Object Filter' is not set.\n\nThe system will fall back to the default behavior: baking all 'WorldStatic' objects as permanent obstacles.\n\nIs this okay?"))
						.Font(FAppStyle::GetFontStyle("StandardDialog.LargeFont"))
						.AutoWrapText(true) // 텍스트가 길어지면 자동 줄바꿈
					]

					// 버튼 영역
					+ SVerticalBox::Slot()
					.AutoHeight()
					.HAlign(HAlign_Right)
					[
						SNew(SUniformGridPanel)
						.SlotPadding(FAppStyle::GetMargin("StandardDialog.SlotPadding"))
						+ SUniformGridPanel::Slot(0, 0)
						[
							SAssignNew(YesButton, SButton)
							.HAlign(HAlign_Center)
							.ContentPadding(FAppStyle::GetMargin("StandardDialog.ContentPadding"))
							.Text(LOCTEXT("Yes", "Yes"))
							.OnClicked_Lambda([&]()
							{
								UserResponse = EAppReturnType::Yes;
								ConfirmationWindow->RequestDestroyWindow();
								return FReply::Handled();
							})
						]
						+ SUniformGridPanel::Slot(1, 0)
						[
							SNew(SButton)
							.HAlign(HAlign_Center)
							.ContentPadding(FAppStyle::GetMargin("StandardDialog.ContentPadding"))
							.Text(LOCTEXT("No", "No"))
							.OnClicked_Lambda([&]()
							{
								UserResponse = EAppReturnType::No;
								ConfirmationWindow->RequestDestroyWindow();
								return FReply::Handled();
							})
						]
					]
				]
			]
		);
		// --- [UI 개선 부분 끝] ---

		ConfirmationWindow->SetWidgetToFocusOnActivate(YesButton);
		FSlateApplication::Get().AddModalWindow(ConfirmationWindow, FSlateApplication::Get().GetActiveTopLevelWindow());

		if (UserResponse != EAppReturnType::Yes)
		{
			UE_LOG(LogTemp, Warning,
			       TEXT("Baking cancelled by user. Please set a 'Static Object Filter' to define what to bake."));
			return FReply::Handled();
		}
	}

	// 경로가 유효하므로 트랜잭션을 시작하고 베이크를 진행합니다.
	GEditor->BeginTransaction(LOCTEXT("BakeOctreeData", "Bake Octree Data"));
	OctreeVolume->Modify();

	OctreeVolume->BakeStaticData();

	GEditor->EndTransaction();
	// OctreeVolume->MarkPackageDirty(); // BakeStaticData 내부에서 Save가 호출된다면 이 줄은 필요 없을 수 있습니다.
	// Save 로직이 패키지를 dirty로 만들기 때문입니다.

	UE_LOG(LogTemp, Log, TEXT("Bake button clicked for %s"), *OctreeVolume->GetName());

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
