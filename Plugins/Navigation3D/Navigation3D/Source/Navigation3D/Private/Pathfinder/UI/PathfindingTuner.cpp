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

// AOctreeVoxelVolume 클래스 헤더 (당신의 프로젝트 경로에 맞게 수정)
#include "Pathfinder/NavData/VoxelObstacleFilter.h"
#include "PropertyCustomizationHelpers.h"
#include "Selection.h"
#include "Pathfinder/NavVolume/OctreeVoxelVolume.h"
// 필터 데이터 에셋 클래스 헤더 (당신의 실제 클래스 이름으로 변경)

#define LOCTEXT_NAMESPACE "PathfindingTuner" 

void SPathfindingTuner::Construct(const FArguments& InArgs)
{
   PathfindingSubsystem = GEngine->GetEngineSubsystem<UPathfindingSubsystem>();
   
   if (GEditor)
   {
      // 위젯 초기화 시 선택된 액터 초기화
      if (USelection* Selection = GEditor->GetSelectedActors())
      {
         if (AActor* FirstActor = Cast<AActor>(Selection->GetSelectedObject(0)))
         {
            LastSelectedActor = FirstActor;
            OnActorSelectionChanged(FirstActor);
         }
      }

      // 선택된 액터를 감시하는 Ticker 등록
      ActorSelectionTickerHandle = FTSTicker::GetCoreTicker().AddTicker(
         FTickerDelegate::CreateLambda([this](float DeltaTime)
         {
            if (!GEditor) return true;

            USelection* SelectedActors = GEditor->GetSelectedActors();
            AActor* FirstSelected = (SelectedActors && SelectedActors->Num() > 0)
               ? Cast<AActor>(SelectedActors->GetSelectedObject(0))
               : nullptr;

            if (FirstSelected != LastSelectedActor)
            {
               LastSelectedActor = FirstSelected;
               OnActorSelectionChanged(FirstSelected);
            }

            return true; // 계속 호출
         })
      );
   }

   // '성능 설정' 카테고리의 초기 확장 상태
   const bool bInitiallyCollapsedPerformance = !bIsPerformanceCategoryExpanded;
   // '옥트리 볼륨 설정' 카테고리의 초기 확장 상태
   const bool bInitiallyCollapsedOctreeSettings = !bIsOctreeSettingsCategoryExpanded;

   ChildSlot
      [
         SNew(SVerticalBox)

            // --- 1. '성능 설정' 카테고리 (기존) ---
            + SVerticalBox::Slot()
            .AutoHeight()
            .Padding(2)
            [
               SNew(SExpandableArea)
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
                                    .OnValueChanged(this, &SPathfindingTuner::OnNumThreadsChanged) // 값 변경 중
                                    .OnValueCommitted(this, &SPathfindingTuner::OnNumThreadsCommitted) // 값 확정 시
                                     // 값 확정 시
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
               .OnAreaExpansionChanged_Raw(this, &SPathfindingTuner::OnPerformanceCategoryExpansionChanged)
            ]

            // --- 2. '옥트리 볼륨 설정' 카테고리 (새로 추가) ---
            + SVerticalBox::Slot()
            .AutoHeight()
            .Padding(2)
            [
               SNew(SExpandableArea)
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
                              .OnValueChanged(this, &SPathfindingTuner::OnMaxOctreeDepthChanged)
                              .OnValueCommitted(this, &SPathfindingTuner::OnMaxOctreeDepthCommitted)
                              .MinValue(0)
                              .MaxValue(10)
                              .MinSliderValue(0)
                              .MaxSliderValue(8) // 슬라이더 범위 조정
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
                              .OnValueChanged(this, &SPathfindingTuner::OnVoxelSizeChanged)
                              .OnValueCommitted(this, &SPathfindingTuner::OnVoxelSizeCommitted)
                              .MinValue(1.0f)
                              .MaxValue(1000.0f)
                              .MinSliderValue(10.0f)
                              .MaxSliderValue(500.0f) // 슬라이더 범위 조정
                              .Delta(1.0f)
                           ]
                        ]
                        
                        // StaticObjectFilter 설정
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
                              .Text(LOCTEXT("StaticObjectFilterLabel", "Static Object Filter:"))
                              .Font(FAppStyle::GetFontStyle("PropertyWindow.NormalFont"))
                           ]
                           + SHorizontalBox::Slot()
                           .FillWidth(0.6f)
                           [
                              SNew(SObjectPropertyEntryBox)
                              .AllowedClass(AOctreeVoxelVolume::StaticClass())
                              .OnObjectChanged(this, &SPathfindingTuner::OnStaticObjectFilterChanged)
                              .ObjectPath(this, &SPathfindingTuner::GetStaticObjectFilterPath)
                           ]
                        ]

                        // DynamicObjectFilter 설정 (제거됨)
                        // 이 부분은 이제 UI에서 완전히 제거됩니다.

                        // Bake 버튼
                        + SVerticalBox::Slot()
                        .AutoHeight()
                        .Padding(8.0f, 15.0f)
                        [
                           SNew(SButton)
                           .Text(LOCTEXT("BakeButtonText", "Bake Octree Data"))
                           .OnClicked(this, &SPathfindingTuner::OnBakeButtonClicked)
                        ]
                  ]
               .InitiallyCollapsed(bInitiallyCollapsedOctreeSettings)
               .OnAreaExpansionChanged_Raw(this, &SPathfindingTuner::OnOctreeSettingsCategoryExpansionChanged)
            ]
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
   
}

void SPathfindingTuner::OnNumThreadsCommitted(int32 NewValue,
   ETextCommit::Type CommitInfo)
{
   if (PathfindingSubsystem.IsValid())
   {
      // 실제 스레드 개수 변경
      PathfindingSubsystem->SetNumWorkerThreads(NewValue);
      // 변경 사항 저장 (이 시점에 한 번만 호출)
      PathfindingSubsystem->SaveConfig();

      UE_LOG(LogTemp, Warning, TEXT("PathfindingTuner: Num Threads Committed to: %d. Config Saved."), NewValue);

      // UI를 강제로 업데이트하여 바인딩된 값이 새로고침되도록 합니다.
      // 이 Invalidate 호출은 SNumericEntryBox의 Value 바인딩이 GetCurrentNumThreads()를 다시 호출하도록 유도합니다.
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


// Target Octree Volume 필드의 ObjectPath 바인딩을 위한 함수
FString SPathfindingTuner::GetSelectedOctreeVolumePath() const
{
   if (AOctreeVoxelVolume* OctreeVolume = GetSelectedOctreeVolume().Get())
   {
      return OctreeVolume->GetPathName();
   }
   return FString();
}

void SPathfindingTuner::OnActorSelectionChanged(AActor* SelectedActor)
{
   if (SelectedActor && SelectedActor->IsA(AOctreeVoxelVolume::StaticClass()))
   {
      CachedOctreeVolume = Cast<AOctreeVoxelVolume>(SelectedActor);
   }
   else
   {
      CachedOctreeVolume = nullptr;
   }

   Invalidate(EInvalidateWidgetReason::LayoutAndVolatility);
}


TWeakObjectPtr<AOctreeVoxelVolume> SPathfindingTuner::
GetSelectedOctreeVolume() const
{
   return CachedOctreeVolume;
}

// MaxOctreeDepth
TOptional<int32> SPathfindingTuner::GetMaxOctreeDepth() const
{
   if (AOctreeVoxelVolume* OctreeVolume = GetSelectedOctreeVolume().Get())
   {
      return OctreeVolume->GetMaxOctreeDepth();
   }
   return TOptional<int32>(); // 유효한 값을 반환할 수 없을 때
}

void SPathfindingTuner::OnMaxOctreeDepthChanged(int32 NewValue)
{
   if (AOctreeVoxelVolume* OctreeVolume = GetSelectedOctreeVolume().Get())
   {
   //   OctreeVolume->MaxOctreeDepth = NewValue;
   }
}

void SPathfindingTuner::OnMaxOctreeDepthCommitted(int32 NewValue, ETextCommit::Type CommitType)
{
   // CachedOctreeVolume이 유효한지 확인합니다.
   if (CachedOctreeVolume.IsValid())
   {
      // GEditor가 유효한지 확인합니다.
      if (GEditor)
      {
         // 언두/리두를 위한 트랜잭션 시작
         GEditor->BeginTransaction(LOCTEXT("SetMaxOctreeDepth", "Set Max Octree Depth"));
            
         // 액터 수정 가능 상태로 만듭니다.
         CachedOctreeVolume->Modify();

         // AOctreeVoxelVolume 클래스에 MaxOctreeDepth를 설정하는 함수가 있다고 가정합니다.
         // 만약 직접 멤버 변수라면, CachedOctreeVolume->MaxOctreeDepth = NewValue; 와 같이 접근합니다.
         // 여기서는 SetMaxOctreeDepth라는 함수가 있다고 가정하고 호출합니다.
         // (실제 AOctreeVoxelVolume 클래스에 맞게 수정해주세요)
         // 예시: CachedOctreeVolume->SetMaxOctreeDepth(NewValue); 
         // 또는: CachedOctreeVolume->MaxOctreeDepth = NewValue;

         // 예시: MaxOctreeDepth가 UPROPERTY로 선언된 멤버 변수라고 가정
         CachedOctreeVolume->SetMaxOctreeDepth(NewValue); 
            
         // 트랜잭션 종료
         GEditor->EndTransaction();

         // 변경 사항을 저장합니다. (UObject의 속성 변경 후에는 SaveConfig()가 필요할 수 있습니다.)
         // AOctreeVoxelVolume이 UActor이므로, SaveConfig() 대신 MarkPackageDirty()를 사용할 수 있습니다.
         // CachedOctreeVolume->GetOuter()->MarkPackageDirty(); 
         // 또는, Persistent Level에 저장되는 액터라면 에디터 저장 시 반영됩니다.
            
         UE_LOG(LogTemp, Warning, TEXT("OctreeVoxelVolume '%s' MaxOctreeDepth committed to: %d"), *CachedOctreeVolume->GetName(), NewValue);
      }
      else
      {
         UE_LOG(LogTemp, Error, TEXT("OnMaxOctreeDepthCommitted: GEditor is not valid."));
      }
   }
   else
   {
      UE_LOG(LogTemp, Warning, TEXT("OnMaxOctreeDepthCommitted: No OctreeVoxelVolume selected or cached. Cannot set MaxOctreeDepth."));
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

void SPathfindingTuner::OnVoxelSizeChanged(float NewValue)
{
   if (AOctreeVoxelVolume* OctreeVolume = GetSelectedOctreeVolume().Get())
   {
      OctreeVolume->VoxelExtent = NewValue;
   }
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
      if (OctreeVolume->GetStaticObstacleFilter())
      {
         return OctreeVolume->GetStaticObstacleFilter()->GetPathName();
      }
   }
   return FString();
}

// DynamicObjectFilter
void SPathfindingTuner::OnDynamicObjectFilterChanged(const FAssetData& AssetData)
{
   if (AOctreeVoxelVolume* OctreeVolume = GetSelectedOctreeVolume().Get())
   {
   //   GEditor->BeginTransaction(LOCTEXT("SetDynamicObjectFilter", "Set Dynamic Object Filter"));
      OctreeVolume->Modify();
   //   OctreeVolume->DynamicObjectFilter = Cast<UDynamicObjectFilterDataAsset>(AssetData.GetAsset()); // 실제 필터 클래스 타입으로 변경
      GEditor->EndTransaction();
      OctreeVolume->MarkPackageDirty();
   }
}

// Bake 버튼 클릭 핸들러
FReply SPathfindingTuner::OnBakeButtonClicked()
{
   if (AOctreeVoxelVolume* OctreeVolume = GetSelectedOctreeVolume().Get())
   {
      GEditor->BeginTransaction(LOCTEXT("BakeOctreeData", "Bake Octree Data"));
      OctreeVolume->Modify(); 
        
      OctreeVolume->BakeStaticData(); 
        
      GEditor->EndTransaction(); 
      OctreeVolume->MarkPackageDirty(); 
        
      UE_LOG(LogTemp, Log, TEXT("Bake button clicked for %s"), *OctreeVolume->GetName());
   }
   else
   {
      UE_LOG(LogTemp, Warning, TEXT("No OctreeVoxelVolume selected or found to bake."));
   }
   return FReply::Handled();
}

void SPathfindingTuner::OnOctreeSettingsCategoryExpansionChanged(bool bIsExpanded)
{
   bIsOctreeSettingsCategoryExpanded = bIsExpanded;
}