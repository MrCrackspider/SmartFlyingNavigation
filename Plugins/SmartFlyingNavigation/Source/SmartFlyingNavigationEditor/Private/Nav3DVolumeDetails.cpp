// Copyright © 2025 TeamHamDev. All Rights Reserved.

#include "Nav3DVolumeDetails.h"
#include "DetailLayoutBuilder.h"
#include "DetailWidgetRow.h"
#include "DetailCategoryBuilder.h"
#include "PropertyEditorModule.h"
#include "NavVolume/OctreeVoxelVolume.h"
#include "PathfindingTunerTabName.h"
#include "Subsystem/OctreeNavSubsystem.h" 
#include "Editor.h"
#include "PathfindingEditorSubsystem.h"

TSharedRef<IDetailCustomization> FNav3DVolumeDetails::MakeInstance()
{
	return MakeShareable(new FNav3DVolumeDetails);
}

void FNav3DVolumeDetails::CustomizeDetails(IDetailLayoutBuilder& DetailBuilder)
{
    // --- 1. 기존 프로퍼티들을 카테고리에 정리 (이 부분은 그대로 둡니다) ---
    IDetailCategoryBuilder& VolumeCategory = DetailBuilder.EditCategory("Volume Setup", FText::GetEmpty(), ECategoryPriority::Important);
    VolumeCategory.AddProperty(GET_MEMBER_NAME_CHECKED(AVoxelVolumeBase, VoxelExtent));
    VolumeCategory.AddProperty(GET_MEMBER_NAME_CHECKED(AOctreeVoxelVolume, BakedDataSaveDirectory));

    IDetailCategoryBuilder& DebuggingCategory = DetailBuilder.EditCategory("Debugging", FText::GetEmpty(), ECategoryPriority::Important);
    DebuggingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(AOctreeVoxelVolume, bDrawDebugBox));

    // --- 2. "Bake Actions" 카테고리와 새로운 베이크 버튼 추가 ---
    IDetailCategoryBuilder& BakeCategory = DetailBuilder.EditCategory("Bake Actions", FText::GetEmpty(), ECategoryPriority::Important);

    // 디테일 패널에서 현재 선택된 오브젝트들의 목록을 가져옵니다.
    TArray<TWeakObjectPtr<UObject>> SelectedObjects;
    DetailBuilder.GetObjectsBeingCustomized(SelectedObjects);

    BakeCategory.AddCustomRow(LOCTEXT("BakeButtonFilter", "Bake Volume"))
        .NameContent()
        [
            SNew(STextBlock)
                .Text(LOCTEXT("BakeActionLabel", "Bake Actions"))
                .Font(IDetailLayoutBuilder::GetDetailFont())
        ]
        .ValueContent()
        [
            SNew(SButton)
                .Text(LOCTEXT("BakeSelectedButton", "Bake Selected Volume(s)"))
                .ToolTipText(LOCTEXT("BakeSelectedButton_Tooltip", "Bakes the navigation data for the currently selected volume(s), including portal connections between them."))
                .HAlign(HAlign_Center)
                .OnClicked_Lambda([SelectedObjects]() -> FReply
                    {
                        TArray<AOctreeVoxelVolume*> VolumesToBake;
                        for (const TWeakObjectPtr<UObject>& ObjPtr : SelectedObjects)
                        {
                            if (AOctreeVoxelVolume* Volume = Cast<AOctreeVoxelVolume>(ObjPtr.Get()))
                            {
                                VolumesToBake.Add(Volume);
                            }
                        }

                        if (VolumesToBake.IsEmpty())
                        {
                            UE_LOG(LogTemp, Warning, TEXT("No AOctreeVoxelVolume selected to bake."));
                            return FReply::Handled();
                        }

                        // <<<< 여기가 핵심적인 수정점 >>>>
                        // 런타임 서브시스템 대신 에디터 서브시스템을 가져옵니다.
                        if (UPathfindingEditorSubsystem* EditorSubsystem = GEditor->GetEditorSubsystem<UPathfindingEditorSubsystem>())
                        {
                            // 에디터 서브시스템의 베이크 함수를 호출합니다.
                            EditorSubsystem->BakeSpecificVolumes(VolumesToBake);
                        }
                        else
                        {
                            UE_LOG(LogTemp, Error, TEXT("PathfindingEditorSubsystem not found! Cannot start bake."));
                        }
                        // <<<< 수정 끝 >>>>

                        return FReply::Handled();
                    })
        ];

    // --- 3. 기존의 "Pathfinding Tuner UI" 버튼은 그대로 유지합니다 ---
    IDetailCategoryBuilder& TunerCategory = DetailBuilder.EditCategory("Tuner", FText::GetEmpty(), ECategoryPriority::Important);
    TunerCategory.AddCustomRow(LOCTEXT("OpenPathfindingTunerFilter", "Open Pathfinding Tuner"))
        .NameContent()
        [
            SNew(STextBlock)
                .Text(LOCTEXT("OpenPathfindingTunerLabel", "Pathfinding Tuner UI"))
                .Font(IDetailLayoutBuilder::GetDetailFont())
        ]
        .ValueContent()
        [
            SNew(SButton)
                .Text(LOCTEXT("OpenPathfindingTunerButton", "Open Tuner"))
                .OnClicked_Lambda([]() -> FReply
                    {
                        FGlobalTabmanager::Get()->TryInvokeTab(PathfindingTunerTabName);
                        return FReply::Handled();
                    })
        ];
}