// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Widgets/SCompoundWidget.h"

class UPathfindingSubsystem;
class UAvoidanceSubsystem;
class UPathfindingTunerSettings;
class AActor;
struct FAssetData; 

class SPathfindingTuner : public SCompoundWidget
{
public:
    SLATE_BEGIN_ARGS(SPathfindingTuner) {}
    SLATE_END_ARGS()

     
    void Construct(const FArguments& InArgs);

    virtual ~SPathfindingTuner() override;

private:
    //=================================================================================================
    // UI Section Builders
    //=================================================================================================
    TSharedRef<SWidget> BuildPerformanceSettingsSection();
    TSharedRef<SWidget> BuildOctreeVolumeSettingsSection();
    TSharedRef<SWidget> BuildBakedDataSettingsSection();
    TSharedRef<SWidget> BuildAgentSettingsSection();
    TSharedRef<SWidget> BuildVoxelDebugSettingsSection();
    TSharedRef<SWidget> BuildAgentDebugSection();
    //=================================================================================================
    // Actor Selection Handling
    //=================================================================================================
 
    bool TickActorSelection(float DeltaTime);
    void OnActorSelectionChanged();

    //=================================================================================================
    // Property Getters & Setters (Generic Helpers)
    //=================================================================================================
    template <typename ObjectType, typename PropertyType>
    TOptional<PropertyType> GetCommonPropertyValue(const TArray<ObjectType*>& Objects, TFunctionRef<PropertyType(const ObjectType*)> Getter) const;
    
    template <typename ObjectType, typename PropertyType>
    void SetCommonPropertyValue(const TArray<ObjectType*>& Objects, const PropertyType& NewValue, const FText& TransactionText, TFunctionRef<void(ObjectType*, const PropertyType&)> Setter);
    
    template <typename T>
    TArray<T*> GetSelectedActorsOfType() const;
    
    template <typename T>
    TArray<T*> GetSelectedComponentsOfType() const;

    //=================================================================================================
    // UI State & Expansion Handlers
    //=================================================================================================
    void OnPerformanceCategoryExpansionChanged(bool bIsExpanded);
    void OnOctreeSettingsCategoryExpansionChanged(bool bIsExpanded);
    void OnBakedDataCategoryExpansionChanged(bool bIsExpanded);
    void OnAgentSettingsCategoryExpansionChanged(bool bIsExpanded);
    void OnDebugSettingsCategoryExpansionChanged(bool bIsExpanded);
    void OnAgentDebugCategoryExpansionChanged(bool bIsExpanded);
   
    //=================================================================================================
    // UI Callbacks & Value Accessors
    //=================================================================================================

    // --- Performance Settings ---
    TOptional<int32> GetCurrentNumThreads() const;
    void OnNumThreadsCommitted(int32 NewValue, ETextCommit::Type CommitInfo);

    // --- Octree Volume Settings ---
    bool AreOnlyOctreeVolumesSelected() const;
    FText GetSelectedVolumeDisplayText() const;
    FText GetBakedDataSaveDirectoryAsText() const;
    void OnBakedDataSaveDirectoryTextCommitted(const FText& InText, ETextCommit::Type CommitInfo);

    FString GetObstacleProfilePath() const;
    void OnObstacleProfileChanged(const FAssetData& AssetData);
    FText GetObstacleProfileFullPathAsText() const;

    FReply OnBrowseButtonClicked();
    TOptional<int32> GetMaxOctreeDepth() const;
    void OnMaxOctreeDepthCommitted(int32 NewValue, ETextCommit::Type CommitType);
    
    TOptional<float> GetVoxelExtent() const;
    void OnVoxelExtentCommitted(float NewValue, ETextCommit::Type CommitType);
    
    FText GetSelectedMapSizeText() const;
    void OnMapSizeSelectionChanged(TSharedPtr<int32> NewSelection, ESelectInfo::Type SelectType);

    // --- Baked Data Settings ---
    FString GetBakedOctreeDataPath() const;
    void OnBakedOctreeDataChanged(const FAssetData& AssetData);
    FText GetBakedDataFullPathAsText() const;

    // --- Debug Settings ---
    TOptional<ECheckBoxState> GetDrawDebugBoxState() const;
    void OnDrawDebugBoxChanged(ECheckBoxState NewState);
    TOptional<float> GetDebugLineWidth() const;
    void OnDebugLineWidthCommitted(float NewValue, ETextCommit::Type CommitType);
   TOptional<ECheckBoxState> GetDrawDebugPathState() const;
    void OnDrawDebugPathChanged(ECheckBoxState NewState);
    TOptional<bool> GetbUseAgentAvoidance() const;
    void OnbAvoidanceComitted(ECheckBoxState NewState);

    // --- Agent Settings ---
    bool AreOnlyAgentsSelected() const;
    FText GetSelectedAgentName() const;
    
    FString GetTargetActorPath() const;
    void OnTargetActorChanged(const FAssetData& AssetData);
    
    FString GetTargetVolumePath() const;
    void OnTargetVolumeChanged(const FAssetData& AssetData);
    
    TOptional<float> GetAgentRadius() const;
    void OnAgentRadiusCommitted(float NewValue, ETextCommit::Type CommitType);
    
    TOptional<float> GetMovementSpeed() const;
    void OnMovementSpeedCommitted(float NewValue, ETextCommit::Type CommitType);

    TOptional<float> GetAvoidanceRadius() const;
    void OnAvoidanceRadiusCommitted(float NewValue, ETextCommit::Type CommitType);
    
    // --- Actions ---
    FReply OnBakeButtonClicked();
    void ShowWarningMessage(const FText& Message) const;
    FReply OnRequestPathButtonClicked();
    bool IsPIEActive() const;
private:
    //=================================================================================================
    // Member Variables
    //=================================================================================================
  
    TWeakObjectPtr<UPathfindingSubsystem> PathfindingSubsystem;
    TWeakObjectPtr<UAvoidanceSubsystem> AvoidanceSubsystem;
    
    FTSTicker::FDelegateHandle ActorSelectionTickerHandle;
    
    TArray<TWeakObjectPtr<AActor>> LastSelectedActors;
    
    TArray<TSharedPtr<int32>> MapSizeOptions;
    
    TObjectPtr<UPathfindingTunerSettings> TunerSettings;
};
 
//=================================================================================================
// Template Member Function Implementations (Must be in header)
//=================================================================================================

template <typename ObjectType, typename PropertyType>
TOptional<PropertyType> SPathfindingTuner::GetCommonPropertyValue(const TArray<ObjectType*>& Objects, TFunctionRef<PropertyType(const ObjectType*)> Getter) const
{
    if (Objects.IsEmpty())
    {
        return TOptional<PropertyType>();
    }

    const PropertyType FirstValue = Getter(Objects[0]);
    for (int32 i = 1; i < Objects.Num(); ++i)
    {
        if (Getter(Objects[i]) != FirstValue)
        {
            return TOptional<PropertyType>(); // Multiple values
        }
    }
    return FirstValue;
}

template <typename ObjectType, typename PropertyType>
void SPathfindingTuner::SetCommonPropertyValue(const TArray<ObjectType*>& Objects, const PropertyType& NewValue, const FText& TransactionText, TFunctionRef<void(ObjectType*, const PropertyType&)> Setter)
{
    if (Objects.IsEmpty()) return;

    const FScopedTransaction Transaction(TransactionText);
    bool bModified = false;

    for (ObjectType* Obj : Objects)
    {
        if (Obj)
        {
            Obj->Modify();
            Setter(Obj, NewValue);
            Obj->MarkPackageDirty();  
            bModified = true;
        }
    }

    if (bModified)
    {
        Invalidate(EInvalidateWidgetReason::LayoutAndVolatility);
    }
}

template <typename T>
TArray<T*> SPathfindingTuner::GetSelectedActorsOfType() const
{
    TArray<T*> SelectedTypedActors;
    for (const TWeakObjectPtr<AActor>& ActorPtr : LastSelectedActors)
    {
        if (T* TypedActor = Cast<T>(ActorPtr.Get()))
        {
            SelectedTypedActors.Add(TypedActor);
        }
    }
    return SelectedTypedActors;
}

template <typename T>
TArray<T*> SPathfindingTuner::GetSelectedComponentsOfType() const
{
    TArray<T*> SelectedComponents;
    for (const TWeakObjectPtr<AActor>& ActorPtr : LastSelectedActors)
    {
        if (AActor* Actor = ActorPtr.Get())
        {
            if (T* Component = Actor->FindComponentByClass<T>())
            {
                SelectedComponents.Add(Component);
            }
        }
    }
    return SelectedComponents;
}