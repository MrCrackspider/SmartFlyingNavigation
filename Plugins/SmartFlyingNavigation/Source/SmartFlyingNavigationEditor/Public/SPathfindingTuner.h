// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "IDetailsView.h"
#include "Subsystem/AvoidanceSubsystem.h"
#include "Widgets/SCompoundWidget.h"

class UPathfindingSubsystem;
class UAvoidanceSubsystem;
class UOctreeNavSubsystem;
class UPathfindingTunerSettings;
class UWorld;
class AActor;
class ULandscapeComponent;
class AOctreeVoxelVolume;
struct FAssetData;
struct FLoadedNavVolumeSummary;

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
    TSharedRef<SWidget> BuildVoxelDebugSettingsSection();
    TSharedRef<SWidget> BuildBakedDataSettingsSection();
    TSharedRef<SWidget> BuildAvoidanceSettingsSection();
    TSharedRef<SWidget> BuildWorldPartitionWorkflowSection();

   
    // TSharedRef<SWidget> BuildAgentSettingsSection();
    // TSharedRef<SWidget> BuildAgentDebugSection();
    //TSharedRef<SWidget> BuildWorldBakeSection();
    
    //=================================================================================================
    // Actor Selection Handling
    //=================================================================================================
    
    bool TickActorSelection(float DeltaTime);
    void OnActorSelectionChanged();

    TSharedPtr<IDetailsView> AgentDetailsView ;
    
     
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
    void OnAvoidanceCategoryExpansionChanged(bool bIsExpanded);
    void OnPerformanceCategoryExpansionChanged(bool bIsExpanded);
    void OnOctreeSettingsCategoryExpansionChanged(bool bIsExpanded);
    void OnBakedDataCategoryExpansionChanged(bool bIsExpanded);
    void OnWorldPartitionWorkflowExpansionChanged(bool bIsExpanded);
    // void OnAgentSettingsCategoryExpansionChanged(bool bIsExpanded);
    void OnDebugSettingsCategoryExpansionChanged(bool bIsExpanded);
    void OnAgentDebugCategoryExpansionChanged(bool bIsExpanded);
    // void OnWorldBakeCategoryExpansionChanged(bool bIsExpanded);
    //=================================================================================================
    // UI Callbacks & Value Accessors
    //=================================================================================================

    // --- Performance Settings ---
    TOptional<int32> GetCurrentNumThreads() const;
    void OnNumThreadsCommitted(int32 NewValue, ETextCommit::Type CommitInfo);

    // --- Octree Volume Settings ---
    bool AreOnlyOctreeVolumesSelected() const;
    FText GetSelectedVolumeDisplayText() const;

    FString GetObstacleProfilePath() const;
    void OnObstacleProfileChanged(const FAssetData& AssetData);
    FText GetObstacleProfileFullPathAsText() const;

    // TOptional<int32> GetMaxOctreeDepth() const;
    // void OnMaxOctreeDepthCommitted(int32 NewValue, ETextCommit::Type CommitType);
    
    TOptional<float> GetVoxelExtent() const;
    void OnVoxelExtentCommitted(float NewValue, ETextCommit::Type CommitType);
    
    FText GetSelectedMapSizeText() const;
    void OnMapSizeSelectionChanged(TSharedPtr<int32> NewSelection, ESelectInfo::Type SelectType);

    // --- Baked Data Settings ---
    /** True when any selected volume needs a (re-)bake according to the world manifest. */
    bool IsAnySelectedVolumeBakeStale(TArray<FText>* OutReasons = nullptr) const;

    // --- Debug Settings ---
    TOptional<ECheckBoxState> GetDrawDebugBoxState() const;
    void OnDrawDebugBoxChanged(ECheckBoxState NewState);
    TOptional<float> GetDebugLineWidth() const;
    void OnDebugLineWidthCommitted(float NewValue, ETextCommit::Type CommitType);
   TOptional<ECheckBoxState> GetDrawDebugPathState() const;
    void OnDrawDebugPathChanged(ECheckBoxState NewState);
  //  TOptional<ECheckBoxState> GetbPeriodicRepathState() const;
    //void OnbPeriodicRepathChanged(ECheckBoxState NewState);
    void OnbAvoidanceComitted(ECheckBoxState NewState);
    void OnbAvoidanceVelocityDebugComitted(ECheckBoxState NewState);

    void OnTimeHorizonCommitted(float NewValue, ETextCommit::Type CommitType);
    void OnMaxNeighborsCommitted(int32 NewValue, ETextCommit::Type CommitType);
    void OnGridCellSizeCommitted(float NewValue, ETextCommit::Type CommitType);
    void OnSymmetryBreakingRangeMultiplierCommitted(float NewValue, ETextCommit::Type CommitType);
    void OnSolverIterationsCommitted(int32 NewValue, ETextCommit::Type CommitType);
    void OnSideBiasMagnitudeCommitted(float NewValue, ETextCommit::Type CommitType);
    void OnInterpSpeedCommitted(float NewValue, ETextCommit::Type CommitType);

    /** Finds the live AvoidanceSubsystem in the PIE world, if PIE is active. Returns nullptr otherwise. */
    UAvoidanceSubsystem* GetPIEAvoidanceSubsystem() const;

    /** Applies a value to both the CDO (for persistence) and the live PIE instance (for immediate effect). */
    template<typename ValueType>
    void ApplyAvoidanceSetting(ValueType NewValue, void (UAvoidanceSubsystem::*Setter)(ValueType));

    /** Reads an avoidance value, preferring the PIE instance when active. */
    template<typename ValueType>
    TOptional<ValueType> ReadAvoidanceSetting(ValueType (UAvoidanceSubsystem::*Getter)() const) const;
    
    // --- Agent Settings ---
    bool AreOnlyAgentsSelected() const;
    // FText GetSelectedAgentName() const;
    //
    // FString GetTargetActorPath() const;
    // void OnTargetActorChanged(const FAssetData& AssetData);
    //     
    // TOptional<float> GetAgentRadius() const;
    // FText GetFinalAgentRadiusAsText() const;
    // void OnAgentRadiusCommitted(float NewValue, ETextCommit::Type CommitType);
    //
    // TOptional<float> GetMovementSpeed() const;
    // void OnMovementSpeedCommitted(float NewValue, ETextCommit::Type CommitType);
    //
    // TOptional<float> GetAvoidanceRadius() const;
    // void OnAvoidanceRadiusCommitted(float NewValue, ETextCommit::Type CommitType);
    // TOptional<float> GetRepathCooldown() const;
    // void OnRepathCooldownCommitted(float NewCooldownValue,
    //                                ETextCommit::Type CommitType);
    //
    // // --- World Bake Settings ---
    // TOptional<float> GetWorldBakeVolumeSize() const;
    // void OnWorldBakeVolumeSizeCommitted(float NewValue, ETextCommit::Type CommitType);
    // TOptional<float> GetWorldBakeVoxelExtent() const;
    // void OnWorldBakeVoxelExtentCommitted(float NewValue, ETextCommit::Type CommitType);
    // TOptional<int32> GetWorldBakeMaxDepth() const;
    // void OnWorldBakeMaxDepthCommitted(int32 NewValue, ETextCommit::Type CommitType);
    // FText GetWorldBakeSaveDirectory() const;
    // void OnWorldBakeSaveDirectoryCommitted(const FText& InText, ETextCommit::Type CommitInfo);
    // FReply OnWorldBakeBrowseButtonClicked();
    // FString GetWorldBakeObstacleProfilePath() const;
    // void OnWorldBakeObstacleProfileChanged(const FAssetData& AssetData);
    //
    // // --- Actions ---
    FReply OnBakeButtonClicked();
    FReply OnBakeLoadedVolumesButtonClicked();
    FReply OnSelectLoadedVolumesButtonClicked();
    // FReply OnBakeWorldButtonClicked();
    // FReply OnCalculateVolumesClicked();
    //
    // FText GetEstimatedVolumesText() const;
    void ShowWarningMessage(const FText& Message) const;
    FReply OnRequestPathButtonClicked();
    bool IsPIEActive() const;
    UWorld* GetEditorWorld() const;
    bool IsWorldPartitionEnabled() const;
    FLoadedNavVolumeSummary GetLoadedNavVolumeSummary() const;
    FText GetLoadedNavVolumeSummaryText() const;
    FText GetLoadedNavVolumeDetailText() const;

private:
    //=================================================================================================
    // Member Variables
    //=================================================================================================
  
    FTSTicker::FDelegateHandle ActorSelectionTickerHandle;
    
    TArray<TWeakObjectPtr<AActor>> LastSelectedActors;
    
    TArray<TSharedPtr<int32>> MapSizeOptions;
    
    //TObjectPtr<UPathfindingTunerSettings> TunerSettings;
    
    TOptional<int32> EstimatedTotalVolumes;

    float MinCoolDown =0.0001f;
    //=================================================================================================
    // Internal Functions
    //=================================================================================================
    // bool ValidateSettingsAndConfirm_Internal(const UPathfindingTunerSettings* InSettings) const;
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

template<typename ValueType>
void SPathfindingTuner::ApplyAvoidanceSetting(ValueType NewValue, void (UAvoidanceSubsystem::*Setter)(ValueType))
{
    if (UAvoidanceSubsystem* CDO = GetMutableDefault<UAvoidanceSubsystem>())
    {
        (CDO->*Setter)(NewValue);
        CDO->SaveConfig();
    }
    // Also push to live PIE instance for immediate effect.
    if (UAvoidanceSubsystem* PIE = GetPIEAvoidanceSubsystem())
    {
        (PIE->*Setter)(NewValue);
    }
    Invalidate(EInvalidateWidgetReason::LayoutAndVolatility);
}

template<typename ValueType>
TOptional<ValueType> SPathfindingTuner::ReadAvoidanceSetting(ValueType (UAvoidanceSubsystem::*Getter)() const) const
{
    // Prefer the live PIE instance value when PIE is active.
    if (const UAvoidanceSubsystem* PIE = GetPIEAvoidanceSubsystem())
    {
        return (PIE->*Getter)();
    }
    if (const UAvoidanceSubsystem* CDO = GetDefault<UAvoidanceSubsystem>())
    {
        return (CDO->*Getter)();
    }
    return TOptional<ValueType>();
}
