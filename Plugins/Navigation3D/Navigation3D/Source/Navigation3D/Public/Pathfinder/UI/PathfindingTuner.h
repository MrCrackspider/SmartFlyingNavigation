#pragma once

#include "CoreMinimal.h"
#include "Widgets/SCompoundWidget.h"

// 전방 선언
class UPathfindingSubsystem;
class UPathfindingTunerSettings;
class AActor;
struct FAssetData;

/**
 * 길찾기 시스템의 다양한 속성을 조정하고 베이킹을 트리거하는 에디터 UI 위젯입니다.
 */
class SPathfindingTuner : public SCompoundWidget
{
public:
    SLATE_BEGIN_ARGS(SPathfindingTuner) {}
    SLATE_END_ARGS()

    /** 위젯을 구성합니다. */
    void Construct(const FArguments& InArgs);

    /** 소멸자에서 티커를 정리합니다. */
    ~SPathfindingTuner();

private:
    //=================================================================================================
    // UI Section Builders
    //=================================================================================================
    TSharedRef<SWidget> BuildPerformanceSettingsSection();
    TSharedRef<SWidget> BuildOctreeVolumeSettingsSection();
    TSharedRef<SWidget> BuildBakedDataSettingsSection();
    TSharedRef<SWidget> BuildAgentSettingsSection();
    TSharedRef<SWidget> BuildVoxelDebugSettingsSection();
    
    //=================================================================================================
    // Actor Selection Handling
    //=================================================================================================
    /** 에디터의 액터 선택 변경을 감지하기 위해 매 틱마다 호출됩니다. */
    bool TickActorSelection(float DeltaTime);
    /** 액터 선택이 변경되었을 때 UI를 갱신합니다. */
    void OnActorSelectionChanged();

    //=================================================================================================
    // Property Getters & Setters (Generic Helpers)
    //=================================================================================================
    /** 여러 객체에 걸쳐 공통된 속성 값을 가져옵니다. 다중 값이면 TOptional은 비어있습니다. */
    template <typename ObjectType, typename PropertyType>
    TOptional<PropertyType> GetCommonPropertyValue(const TArray<ObjectType*>& Objects, TFunctionRef<PropertyType(const ObjectType*)> Getter) const;

    /** 여러 객체의 속성 값을 일괄적으로 설정합니다. */
    template <typename ObjectType, typename PropertyType>
    void SetCommonPropertyValue(const TArray<ObjectType*>& Objects, const PropertyType& NewValue, const FText& TransactionText, TFunctionRef<void(ObjectType*, const PropertyType&)> Setter);

    /** 선택된 액터 중 특정 클래스 타입의 액터만 필터링하여 반환합니다. */
    template <typename T>
    TArray<T*> GetSelectedActorsOfType() const;

    /** 선택된 액터들로부터 특정 타입의 컴포넌트를 모두 찾아 배열로 반환합니다. */
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
    void OnDebugLineWidthCommitted(float NewValue,
                                   ETextCommit::Type CommitType);


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
private:
    //=================================================================================================
    // Member Variables
    //=================================================================================================
    /** 길찾기 서브시스템에 대한 약한 포인터 */
    TWeakObjectPtr<UPathfindingSubsystem> PathfindingSubsystem;

    /** 에디터 액터 선택을 감지하는 티커 핸들 */
    FTSTicker::FDelegateHandle ActorSelectionTickerHandle;

    /** 마지막으로 감지된 선택된 액터 목록 */
    TArray<TWeakObjectPtr<AActor>> LastSelectedActors;

    /** 맵 크기 콤보박스 옵션 */
    TArray<TSharedPtr<int32>> MapSizeOptions;

    /** UI 상태를 저장/로드하는 세팅 객체 */
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