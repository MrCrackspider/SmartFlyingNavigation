// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "DynamicObstacle.h"
#include "VoxelVolumeBase.generated.h"

class UHierarchicalInstancedStaticMeshComponent;
class UBoxComponent;

/**
 * @enum EVolumeDataStructure
 * @brief Defines the type of data structure used for voxel volume representation.
 */
UENUM(BlueprintType)
enum class EVolumeDataStructure : uint8
{
	NONE,
	UniformGrid,
	Octree,
	Other
};

/**
 * @class AVoxelVolumeBase
 * @brief Declared as an 'abstract class' to ensure that child classes must implement core functionalities.
 * This class serves as the base for all voxel volume actors, providing common properties and functionalities.
 */
UCLASS(Abstract)
class SMARTFLYINGNAVIGATION_API AVoxelVolumeBase : public AActor
{
	GENERATED_BODY()

public:
	/**
	 * @brief Default constructor.
	 */
	AVoxelVolumeBase();

	/**
 * @brief Called when an instance of this class is placed or updated in the editor.
 * @param Transform The transform of the actor.
 */
	virtual void OnConstruction(const FTransform& Transform) override;
protected:
	//~======================================================================
	// AActor overrides
	//~======================================================================

	/**
	 * @brief Called when the game starts or when spawned.
	 */
	virtual void BeginPlay() override;

 

public:
	/**
	 * @brief Called every frame.
	 * @param DeltaTime The time since the last frame.
	 */
	virtual void Tick(float DeltaTime) override;

	/**
	 * @brief Called when the actor is being destroyed.
	 * @param EndPlayReason The reason why the play is ending.
	 */
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	//~======================================================================
	// Volume Settings
	//~======================================================================

	/**
	 * @brief The extent (size) of a single voxel.
	 */
	UPROPERTY(BlueprintReadOnly, Category = "Voxel|VolumeSettings")
	float VoxelExtent;

	/**
	 * @brief The size of the volume in voxels.
	 */
	UPROPERTY(BlueprintReadOnly, Category = "Voxel|VolumeSettings")
	FIntVector MapSize;

	/**
	 * @brief The component that acts as the pivot for this volume.
	 */
	UPROPERTY()
	TObjectPtr<USceneComponent> PivotComponent;

	/**
	 * @brief The bounding box component for the volume.
	 */
	UPROPERTY()
	TObjectPtr<UBoxComponent> VolumeBox;

	//~======================================================================
	// Voxel Conversion and Queries
	//~======================================================================

	/**
	 * @brief Pure virtual function to get the type of data structure used by the volume.
	 * @return The data structure type.
	 */
	virtual EVolumeDataStructure GetStructureType() const PURE_VIRTUAL(AVoxelVolumeBase::GetStructureType, return EVolumeDataStructure::NONE;);

	/**
	 * @brief Converts a voxel index to its corresponding world position.
	 * @param VoxelIndex The index of the voxel.
	 * @return The world position of the voxel's center.
	 */
	UFUNCTION(BlueprintPure, Category = "Voxel")
	virtual FVector VoxelIndexToWorld(const FIntVector& VoxelIndex) const;

	/**
	 * @brief Converts a world position to the corresponding voxel index.
	 * @param WorldPosition The position in world space.
	 * @return The voxel index containing the world position.
	 */
	UFUNCTION(BlueprintPure, Category = "Voxel")
	virtual FIntVector WorldToVoxelIndex(const FVector& WorldPosition) const;

	/**
	 * @brief Checks if a given voxel index is within the bounds of the volume.
	 * @param Index The voxel index to check.
	 * @return True if the index is valid, false otherwise.
	 */
	bool IsIndexValid(const FIntVector& Index) const;

	/**
	 * @brief Clamps the given min and max voxel indices to be within the volume's bounds.
	 * @param InMinVoxelIndex The minimum voxel index to clamp.
	 * @param InMaxVoxelIndex The maximum voxel index to clamp.
	 * @param OutClampedMin The resulting clamped minimum index.
	 * @param OutClampedMax The resulting clamped maximum index.
	 * @return True if the original indices were at least partially within the volume, false otherwise.
	 */
	bool ClampVoxelIndicesToVolume(const FIntVector& InMinVoxelIndex, const FIntVector& InMaxVoxelIndex, FIntVector& OutClampedMin, FIntVector& OutClampedMax) const;

	/**
	 * @brief Gets all components overlapping with the volume that match the given query parameters.
	 * @param ObjectQueryParams The collision object query parameters to filter the components.
	 * @param OutResults An array to store the overlap results.
	 * @return True if any overlapping components were found, false otherwise.
	 */
	bool GetOverlappingComponentsInVolume(const FCollisionObjectQueryParams& ObjectQueryParams, TArray<FOverlapResult>& OutResults) const;

	/**
	 * @brief Checks if a specific voxel is overlapping with a given primitive component.
	 * @param VoxelIndex The index of the voxel to test.
	 * @param ComponentToTest The component to check for overlap.
	 * @return True if the voxel and component overlap, false otherwise.
	 */
	bool IsVoxelOverlapping(const FIntVector& VoxelIndex, UPrimitiveComponent* ComponentToTest) const;

	/**
	 * @brief Gets the volume's bounding box component.
	 * @return A pointer to the UBoxComponent.
	 */
	UBoxComponent* GetVolumeBox() const { return VolumeBox; }

	//~======================================================================
	// Debug Visualization
	//~======================================================================

	bool GetbDrawDebugBox() const { return bDrawDebugBox; }
	void SetbDrawDebugBox(bool Value) { bDrawDebugBox = Value; }
	
	/**
	 * @brief If true, draws debug boxes for the volume.
	 */
	UPROPERTY(EditAnywhere, Category = "Voxel Volume|Debug")
	bool bDrawDebugBox;


	float GetDebugLineWidth() const { return DebugLineWidth; }
	void SetDebugLineWidth(float Value) { DebugLineWidth = Value; }
	/**
	 * @brief The thickness of the debug lines.
	 */
	UPROPERTY()
	float DebugLineWidth;

#if WITH_EDITORONLY_DATA

	/**
	 * @brief The static mesh to use for debug visualization.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Voxel|Debug")
	TSoftObjectPtr<UStaticMesh> DebugMeshAsset;

	/**
	 * @brief Hierarchical Instanced Static Mesh Component for visualizing solid nodes.
	 */
	UPROPERTY()
	TObjectPtr<UHierarchicalInstancedStaticMeshComponent> DebugSolidNodesComponent;

	/**
	 * @brief Hierarchical Instanced Static Mesh Component for visualizing empty nodes.
	 */
	UPROPERTY()
	TObjectPtr<UHierarchicalInstancedStaticMeshComponent> DebugEmptyNodesComponent;

	/**
	 * @brief Hierarchical Instanced Static Mesh Component for visualizing mixed (partially filled) nodes.
	 */
	UPROPERTY()
	TObjectPtr<UHierarchicalInstancedStaticMeshComponent> DebugMixedNodesComponent;

	/**
	 * @brief Soft path to the material for visualizing solid nodes (configurable in the editor).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Voxel|Debug", meta = (AllowedClasses = "MaterialInterface"))
	FSoftObjectPath DebugSolidNodeMaterialPath;

	/**
	 * @brief Soft path to the material for visualizing empty nodes (configurable in the editor).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Voxel|Debug", meta = (AllowedClasses = "MaterialInterface"))
	FSoftObjectPath DebugEmptyNodeMaterialPath;

	/**
	 * @brief Soft path to the material for visualizing mixed nodes (configurable in the editor).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Voxel|Debug", meta = (AllowedClasses = "MaterialInterface"))
	FSoftObjectPath DebugMixedNodeMaterialPath;

	/**
	 * @brief Material instance for solid nodes.
	 */
	UPROPERTY()
	TObjectPtr<UMaterialInterface> SolidNodeMaterial;

	/**
	 * @brief Material instance for empty nodes.
	 */
	UPROPERTY()
	TObjectPtr<UMaterialInterface> EmptyNodeMaterial;

	/**
	 * @brief Material instance for mixed nodes.
	 */
	UPROPERTY()
	TObjectPtr<UMaterialInterface> MixedNodeMaterial;

#endif

	/**
	 * @brief Updates the visualization of obstacles for debugging purposes.
	 */
	virtual void UpdateDebugObstacleVisualization();
	void DrawMainVolumeDebugBox() const;
protected:
	/**
	 * @brief Timer handle for scheduling debug visualization updates.
	 */
	FTimerHandle DebugUpdateTimer;

	/**
	 * @brief Flag to prevent concurrent updates to the visualization.
	 */
	bool bIsUpdatingVisualization = false;

	/**
	 * @brief The set of dynamic obstacle components currently registered with this volume.
	 */
	UPROPERTY()
	TSet<TObjectPtr<UDynamicObstacleComponent>> RegisteredObstacles;
};