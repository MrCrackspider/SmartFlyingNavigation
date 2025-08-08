// Copyright © 2025 TeamHamDev. All Rights Reserved.

#include "PathfindingEditorSubsystem.h"
#include "Subsystem/OctreeNavSubsystem.h"
#include "NavVolume/OctreeVoxelVolume.h"
#include "SPathfindingTunerSettings.h"
#include "Components/BoxComponent.h"
#include "NavData/VoxelObstacleProfile.h"
#include "NavData/WorldNavigationDataAsset.h"

#if WITH_EDITOR
#include "Editor.h"
#include "EngineUtils.h"
#include "Modules/ModuleManager.h"
#include "AssetRegistry/AssetRegistryModule.h"
#include "FileHelpers.h"
#include "EditorAssetLibrary.h"
#include "ObjectTools.h"
#include "Misc/ScopedSlowTask.h"
#include "LandscapeProxy.h"
#include "LandscapeComponent.h"
#include "Components/StaticMeshComponent.h"
#include "GameFramework/Actor.h"
#include "Kismet/GameplayStatics.h"
#endif

#define LOCTEXT_NAMESPACE "PathfindingEditorSubsystem"

#if WITH_EDITOR

FORCEINLINE uint32 GetTypeHash(const TSoftObjectPtr<UOctreeDataAsset>& Ptr)
{
	return GetTypeHash(Ptr.ToSoftObjectPath());
}

void UPathfindingEditorSubsystem::BakeSpecificVolumes(const TArray<AOctreeVoxelVolume*>& VolumesToBake)
{
	if (VolumesToBake.IsEmpty())
	{
		UE_LOG(LogTemp, Warning, TEXT("BakeSpecificVolumes: No volumes were selected to bake."));
		return;
	}

	UWorld* World = nullptr;
	for (AOctreeVoxelVolume* Volume : VolumesToBake)
	{
		if (IsValid(Volume))
		{
			World = Volume->GetWorld();
			break;
		}
	}

	if (!World)
	{
		UE_LOG(LogTemp, Error, TEXT("BakeSpecificVolumes failed: Could not get a valid world context."));
		return;
	}

	const FScopedTransaction Transaction(LOCTEXT("BakeSelectedVolumesTransaction", "Bake Selected Octree Volumes"));
	BakeAndConnectVolumes_Internal(VolumesToBake, World);
}

// void UPathfindingEditorSubsystem::BakeEntireWorld(const UPathfindingTunerSettings* TunerSettings)
// {
// 	UWorld* World = GEditor->GetEditorWorldContext().World();
// 	if (!World || !TunerSettings)
// 	{
// 		UE_LOG(LogTemp, Error, TEXT("BakeEntireWorld failed: World or tuner settings are not valid."));
// 		return;
// 	}
//
// 	const FScopedTransaction Transaction(LOCTEXT("WorldBakeTransaction", "Bake Entire World Navigation"));
//
// 	TMap<ULandscapeComponent*, TEnumAsByte<ECollisionEnabled::Type>> OriginalCollisionStates;
// 	SetupLandscapeCollisions_Internal(World, OriginalCollisionStates);
//
// 	CleanupPreviousAssets(World, TunerSettings->WorldBake_SaveDirectory.Path);
// 	if (UOctreeNavSubsystem* NavSubsystem = World->GetSubsystem<UOctreeNavSubsystem>())
// 	{
// 		NavSubsystem->ClearAllRegisteredVolumes();
// 	}
//
// 	FBox WorldBounds(ForceInit);
// 	if (!CalculateWorldBakeBounds(World, WorldBounds))
// 	{
// 		UE_LOG(LogTemp, Warning, TEXT("World Bake aborted: Could not determine world bounds to bake."));
// 		RestoreLandscapeCollisions_Internal(OriginalCollisionStates);
// 		return;
// 	}
//
// 	TArray<FWorldBakeResult> BakeResults;
// 	if (!BakeAndCreateVolumesInGrid(World, WorldBounds, TunerSettings, BakeResults))
// 	{
// 		RestoreLandscapeCollisions_Internal(OriginalCollisionStates);
// 		return;
// 	}
//
// 	if (BakeResults.IsEmpty())
// 	{
// 		UE_LOG(LogTemp, Warning, TEXT("World Bake complete: No obstacles found to generate navigation data."));
// 		RestoreLandscapeCollisions_Internal(OriginalCollisionStates);
// 		return;
// 	}
//
// 	TArray<AOctreeVoxelVolume*> CreatedVolumes;
// 	for (const FWorldBakeResult& Result : BakeResults)
// 	{
// 		CreatedVolumes.Add(Result.Volume);
// 	}
//
// 	//BakeAndConnectVolumes_Internal(CreatedVolumes, World);
//
// 	RestoreLandscapeCollisions_Internal(OriginalCollisionStates);
//
// 	UE_LOG(LogTemp, Log, TEXT("=== Starting generation of World Navigation Data Manifest. ==="));
//
// 	const FString ManifestDirectory = TEXT("/Game/Data/Navigation/");
// 	const FString ManifestAssetName = TEXT("DA_WorldNav_") + World->GetMapName();
// 	const FString ManifestPackagePath = FPaths::Combine(ManifestDirectory, ManifestAssetName);
//
// 	UWorldNavigationDataAsset* WorldNavData = nullptr;
//
// 	WorldNavData = LoadObject<UWorldNavigationDataAsset>(nullptr, *ManifestPackagePath);
//
// 	if (WorldNavData)
// 	{
// 		UE_LOG(LogTemp, Log, TEXT("Overwriting existing manifest asset '%s'."), *ManifestPackagePath);
// 		WorldNavData->VolumeInfos.Empty();
// 	}
// 	else
// 	{
// 		UE_LOG(LogTemp, Log, TEXT("Creating new manifest asset '%s'."), *ManifestPackagePath);
//
// 		if (!UEditorAssetLibrary::DoesDirectoryExist(ManifestDirectory))
// 		{
// 			UE_LOG(LogTemp, Log, TEXT("Directory '%s' does not exist, creating it."), *ManifestDirectory);
// 			UEditorAssetLibrary::MakeDirectory(ManifestDirectory);
// 		}
//
// 		UPackage* ManifestPackage = CreatePackage(*ManifestPackagePath);
// 		WorldNavData = NewObject<UWorldNavigationDataAsset>(ManifestPackage, FName(*ManifestAssetName), RF_Public | RF_Standalone);
// 		FAssetRegistryModule::AssetCreated(WorldNavData);
// 	}
//
// 	for (AOctreeVoxelVolume* Volume : CreatedVolumes)
// 	{
// 		if (IsValid(Volume))
// 		{
// 			FVolumeSpatialInfo Info;
// 			Info.Bounds = Volume->GetVolumeBounds();
// 			Info.NavDataAsset = Volume->GetBakedOctreeData();
// 			WorldNavData->VolumeInfos.Add(Info);
// 		}
// 	}
//
// 	WorldNavData->MarkPackageDirty();
// 	UPackage* PackageToSave = WorldNavData->GetPackage();
// 	const bool bSaveSuccess = UEditorLoadingAndSavingUtils::SavePackages({ PackageToSave }, true);
//
// 	if (!bSaveSuccess)
// 	{
// 		UE_LOG(LogTemp, Error, TEXT("Failed to save World Navigation Data Manifest '%s'."), *ManifestPackagePath);
// 	}
// }

void UPathfindingEditorSubsystem::BakeAndConnectVolumes_Internal(const TArray<AOctreeVoxelVolume*>& VolumesToProcess, UWorld* InWorld)
{
	if (VolumesToProcess.IsEmpty() || !InWorld)
	{
		return;
	}

	FAssetRegistryModule& AssetRegistryModule = FModuleManager::LoadModuleChecked<FAssetRegistryModule>("AssetRegistry");

	UE_LOG(LogTemp, Log, TEXT("--- Step 1: Starting creation and assignment of all data assets ---"));
	for (AOctreeVoxelVolume* Volume : VolumesToProcess)
	{
		if (!IsValid(Volume)) continue;
		Volume->Modify();

		FString PackageName, AssetName;
		if (Volume->GetBakedAssetPath(PackageName, AssetName))
		{
			FAssetData ExistingAssetData = AssetRegistryModule.Get().GetAssetByObjectPath(FSoftObjectPath(PackageName));
			if (ExistingAssetData.IsValid())
			{
				ObjectTools::DeleteAssets({ ExistingAssetData }, false);
			}

			UPackage* Package = CreatePackage(*PackageName);
			Package->FullyLoad();
			UOctreeDataAsset* NewDataAsset = NewObject<UOctreeDataAsset>(Package, FName(*AssetName), RF_Public | RF_Standalone);

			Volume->SetBakedOctreeData(NewDataAsset);
			FAssetRegistryModule::AssetCreated(NewDataAsset);
			UE_LOG(LogTemp, Verbose, TEXT("Data asset '%s' created for '%s'."), *AssetName, *Volume->GetName());
		}
	}
	UE_LOG(LogTemp, Log, TEXT("--- Step 1: Finished creation and assignment of all data assets ---"));

	UE_LOG(LogTemp, Log, TEXT("--- Step 2: Starting bake of internal Octree data ---"));
	TMap<AOctreeVoxelVolume*, TArray<FOctreeNode>> BakedVolumeData;
	for (AOctreeVoxelVolume* Volume : VolumesToProcess)
	{
		if (!IsValid(Volume)) continue;
		TArray<FOctreeNode> NodePool;
		if (Volume->BakeStaticData(InWorld, NodePool))
		{
			BakedVolumeData.Add(Volume, NodePool);
		}
	}
	UE_LOG(LogTemp, Log, TEXT("--- Step 2: Finished bake of internal Octree data ---"));

	TArray<AOctreeVoxelVolume*> AllRelevantVolumes;
	TMap<AOctreeVoxelVolume*, TArray<FOctreeNode>> AllVolumeNodeData;
	AllRelevantVolumes.Append(VolumesToProcess);
	AllVolumeNodeData.Append(BakedVolumeData);

	for (TActorIterator<AOctreeVoxelVolume> It(InWorld); It; ++It)
	{
		AOctreeVoxelVolume* ExistingVolume = *It;
		if (IsValid(ExistingVolume) && !AllRelevantVolumes.Contains(ExistingVolume))
		{
			AllRelevantVolumes.Add(ExistingVolume);
			if (UOctreeDataAsset* DataAsset = ExistingVolume->GetBakedOctreeData().LoadSynchronous())
			{
				DataAsset->DecompressAndInitialize();
				AllVolumeNodeData.Add(ExistingVolume, DataAsset->NodePool);
			}
		}
	}

	UE_LOG(LogTemp, Log, TEXT("--- Step 4: Starting portal connection ---"));
	TMap<AOctreeVoxelVolume*, TMap<int32, FVolumePortalLinks>> AllPortalData;
	for (AOctreeVoxelVolume* Volume : VolumesToProcess)
	{
		TArray<FOctreeNode>* MyNodePool = BakedVolumeData.Find(Volume);
		if (MyNodePool)
		{
			TMap<int32, FVolumePortalLinks> PortalNodes;
			Volume->ConnectPortals_Bake(*MyNodePool, PortalNodes, AllVolumeNodeData, AllRelevantVolumes);
			AllPortalData.Add(Volume, PortalNodes);
		}
	}
	UE_LOG(LogTemp, Log, TEXT("--- Step 4: Finished portal connection ---"));

	UE_LOG(LogTemp, Log, TEXT("--- Step 5: Preparing final data for saving ---"));
	TArray<UPackage*> PackagesToSave;
	for (AOctreeVoxelVolume* Volume : VolumesToProcess)
	{
		UOctreeDataAsset* DataAsset = Volume->GetBakedOctreeData().Get();
		TArray<FOctreeNode>* NodePoolToSave = BakedVolumeData.Find(Volume);
		TMap<int32, FVolumePortalLinks>* PortalsToSave = AllPortalData.Find(Volume);

		if (DataAsset && NodePoolToSave && PortalsToSave)
		{
			Volume->SaveOctreeToAsset(*NodePoolToSave, *PortalsToSave);
			if (UPackage* Package = DataAsset->GetPackage())
			{
				PackagesToSave.AddUnique(Package);
			}
		}
	}

	if (PackagesToSave.Num() > 0)
	{
		UE_LOG(LogTemp, Log, TEXT("--- Step 6: Saving %d data asset packages to disk. ---"), PackagesToSave.Num());
		UEditorLoadingAndSavingUtils::SavePackages(PackagesToSave, true);
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("--- Step 6: No data asset packages to save. ---"));
	}
}
#endif
#if WITH_EDITOR
void UPathfindingEditorSubsystem::CleanupPreviousAssets(UWorld* InWorld, const FString& InSavePath) const
{
	FString SavePath = InSavePath;
	if (!SavePath.StartsWith(TEXT("/")))
	{
		SavePath = FPaths::Combine(TEXT("/Game"), SavePath);
	}
	SavePath.RemoveFromEnd(TEXT("/")); 

	TArray<FString> AssetPathStrings = UEditorAssetLibrary::ListAssets(SavePath, true, false);
	if (!AssetPathStrings.IsEmpty())
	{
		TArray<FAssetData> AssetsToDelete;
		FAssetRegistryModule& AssetRegistryModule = FModuleManager::LoadModuleChecked<FAssetRegistryModule>("AssetRegistry");

		for (const FString& Path : AssetPathStrings)
		{
			FAssetData AssetData = AssetRegistryModule.Get().GetAssetByObjectPath(FSoftObjectPath(Path));
			if (AssetData.IsValid() && AssetData.GetClass() == UOctreeDataAsset::StaticClass())
			{
				AssetsToDelete.Add(AssetData);
			}
		}

		if (!AssetsToDelete.IsEmpty())
		{
			UE_LOG(LogTemp, Log, TEXT("CleanupPreviousAssets: Deleting %d previous data assets from '%s'."), AssetsToDelete.Num(), *SavePath);
			ObjectTools::DeleteAssets(AssetsToDelete, true);
		}
	}

	TArray<AActor*> ActorsToDelete;
	for (TActorIterator<AOctreeVoxelVolume> It(InWorld); It; ++It)
	{
		ActorsToDelete.Add(*It);
	}

	if (!ActorsToDelete.IsEmpty())
	{
		for (AActor* Actor : ActorsToDelete)
		{
			InWorld->DestroyActor(Actor);
		}
	}
}

bool UPathfindingEditorSubsystem::CalculateWorldBakeBounds(UWorld* InWorld, FBox& OutBounds) const
{
	OutBounds.Init();
	for (TActorIterator<AActor> It(InWorld); It; ++It)
	{
		AActor* Actor = *It;
		if (!Actor || Actor->ActorHasTag(FName("NoNavBake"))) continue;

		TArray<UStaticMeshComponent*> StaticMeshComponents;
		Actor->GetComponents<UStaticMeshComponent>(StaticMeshComponents);
		for (UStaticMeshComponent* Component : StaticMeshComponents)
		{
			if (Component && Component->GetCollisionEnabled() != ECollisionEnabled::NoCollision)
			{
				OutBounds += Component->Bounds.GetBox();
			}
		}
	}
	for (TActorIterator<ALandscapeProxy> It(InWorld); It; ++It)
	{
		if (*It && !(*It)->ActorHasTag(FName("NoNavBake")))
		{
			OutBounds += (*It)->GetComponentsBoundingBox(true);
		}
	}
	return OutBounds.IsValid != 0;
}

bool UPathfindingEditorSubsystem::BakeAndCreateVolumesInGrid(
	UWorld* InWorld,
	const FBox& InWorldBounds,
	const UPathfindingTunerSettings* InSettings,
	TArray<FWorldBakeResult>& OutBakeResults)
{
	OutBakeResults.Empty();

	const float VolumeSize = InSettings->WorldBake_VolumeSize;
	if (VolumeSize <= 0) return false;

	const int32 GridMinX = FMath::FloorToInt(InWorldBounds.Min.X / VolumeSize);
	const int32 GridMaxX = FMath::CeilToInt(InWorldBounds.Max.X / VolumeSize);
	const int32 GridMinY = FMath::FloorToInt(InWorldBounds.Min.Y / VolumeSize);
	const int32 GridMaxY = FMath::CeilToInt(InWorldBounds.Max.Y / VolumeSize);
	const int32 GridMinZ = FMath::FloorToInt(InWorldBounds.Min.Z / VolumeSize);
	const int32 GridMaxZ = FMath::CeilToInt(InWorldBounds.Max.Z / VolumeSize);

	const int32 TotalVolumes = (GridMaxX - GridMinX) * (GridMaxY - GridMinY) * (GridMaxZ - GridMinZ);
	if (TotalVolumes <= 0) return true;

	FScopedSlowTask BakeTask(TotalVolumes, LOCTEXT("BakingWorldTask", "Baking World Navigation Volumes..."));
	BakeTask.MakeDialog(true);

	UVoxelObstacleProfile* ObstacleProfile = InSettings->WorldBake_ObstacleProfile.LoadSynchronous();
	if (!ObstacleProfile) return false;

	for (int32 GZ = GridMinZ; GZ < GridMaxZ; ++GZ)
	{
		for (int32 GY = GridMinY; GY < GridMaxY; ++GY)
		{
			for (int32 GX = GridMinX; GX < GridMaxX; ++GX)
			{
				if (BakeTask.ShouldCancel())
				{
					for (const FWorldBakeResult& Result : OutBakeResults)
					{
						if (Result.Volume) Result.Volume->Destroy();
					}
					OutBakeResults.Empty();
					return false;
				}
				BakeTask.EnterProgressFrame(1.f, FText::Format(LOCTEXT("BakingVolumeProgress", "Baking Volume ({0}, {1}, {2})"), FText::AsNumber(GX), FText::AsNumber(GY), FText::AsNumber(GZ)));

				const FBox VolumeBounds(FVector(GX, GY, GZ) * VolumeSize, FVector(GX + 1, GY + 1, GZ + 1) * VolumeSize);
				AOctreeVoxelVolume* NewVolume = InWorld->SpawnActor<AOctreeVoxelVolume>(VolumeBounds.GetCenter(), FRotator::ZeroRotator);
				if (!NewVolume) continue;

				NewVolume->Modify();
				NewVolume->SetActorLabel(FString::Printf(TEXT("NavVolume_%d_%d_%d"), GX, GY, GZ));
				NewVolume->SetFolderPath(FName("/NavVolumes"));
				NewVolume->VolumeBox->SetBoxExtent(VolumeBounds.GetExtent(), true);
				NewVolume->VoxelExtent = InSettings->WorldBake_VoxelExtent;
				NewVolume->SetMapSize(FIntVector(FMath::RoundToInt(VolumeSize / NewVolume->VoxelExtent)));
				NewVolume->SetMaxOctreeDepth(InSettings->WorldBake_MaxOctreeDepth);
				NewVolume->SetStaticObstacleProfile(ObstacleProfile);
				NewVolume->BakedDataSaveDirectory.Path = InSettings->WorldBake_SaveDirectory.Path;

				TArray<FOctreeNode> NodePool;
				if (NewVolume->BakeStaticData(InWorld, NodePool))
				{
					FWorldBakeResult Result;
					Result.Volume = NewVolume;
					Result.NodePool = MoveTemp(NodePool);
					OutBakeResults.Add(Result);
				}
				else
				{
					NewVolume->Destroy();
				}
			}
		}
	}

	return true;
}

void UPathfindingEditorSubsystem::SaveBakedResults(const TArray<TObjectPtr<UPackage>>& InPackagesToSave, const TArray<TObjectPtr<AActor>>& InCreatedActors) const
{
	if (InPackagesToSave.IsEmpty() && InCreatedActors.IsEmpty())
	{
		return;
	}

	TArray<UPackage*> PackagesToSave;
	for (const auto& Package : InPackagesToSave)
	{
		PackagesToSave.Add(Package.Get());
	}

	TSet<UPackage*> LevelPackagesToSave;
	for (const auto& Actor : InCreatedActors)
	{
		if (IsValid(Actor))
		{
			LevelPackagesToSave.Add(Actor->GetLevel()->GetPackage());
		}
	}
	PackagesToSave.Append(LevelPackagesToSave.Array());

	UEditorLoadingAndSavingUtils::SavePackages(PackagesToSave, true);
}

void UPathfindingEditorSubsystem::SetupLandscapeCollisions_Internal(UWorld* InWorld, TMap<ULandscapeComponent*, TEnumAsByte<ECollisionEnabled::Type>>& OutOriginalStates) const
{
	for (TActorIterator<ALandscapeProxy> It(InWorld); It; ++It)
	{
		if (ALandscapeProxy* Proxy = *It)
		{
			TArray<ULandscapeComponent*> LandscapeComponents;
			Proxy->GetComponents<ULandscapeComponent>(LandscapeComponents);
			for (ULandscapeComponent* Comp : LandscapeComponents)
			{
				if (Comp)
				{
					OutOriginalStates.Add(Comp, Comp->GetCollisionEnabled());
					Comp->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
				}
			}
		}
	}
}

void UPathfindingEditorSubsystem::RestoreLandscapeCollisions_Internal(const TMap<ULandscapeComponent*, TEnumAsByte<ECollisionEnabled::Type>>& InOriginalStates) const
{
	for (auto const& [Component, OriginalState] : InOriginalStates)
	{
		if (Component)
		{
			Component->SetCollisionEnabled(OriginalState);
		}
	}
}

#endif // WITH_EDITOR

#undef LOCTEXT_NAMESPACE