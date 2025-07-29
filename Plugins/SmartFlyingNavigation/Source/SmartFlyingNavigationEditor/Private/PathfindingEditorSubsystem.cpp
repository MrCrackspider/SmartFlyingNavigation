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
		UE_LOG(LogTemp, Warning, TEXT("BakeSpecificVolumes: 베이크할 볼륨이 선택되지 않았습니다."));
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
		UE_LOG(LogTemp, Error, TEXT("BakeSpecificVolumes 실패: 유효한 월드 컨텍스트를 얻을 수 없습니다."));
		return;
	}

	const FScopedTransaction Transaction(LOCTEXT("BakeSelectedVolumesTransaction", "Bake Selected Octree Volumes"));
	// 모든 복잡한 로직을 내부 함수에 위임
	BakeAndConnectVolumes_Internal(VolumesToBake, World);
}

void UPathfindingEditorSubsystem::BakeEntireWorld(const UPathfindingTunerSettings* TunerSettings)
{
	UWorld* World = GEditor->GetEditorWorldContext().World();
	if (!World || !TunerSettings)
	{
		UE_LOG(LogTemp, Error, TEXT("BakeEntireWorld 실패: 월드 또는 튜너 설정이 유효하지 않습니다."));
		return;
	}

	const FScopedTransaction Transaction(LOCTEXT("WorldBakeTransaction", "Bake Entire World Navigation"));

	// --- 1. 환경 설정 ---
	TMap<ULandscapeComponent*, TEnumAsByte<ECollisionEnabled::Type>> OriginalCollisionStates;
	SetupLandscapeCollisions_Internal(World, OriginalCollisionStates);

	// --- 2. 사전 정리 ---
	CleanupPreviousAssets(World, TunerSettings->WorldBake_SaveDirectory.Path);
	if (UOctreeNavSubsystem* NavSubsystem = World->GetSubsystem<UOctreeNavSubsystem>())
	{
		NavSubsystem->ClearAllRegisteredVolumes();
	}

	// --- 3. 베이크 범위 계산 ---
	FBox WorldBounds(ForceInit);
	if (!CalculateWorldBakeBounds(World, WorldBounds))
	{
		UE_LOG(LogTemp, Warning, TEXT("World Bake 중단: 베이크할 월드 범위를 결정할 수 없습니다."));
		RestoreLandscapeCollisions_Internal(OriginalCollisionStates);
		return;
	}

	// --- 4. 볼륨 생성 ---
	TArray<FWorldBakeResult> BakeResults;
	if (!BakeAndCreateVolumesInGrid(World, WorldBounds, TunerSettings, BakeResults))
	{
		RestoreLandscapeCollisions_Internal(OriginalCollisionStates);
		return;
	}

	if (BakeResults.IsEmpty())
	{
		UE_LOG(LogTemp, Warning, TEXT("World Bake 완료: 내비게이션 데이터를 생성할 장애물을 찾지 못했습니다."));
		RestoreLandscapeCollisions_Internal(OriginalCollisionStates);
		return;
	}

	// --- 5. 생성된 볼륨 목록 추출 ---
	TArray<AOctreeVoxelVolume*> CreatedVolumes;
	for (const FWorldBakeResult& Result : BakeResults)
	{
		CreatedVolumes.Add(Result.Volume);
	}

	// --- 6. 핵심 로직 위임: 볼륨 베이크, 포털 연결, 개별 데이터 에셋 저장 ---
	BakeAndConnectVolumes_Internal(CreatedVolumes, World);

	// --- 7. 환경 복원 ---
	RestoreLandscapeCollisions_Internal(OriginalCollisionStates);

	// =================================================================================
	// <<< 월드 내비게이션 데이터 명세(Manifest) 생성 및 저장 (최종 수정본) >>>
	// =================================================================================
	UE_LOG(LogTemp, Log, TEXT("=== 월드 내비게이션 데이터 명세 생성을 시작합니다. ==="));

	const FString ManifestDirectory = TEXT("/Game/Data/Navigation/");
	const FString ManifestAssetName = TEXT("DA_WorldNav_") + World->GetMapName();
	const FString ManifestPackagePath = FPaths::Combine(ManifestDirectory, ManifestAssetName);

	UWorldNavigationDataAsset* WorldNavData = nullptr;

	// --- "삭제 후 생성" 대신 "로드 또는 생성" 방식으로 "partially loaded" 오류를 해결합니다. ---

	// 1. 먼저 해당 경로의 에셋을 메모리로 완전히 로드하려고 시도합니다.
	WorldNavData = LoadObject<UWorldNavigationDataAsset>(nullptr, *ManifestPackagePath);

	if (WorldNavData)
	{
		// 에셋이 이미 존재하고 성공적으로 로드된 경우
		UE_LOG(LogTemp, Log, TEXT("기존 명세 에셋 '%s'를 덮어씁니다."), *ManifestPackagePath);
		WorldNavData->VolumeInfos.Empty();
	}
	else
	{
		// 에셋이 존재하지 않는 경우 (첫 베이크)
		UE_LOG(LogTemp, Log, TEXT("새로운 명세 에셋 '%s'를 생성합니다."), *ManifestPackagePath);

		// 폴더가 없을 수 있으므로 확인하고, 없다면 생성합
		if (!UEditorAssetLibrary::DoesDirectoryExist(ManifestDirectory))
		{
			UE_LOG(LogTemp, Log, TEXT("'%s' 디렉토리가 존재하지 않아 새로 생성합니다."), *ManifestDirectory);
			UEditorAssetLibrary::MakeDirectory(ManifestDirectory);
		}

		// 새 패키지와 명세 에셋을 메모리에 생성
		UPackage* ManifestPackage = CreatePackage(*ManifestPackagePath);
		WorldNavData = NewObject<UWorldNavigationDataAsset>(ManifestPackage, FName(*ManifestAssetName), RF_Public | RF_Standalone);
		FAssetRegistryModule::AssetCreated(WorldNavData);
	}

	// 생성되었거나 로드된 WorldNavData 에셋에 새로운 볼륨 정보를 채웁니다.
	for (AOctreeVoxelVolume* Volume : CreatedVolumes)
	{
		if (IsValid(Volume))
		{
			FVolumeSpatialInfo Info;
			Info.Bounds = Volume->GetVolumeBounds();
			Info.NavDataAsset = Volume->GetBakedOctreeData();
			WorldNavData->VolumeInfos.Add(Info);
		}
	}

	// 에셋을 저장 준비 상태로 만들고 디스크에 저장
	// 이 시점에서 WorldNavData는 항상 유효하고 완전히 로드된 상태임이 보장
	WorldNavData->MarkPackageDirty();
	UPackage* PackageToSave = WorldNavData->GetPackage();
	const bool bSaveSuccess = UEditorLoadingAndSavingUtils::SavePackages({ PackageToSave }, true);

	if (!bSaveSuccess)
	{
		UE_LOG(LogTemp, Error, TEXT("월드 내비게이션 데이터 명세 '%s' 저장에 실패했습니다."), *ManifestPackagePath);
	}
}
// PathfindingEditorSubsystem.cpp

#if WITH_EDITOR
void UPathfindingEditorSubsystem::BakeAndConnectVolumes_Internal(const TArray<AOctreeVoxelVolume*>& VolumesToProcess, UWorld* InWorld)
{
	if (VolumesToProcess.IsEmpty() || !InWorld)
	{
		return;
	}

	FAssetRegistryModule& AssetRegistryModule = FModuleManager::LoadModuleChecked<FAssetRegistryModule>("AssetRegistry");

	// ==================================================================
	// 단계 1: 모든 볼륨에 대한 데이터 에셋을 먼저 생성하고 할당합니다.
	// ==================================================================
	UE_LOG(LogTemp, Log, TEXT("--- 단계 1: 모든 데이터 에셋 생성 및 할당 시작 ---"));
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
			UE_LOG(LogTemp, Verbose, TEXT("'%s'에 대한 데이터 에셋 '%s' 생성 완료."), *Volume->GetName(), *AssetName);
		}
	}
	UE_LOG(LogTemp, Log, TEXT("--- 단계 1: 모든 데이터 에셋 생성 및 할당 완료 ---"));


	// ==================================================================
	// 단계 2: 각 볼륨의 내부 Octree 데이터를 베이크합니다.
	// ==================================================================
	UE_LOG(LogTemp, Log, TEXT("--- 단계 2: 내부 Octree 데이터 베이크 시작 ---"));
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
	UE_LOG(LogTemp, Log, TEXT("--- 단계 2: 내부 Octree 데이터 베이크 완료 ---"));


	// ==================================================================
	// 단계 3: 포털 연결을 위한 전체 컨텍스트를 수집합니다.
	// ==================================================================
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

	// ==================================================================
	// 단계 4: 포털을 연결합니다.
	// ==================================================================
	UE_LOG(LogTemp, Log, TEXT("--- 단계 4: 포털 연결 시작 ---"));
	// [수정] AllPortalData의 타입을 새로운 데이터 구조에 맞게 변경합니다.
	TMap<AOctreeVoxelVolume*, TMap<int32, FVolumePortalLinks>> AllPortalData;
	for (AOctreeVoxelVolume* Volume : VolumesToProcess)
	{
		TArray<FOctreeNode>* MyNodePool = BakedVolumeData.Find(Volume);
		if (MyNodePool)
		{
			// [수정] 지역 변수 PortalNodes의 타입도 새로운 데이터 구조에 맞게 변경합니다.
			TMap<int32, FVolumePortalLinks> PortalNodes;
			Volume->ConnectPortals_Bake(*MyNodePool, PortalNodes, AllVolumeNodeData, AllRelevantVolumes);
			AllPortalData.Add(Volume, PortalNodes);
		}
	}
	UE_LOG(LogTemp, Log, TEXT("--- 단계 4: 포털 연결 완료 ---"));


	// ==================================================================
	// 단계 5: 최종 데이터를 에셋에 할당하고 저장할 패키지 목록을 만듭니다.
	// ==================================================================
	UE_LOG(LogTemp, Log, TEXT("--- 단계 5: 최종 데이터 저장 준비 ---"));
	TArray<UPackage*> PackagesToSave;
	for (AOctreeVoxelVolume* Volume : VolumesToProcess)
	{
		UOctreeDataAsset* DataAsset = Volume->GetBakedOctreeData().Get();
		TArray<FOctreeNode>* NodePoolToSave = BakedVolumeData.Find(Volume);
		// [수정] PortalsToSave의 타입도 이제 올바른 TMap<int32, FVolumePortalLinks>* 가 됩니다.
		TMap<int32, FVolumePortalLinks>* PortalsToSave = AllPortalData.Find(Volume);

		if (DataAsset && NodePoolToSave && PortalsToSave)
		{
			// 이제 SaveOctreeToAsset 함수 호출 시 타입이 일치합니다.
			Volume->SaveOctreeToAsset(*NodePoolToSave, *PortalsToSave);
			if (UPackage* Package = DataAsset->GetPackage())
			{
				PackagesToSave.AddUnique(Package);
			}
		}
	}

	// ==================================================================
	// 단계 6: 모든 패키지를 한 번에 디스크에 저장합니다.
	// ==================================================================
	if (PackagesToSave.Num() > 0)
	{
		UE_LOG(LogTemp, Log, TEXT("--- 단계 6: %d개의 데이터 에셋 패키지를 디스크에 저장합니다. ---"), PackagesToSave.Num());
		UEditorLoadingAndSavingUtils::SavePackages(PackagesToSave, true);
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("--- 단계 6: 저장할 데이터 에셋 패키지가 없습니다. ---"));
	}
}
#endif

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

			// 에셋이 유효하고, 클래스가 'UOctreeDataAsset'인 경우에만 삭제 목록에 추가
			// 근데 이걸로는 조건이 부족할 것 같은데 아닌가?
			if (AssetData.IsValid() && AssetData.GetClass() == UOctreeDataAsset::StaticClass())
			{
				AssetsToDelete.Add(AssetData);
			}
		}

		// 필터링된 에셋 목록을 일괄 삭제
		if (!AssetsToDelete.IsEmpty())
		{
			UE_LOG(LogTemp, Log, TEXT("CleanupPreviousAssets: '%s' 경로에서 %d개의 이전 데이터 에셋을 삭제합니다."), *SavePath, AssetsToDelete.Num());
			ObjectTools::DeleteAssets(AssetsToDelete, true);
		}
	}

	// 월드에 존재하는 모든 AOctreeVoxelVolume 액터를 찾아서 삭제
	TArray<AActor*> ActorsToDelete;
	for (TActorIterator<AOctreeVoxelVolume> It(InWorld); It; ++It)
	{
		ActorsToDelete.Add(*It);
	}

	if (!ActorsToDelete.IsEmpty())
	{
		for (AActor* Actor : ActorsToDelete)
		{
			// InWorld->DestroyActor(Actor)는 즉시 실행되지 않을 수 있으므로,
			// 에디터 유틸리티에서는 EditorLevelUtils 등을 사용하는 것이 더 안정적일 수 있으나,
			// 현재 컨텍스트에서는 이 방법도 일반적으로 잘 동작합니다.
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

				// 이걸 하면 장애물이 없는 볼륨은 생략
				//if (NewVolume->BakeStaticData(InWorld, NodePool) && NodePool.Num() > 1)
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