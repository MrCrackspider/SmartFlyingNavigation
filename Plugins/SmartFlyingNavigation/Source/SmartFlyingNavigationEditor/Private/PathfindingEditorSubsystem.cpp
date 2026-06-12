// Copyright © 2025 TeamHamDev. All Rights Reserved.

#include "PathfindingEditorSubsystem.h"
#include "Bake/OctreeBaker.h"
#include "Subsystem/OctreeNavSubsystem.h"
#include "NavVolume/OctreeVoxelVolume.h"
#include "Component/NavBakeExcludeComponent.h"
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
#include "Misc/ScopedSlowTask.h"
#include "ScopedTransaction.h"
#include "Components/PrimitiveComponent.h"
#include "GameFramework/Actor.h"
#endif

#define LOCTEXT_NAMESPACE "PathfindingEditorSubsystem"

#if WITH_EDITOR

/** Headless/automation entry point: bakes all stale loaded volumes in the current editor world. */
static FAutoConsoleCommandWithWorldAndArgs GBakeNavCmd(
	TEXT("sfn.BakeNav"),
	TEXT("Bakes all stale loaded OctreeVoxelVolumes in the current editor world (editor only)."),
	FConsoleCommandWithWorldAndArgsDelegate::CreateStatic(
		[](const TArray<FString>& Args, UWorld* World)
		{
			if (UPathfindingEditorSubsystem* Subsystem = GEditor ? GEditor->GetEditorSubsystem<UPathfindingEditorSubsystem>() : nullptr)
			{
				Subsystem->BakeLoadedVolumes(World);
			}
		}));

static FText FormatBytesForBakeEstimate(int64 ByteCount)
{
	constexpr double KiB = 1024.0;
	constexpr double MiB = KiB * 1024.0;
	constexpr double GiB = MiB * 1024.0;

	if (ByteCount >= static_cast<int64>(GiB))
	{
		return FText::Format(LOCTEXT("BakeEstimateGiB", "{0} GiB"), FText::AsNumber(static_cast<double>(ByteCount) / GiB));
	}
	if (ByteCount >= static_cast<int64>(MiB))
	{
		return FText::Format(LOCTEXT("BakeEstimateMiB", "{0} MiB"), FText::AsNumber(static_cast<double>(ByteCount) / MiB));
	}
	if (ByteCount >= static_cast<int64>(KiB))
	{
		return FText::Format(LOCTEXT("BakeEstimateKiB", "{0} KiB"), FText::AsNumber(static_cast<double>(ByteCount) / KiB));
	}
	return FText::Format(LOCTEXT("BakeEstimateBytes", "{0} bytes"), FText::AsNumber(ByteCount));
}

/** Hash of every bake-relevant volume setting. A mismatch against the manifest means the chunk is stale. */
static FString BuildVolumeSourceHash(const AOctreeVoxelVolume* Volume)
{
	if (!Volume)
	{
		return FString();
	}

	const FBox Bounds = Volume->GetVolumeBounds();
	return FString::Printf(
		TEXT("%s|%s|%.3f|%d|%s|band%d"),
		*Volume->GetPathName(),
		*Volume->GetMapSize().ToString(),
		Volume->VoxelExtent,
		Volume->GetMaxOctreeDepth(),
		*Bounds.ToString(),
		Volume->GetDistanceFieldBandVoxels());
}

static FIntVector EstimateChunkCoord(const AOctreeVoxelVolume* Volume)
{
	if (!Volume)
	{
		return FIntVector::ZeroValue;
	}

	const FBox Bounds = Volume->GetVolumeBounds();
	const FVector Size = Bounds.GetSize();
	const FVector Center = Bounds.GetCenter();
	return FIntVector(
		Size.X > 0.f ? FMath::FloorToInt(Center.X / Size.X) : 0,
		Size.Y > 0.f ? FMath::FloorToInt(Center.Y / Size.Y) : 0,
		Size.Z > 0.f ? FMath::FloorToInt(Center.Z / Size.Z) : 0);
}

static FString GetManifestPackagePathForWorld(const UWorld* InWorld, FString* OutDirectory = nullptr, FString* OutAssetName = nullptr)
{
	const FString MapName = UWorld::RemovePIEPrefix(InWorld->GetMapName());
	const FString ManifestDirectory = TEXT("/Game/Data/Navigation/");
	const FString ManifestAssetName = TEXT("DA_WorldNav_") + MapName;
	if (OutDirectory) *OutDirectory = ManifestDirectory;
	if (OutAssetName) *OutAssetName = ManifestAssetName;
	return FPaths::Combine(ManifestDirectory, ManifestAssetName);
}

static UWorldNavigationDataAsset* LoadManifestForWorld(const UWorld* InWorld, bool bCreateIfMissing)
{
	if (!InWorld)
	{
		return nullptr;
	}

	FString ManifestDirectory, ManifestAssetName;
	const FString ManifestPackagePath = GetManifestPackagePathForWorld(InWorld, &ManifestDirectory, &ManifestAssetName);

	UWorldNavigationDataAsset* Manifest = LoadObject<UWorldNavigationDataAsset>(nullptr, *ManifestPackagePath);
	if (!Manifest && bCreateIfMissing)
	{
		if (!UEditorAssetLibrary::DoesDirectoryExist(ManifestDirectory))
		{
			UEditorAssetLibrary::MakeDirectory(ManifestDirectory);
		}

		UPackage* ManifestPackage = CreatePackage(*ManifestPackagePath);
		Manifest = NewObject<UWorldNavigationDataAsset>(ManifestPackage, FName(*ManifestAssetName), RF_Public | RF_Standalone);
		FAssetRegistryModule::AssetCreated(Manifest);
	}

	return Manifest;
}

/** Locates the manifest chunk record covering this volume by matching bounds. */
static const FNavigationChunkRecord* FindChunkRecordForVolume(const UWorldNavigationDataAsset* Manifest, const AOctreeVoxelVolume* Volume)
{
	if (!Manifest || !IsValid(Volume))
	{
		return nullptr;
	}

	const FBox VolumeBounds = Volume->GetVolumeBounds();
	const FVector VolumeCenter = VolumeBounds.GetCenter();
	const FVector VolumeExtent = VolumeBounds.GetExtent();

	return Manifest->Chunks.FindByPredicate(
		[&VolumeCenter, &VolumeExtent](const FNavigationChunkRecord& Chunk)
		{
			return Chunk.Bounds.IsInsideOrOn(VolumeCenter) && Chunk.Bounds.GetExtent().Equals(VolumeExtent, 1.0f);
		});
}

static const FNavigationChunkPayload* FindChunkPayload(const UWorldNavigationDataAsset* Manifest, int32 ChunkId)
{
	if (!Manifest)
	{
		return nullptr;
	}

	return Manifest->Payloads.FindByPredicate(
		[ChunkId](const FNavigationChunkPayload& Payload)
		{
			return Payload.ChunkId == ChunkId &&
				(Payload.CompressedNodePool.Num() > 0 || Payload.CompressedAirwayGraph.Num() > 0);
		});
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

	TArray<FNavVolumeBakeIssue> Issues;
	if (!ValidateVolumesForBake(VolumesToBake, Issues))
	{
		for (const FNavVolumeBakeIssue& Issue : Issues)
		{
			const FString VolumeLabel = Issue.Volume.IsValid() ? Issue.Volume->GetActorLabel() : TEXT("Invalid Volume");
			UE_LOG(LogTemp, Error, TEXT("BakeSpecificVolumes validation failed for '%s': %s"), *VolumeLabel, *Issue.Message.ToString());
		}
		return;
	}

	const FScopedTransaction Transaction(LOCTEXT("BakeSelectedVolumesTransaction", "Bake Selected Octree Volumes"));
	BakeAndConnectVolumes_Internal(VolumesToBake, World);
}

void UPathfindingEditorSubsystem::BakeLoadedVolumes(UWorld* InWorld)
{
	if (!InWorld)
	{
		UE_LOG(LogTemp, Error, TEXT("BakeLoadedVolumes failed: World is invalid."));
		return;
	}

	TArray<AOctreeVoxelVolume*> LoadedVolumes;
	GetLoadedOctreeVolumes(InWorld, LoadedVolumes);

	if (LoadedVolumes.IsEmpty())
	{
		UE_LOG(LogTemp, Warning, TEXT("BakeLoadedVolumes: No loaded OctreeVoxelVolume actors were found."));
		return;
	}

	TArray<AOctreeVoxelVolume*> VolumesNeedingBake;
	GetVolumesNeedingBake(LoadedVolumes, VolumesNeedingBake);

	if (VolumesNeedingBake.IsEmpty())
	{
		UE_LOG(LogTemp, Log, TEXT("BakeLoadedVolumes: All loaded OctreeVoxelVolume actors are up to date."));
		return;
	}

	TArray<FNavVolumeBakeIssue> Issues;
	if (!ValidateVolumesForBake(VolumesNeedingBake, Issues))
	{
		for (const FNavVolumeBakeIssue& Issue : Issues)
		{
			const FString VolumeLabel = Issue.Volume.IsValid() ? Issue.Volume->GetActorLabel() : TEXT("Invalid Volume");
			UE_LOG(LogTemp, Error, TEXT("BakeLoadedVolumes validation failed for '%s': %s"), *VolumeLabel, *Issue.Message.ToString());
		}
		return;
	}

	const FScopedTransaction Transaction(LOCTEXT("BakeLoadedVolumesTransaction", "Bake Loaded Octree Volumes"));
	BakeAndConnectVolumes_Internal(VolumesNeedingBake, InWorld);
}

void UPathfindingEditorSubsystem::GetLoadedOctreeVolumes(UWorld* InWorld, TArray<AOctreeVoxelVolume*>& OutVolumes) const
{
	OutVolumes.Reset();
	if (!InWorld)
	{
		return;
	}

	for (TActorIterator<AOctreeVoxelVolume> It(InWorld); It; ++It)
	{
		AOctreeVoxelVolume* Volume = *It;
		if (IsValid(Volume))
		{
			OutVolumes.Add(Volume);
		}
	}
}

void UPathfindingEditorSubsystem::GetVolumesNeedingBake(const TArray<AOctreeVoxelVolume*>& CandidateVolumes, TArray<AOctreeVoxelVolume*>& OutVolumes) const
{
	OutVolumes.Reset();

	for (AOctreeVoxelVolume* Volume : CandidateVolumes)
	{
		if (IsValid(Volume) && IsVolumeBakeStale(Volume))
		{
			OutVolumes.Add(Volume);
		}
	}
}

FLoadedNavVolumeSummary UPathfindingEditorSubsystem::GetLoadedVolumeSummary(UWorld* InWorld) const
{
	FLoadedNavVolumeSummary Summary;
	TArray<AOctreeVoxelVolume*> LoadedVolumes;
	GetLoadedOctreeVolumes(InWorld, LoadedVolumes);

	const UWorldNavigationDataAsset* Manifest = LoadManifestForWorld(InWorld, false);

	Summary.Total = LoadedVolumes.Num();
	for (AOctreeVoxelVolume* Volume : LoadedVolumes)
	{
		if (!IsValid(Volume))
		{
			Summary.Invalid++;
			continue;
		}

		TArray<FNavVolumeBakeIssue> Issues;
		TArray<AOctreeVoxelVolume*> SingleVolume;
		SingleVolume.Add(Volume);
		if (!ValidateVolumesForBake(SingleVolume, Issues))
		{
			Summary.Invalid++;
		}

		const FNavigationChunkRecord* Chunk = FindChunkRecordForVolume(Manifest, Volume);
		if (!Chunk || !FindChunkPayload(Manifest, Chunk->ChunkId))
		{
			Summary.MissingBakeData++;
		}
		else if (Chunk->SourceBuildHash != BuildVolumeSourceHash(Volume))
		{
			Summary.Stale++;
		}
	}

	return Summary;
}

FNavBakeCostEstimate UPathfindingEditorSubsystem::EstimateBakeCost(UWorld* InWorld, const TArray<AOctreeVoxelVolume*>& CandidateVolumes, bool bOnlyVolumesNeedingBake) const
{
	FNavBakeCostEstimate Estimate;
	Estimate.InputVolumeCount = CandidateVolumes.Num();

	for (AOctreeVoxelVolume* Volume : CandidateVolumes)
	{
		if (!IsValid(Volume))
		{
			continue;
		}

		const bool bNeedsBake = IsVolumeBakeStale(Volume);
		if (bOnlyVolumesNeedingBake && !bNeedsBake)
		{
			Estimate.UpToDateSkipped++;
			continue;
		}

		Estimate.VolumesToBake++;

		const FIntVector MapSize = Volume->GetMapSize();
		if (MapSize.X > 0 && MapSize.Y > 0 && MapSize.Z > 0)
		{
			const int64 VolumeVoxelCount = static_cast<int64>(MapSize.X) * static_cast<int64>(MapSize.Y) * static_cast<int64>(MapSize.Z);
			Estimate.TotalVoxelCount += VolumeVoxelCount;
			Estimate.MaxVolumeVoxelCount = FMath::Max(Estimate.MaxVolumeVoxelCount, VolumeVoxelCount);

			constexpr int64 ApproxNodeAndAirwayBytesPerLeaf = 192;
			Estimate.EstimatedWorstCaseUncompressedBytes += VolumeVoxelCount * ApproxNodeAndAirwayBytesPerLeaf;
		}

		Estimate.RelevantComponentCount += CountRelevantBakeComponents(InWorld, Volume);
	}

	constexpr int64 ExpensiveVoxelThreshold = 8ll * 1024ll * 1024ll;
	constexpr int64 ExpensiveByteThreshold = 1024ll * 1024ll * 1024ll;
	constexpr int32 ExpensiveComponentThreshold = 500;
	constexpr int32 ExpensiveVolumeThreshold = 8;

	Estimate.bLikelyExpensive =
		Estimate.TotalVoxelCount >= ExpensiveVoxelThreshold ||
		Estimate.EstimatedWorstCaseUncompressedBytes >= ExpensiveByteThreshold ||
		Estimate.RelevantComponentCount >= ExpensiveComponentThreshold ||
		Estimate.VolumesToBake >= ExpensiveVolumeThreshold;

	return Estimate;
}

FText UPathfindingEditorSubsystem::FormatBakeCostEstimate(const FNavBakeCostEstimate& Estimate) const
{
	return FText::Format(
		LOCTEXT("BakeCostEstimateFormat",
			"Volumes to bake: {0} / {1}\nUp-to-date skipped: {2}\nPotential voxel budget: {3}\nRelevant overlap components: {4}\nWorst-case uncompressed nav data: {5}"),
		FText::AsNumber(Estimate.VolumesToBake),
		FText::AsNumber(Estimate.InputVolumeCount),
		FText::AsNumber(Estimate.UpToDateSkipped),
		FText::AsNumber(Estimate.TotalVoxelCount),
		FText::AsNumber(Estimate.RelevantComponentCount),
		FormatBytesForBakeEstimate(Estimate.EstimatedWorstCaseUncompressedBytes));
}

bool UPathfindingEditorSubsystem::ValidateVolumesForBake(const TArray<AOctreeVoxelVolume*>& VolumesToBake, TArray<FNavVolumeBakeIssue>& OutIssues) const
{
	OutIssues.Reset();

	for (AOctreeVoxelVolume* Volume : VolumesToBake)
	{
		if (!IsValid(Volume))
		{
			FNavVolumeBakeIssue& Issue = OutIssues.AddDefaulted_GetRef();
			Issue.Message = LOCTEXT("InvalidBakeVolume", "Volume is invalid.");
			continue;
		}

		if (Volume->VoxelExtent <= 0.f)
		{
			FNavVolumeBakeIssue& Issue = OutIssues.AddDefaulted_GetRef();
			Issue.Volume = Volume;
			Issue.Message = LOCTEXT("InvalidBakeVoxelExtent", "Voxel Extent must be greater than zero.");
		}

		const FIntVector MapSize = Volume->GetMapSize();
		if (MapSize.X <= 0 || MapSize.X != MapSize.Y || MapSize.X != MapSize.Z || !FMath::IsPowerOfTwo(static_cast<uint32>(MapSize.X)))
		{
			FNavVolumeBakeIssue& Issue = OutIssues.AddDefaulted_GetRef();
			Issue.Volume = Volume;
			Issue.Message = FText::Format(
				LOCTEXT("InvalidBakeMapSize", "Map Size {0} is not supported. Current octree bake requires a cubic power-of-two map size."),
				FText::FromString(MapSize.ToString()));
		}
	}

	return OutIssues.IsEmpty();
}

bool UPathfindingEditorSubsystem::IsVolumeBakeStale(const AOctreeVoxelVolume* Volume, TArray<FText>* OutReasons) const
{
	if (!IsValid(Volume))
	{
		if (OutReasons) OutReasons->Add(LOCTEXT("StaleInvalidVolume", "Volume is invalid."));
		return true;
	}

	const UWorldNavigationDataAsset* Manifest = LoadManifestForWorld(Volume->GetWorld(), false);
	if (!Manifest)
	{
		if (OutReasons) OutReasons->Add(LOCTEXT("StaleNoManifest", "No world navigation manifest exists for this level."));
		return true;
	}

	if (Manifest->FormatVersion < UWorldNavigationDataAsset::CurrentFormatVersion)
	{
		if (OutReasons) OutReasons->Add(LOCTEXT("StaleFormat", "The manifest was baked with an older data format."));
		return true;
	}

	const FNavigationChunkRecord* Chunk = FindChunkRecordForVolume(Manifest, Volume);
	if (!Chunk || !FindChunkPayload(Manifest, Chunk->ChunkId))
	{
		if (OutReasons) OutReasons->Add(LOCTEXT("StaleNoPayload", "The manifest has no baked payload for this volume."));
		return true;
	}

	if (Chunk->SourceBuildHash != BuildVolumeSourceHash(Volume))
	{
		if (OutReasons) OutReasons->Add(LOCTEXT("StaleHashMismatch", "Volume settings (size, voxel extent, depth, bounds, or distance field band) changed since the last bake."));
		return true;
	}

	return false;
}

void UPathfindingEditorSubsystem::BakeAndConnectVolumes_Internal(const TArray<AOctreeVoxelVolume*>& VolumesToProcess, UWorld* InWorld)
{
	if (VolumesToProcess.IsEmpty() || !InWorld)
	{
		return;
	}

	TMap<AOctreeVoxelVolume*, TArray<FOctreeNode>> BakedVolumeData;
	TMap<AOctreeVoxelVolume*, FNavDistanceField> BakedDistanceFields;
	for (AOctreeVoxelVolume* Volume : VolumesToProcess)
	{
		if (!IsValid(Volume)) continue;

		FVoxelVolumeDesc Desc;
		if (!FOctreeBaker::BuildDescFromVolume(Volume, Desc))
		{
			continue;
		}

		TArray<FOctreeNode> NodePool;
		FOctreeBaker::FVoxelField Field;
		if (FOctreeBaker::BakeVolume(InWorld, Desc, NodePool, &Field))
		{
			BakedVolumeData.Add(Volume, MoveTemp(NodePool));

			FNavDistanceField DistanceField;
			FOctreeBaker::BuildDistanceField(Desc, Field, DistanceField);
			BakedDistanceFields.Add(Volume, MoveTemp(DistanceField));
		}
	}

	// Gather every loaded volume so portals can connect into volumes that are not being re-baked.
	TArray<AOctreeVoxelVolume*> AllRelevantVolumes;
	TMap<AOctreeVoxelVolume*, TArray<FOctreeNode>> AllVolumeNodeData;
	AllRelevantVolumes.Append(VolumesToProcess);
	AllVolumeNodeData.Append(BakedVolumeData);

	const UWorldNavigationDataAsset* ExistingManifest = LoadManifestForWorld(InWorld, false);

	for (TActorIterator<AOctreeVoxelVolume> It(InWorld); It; ++It)
	{
		AOctreeVoxelVolume* ExistingVolume = *It;
		if (!IsValid(ExistingVolume) || AllRelevantVolumes.Contains(ExistingVolume))
		{
			continue;
		}

		AllRelevantVolumes.Add(ExistingVolume);

		// Load the existing volume's node data from the manifest payload, if available.
		if (const FNavigationChunkRecord* Chunk = FindChunkRecordForVolume(ExistingManifest, ExistingVolume))
		{
			TArray<FOctreeNode> NodePool;
			FAirwayGraph AirwayGraph;
			if (ExistingManifest->DecompressChunkPayload(Chunk->ChunkId, NodePool, AirwayGraph) && NodePool.Num() > 0)
			{
				AllVolumeNodeData.Add(ExistingVolume, MoveTemp(NodePool));
			}
		}
	}

	TMap<AOctreeVoxelVolume*, TMap<int32, TArray<FNavPortalBakeLink>>> AllPortalData;
	for (AOctreeVoxelVolume* Volume : VolumesToProcess)
	{
		TArray<FOctreeNode>* MyNodePool = BakedVolumeData.Find(Volume);
		if (MyNodePool)
		{
			TMap<int32, TArray<FNavPortalBakeLink>> PortalNodes;
			FOctreeBaker::ConnectPortals(InWorld, Volume, *MyNodePool, PortalNodes, AllVolumeNodeData, AllRelevantVolumes);
			AllPortalData.Add(Volume, PortalNodes);
		}
	}

	SaveBakedDataToManifest_Internal(AllRelevantVolumes, BakedVolumeData, AllPortalData, BakedDistanceFields, InWorld);
}

void UPathfindingEditorSubsystem::SaveBakedDataToManifest_Internal(
	const TArray<AOctreeVoxelVolume*>& AllRelevantVolumes,
	const TMap<AOctreeVoxelVolume*, TArray<FOctreeNode>>& BakedVolumeData,
	const TMap<AOctreeVoxelVolume*, TMap<int32, TArray<FNavPortalBakeLink>>>& PortalData,
	const TMap<AOctreeVoxelVolume*, FNavDistanceField>& DistanceFields,
	UWorld* InWorld) const
{
	if (!InWorld || BakedVolumeData.IsEmpty())
	{
		UE_LOG(LogTemp, Warning, TEXT("SaveBakedDataToManifest_Internal skipped: no baked volume data."));
		return;
	}

	UWorldNavigationDataAsset* Manifest = LoadManifestForWorld(InWorld, true);
	if (!Manifest)
	{
		UE_LOG(LogTemp, Error, TEXT("SaveBakedDataToManifest_Internal failed: could not create the world navigation manifest."));
		return;
	}

	// Snapshot the previous manifest so chunks that are not re-baked keep their payloads and portals.
	const TArray<FNavigationChunkRecord> OldChunks = Manifest->Chunks;
	const TArray<FNavigationChunkPortal> OldPortals = Manifest->Portals;
	const TArray<FNavigationChunkPayload> OldPayloads = Manifest->Payloads;

	Manifest->Modify();
	Manifest->FormatVersion = UWorldNavigationDataAsset::CurrentFormatVersion;
	Manifest->BuildGuid = FGuid::NewGuid();
	Manifest->Chunks.Reset();
	Manifest->Portals.Reset();
	Manifest->Payloads.Reset();

	TMap<AOctreeVoxelVolume*, int32> VolumeToChunkId;
	TMap<int32, int32> OldToNewChunkId;

	// Pass 1: create chunk records. Freshly baked volumes get new payloads; other volumes carry
	// their previous payloads over (re-keyed to the new chunk id).
	for (AOctreeVoxelVolume* Volume : AllRelevantVolumes)
	{
		if (!IsValid(Volume))
		{
			continue;
		}

		const TArray<FOctreeNode>* NodePool = BakedVolumeData.Find(Volume);

		const FNavigationChunkRecord* OldChunk = nullptr;
		const FNavigationChunkPayload* OldPayload = nullptr;
		{
			const FBox VolumeBounds = Volume->GetVolumeBounds();
			const FVector VolumeCenter = VolumeBounds.GetCenter();
			const FVector VolumeExtent = VolumeBounds.GetExtent();
			OldChunk = OldChunks.FindByPredicate(
				[&VolumeCenter, &VolumeExtent](const FNavigationChunkRecord& Chunk)
				{
					return Chunk.Bounds.IsInsideOrOn(VolumeCenter) && Chunk.Bounds.GetExtent().Equals(VolumeExtent, 1.0f);
				});
			if (OldChunk)
			{
				OldPayload = OldPayloads.FindByPredicate(
					[Id = OldChunk->ChunkId](const FNavigationChunkPayload& Payload)
					{
						return Payload.ChunkId == Id;
					});
			}
		}

		if (!NodePool && !OldPayload)
		{
			// Nothing baked for this volume, before or now.
			continue;
		}

		const int32 ChunkId = Manifest->Chunks.Num();
		FNavigationChunkRecord& Chunk = Manifest->Chunks.AddDefaulted_GetRef();
		Chunk.ChunkId = ChunkId;
		Chunk.ChunkCoord = EstimateChunkCoord(Volume);
		Chunk.Bounds = Volume->GetVolumeBounds();
		Chunk.PayloadMode = ENavChunkPayloadMode::Octree;
		Chunk.SourceBuildHash = NodePool ? BuildVolumeSourceHash(Volume) : (OldChunk ? OldChunk->SourceBuildHash : FString());
		Chunk.CellCount = NodePool ? NodePool->Num() : (OldChunk ? OldChunk->CellCount : 0);

		if (NodePool)
		{
			const FNavDistanceField* DistanceField = DistanceFields.Find(Volume);
			if (DistanceField && !DistanceField->IsValid())
			{
				DistanceField = nullptr;
			}

			if (Manifest->SetChunkPayloadFromNodePool(ChunkId, *NodePool, DistanceField))
			{
				Chunk.bHasDistanceField = DistanceField != nullptr;
				Chunk.DistanceFieldBandVoxels = DistanceField ? Volume->GetDistanceFieldBandVoxels() : 0;

				if (const FNavigationChunkPayload* Payload = Manifest->Payloads.FindByPredicate(
					[ChunkId](const FNavigationChunkPayload& Candidate)
					{
						return Candidate.ChunkId == ChunkId;
					}))
				{
					Chunk.PayloadCompressedSize = Payload->CompressedNodePool.Num() + Payload->CompressedAirwayGraph.Num() + Payload->CompressedDistanceField.Num();
					Chunk.PayloadUncompressedSize = static_cast<int64>(Payload->UncompressedNodePoolSize)
						+ static_cast<int64>(Payload->UncompressedAirwayGraphSize)
						+ static_cast<int64>(Payload->UncompressedDistanceFieldSize);
				}
			}
		}
		else if (OldPayload)
		{
			FNavigationChunkPayload& CarriedPayload = Manifest->Payloads.Add_GetRef(*OldPayload);
			CarriedPayload.ChunkId = ChunkId;
			Chunk.PayloadCompressedSize = OldChunk->PayloadCompressedSize;
			Chunk.PayloadUncompressedSize = OldChunk->PayloadUncompressedSize;
			Chunk.bHasDistanceField = OldChunk->bHasDistanceField;
			Chunk.DistanceFieldBandVoxels = OldChunk->DistanceFieldBandVoxels;
		}

		if (OldChunk)
		{
			OldToNewChunkId.Add(OldChunk->ChunkId, ChunkId);
		}

		VolumeToChunkId.Add(Volume, ChunkId);
	}

	// Pass 2: portals. Freshly baked volumes use the new bake links; carried-over chunks keep
	// their previous portals with remapped chunk ids.
	for (AOctreeVoxelVolume* Volume : AllRelevantVolumes)
	{
		const int32* SourceChunkIdPtr = VolumeToChunkId.Find(Volume);
		if (!SourceChunkIdPtr || !Manifest->Chunks.IsValidIndex(*SourceChunkIdPtr))
		{
			continue;
		}

		FNavigationChunkRecord& SourceChunk = Manifest->Chunks[*SourceChunkIdPtr];
		SourceChunk.PortalStartIndex = Manifest->Portals.Num();

		if (const TMap<int32, TArray<FNavPortalBakeLink>>* SourcePortals = PortalData.Find(Volume))
		{
			for (const TPair<int32, TArray<FNavPortalBakeLink>>& PortalPair : *SourcePortals)
			{
				for (const FNavPortalBakeLink& Link : PortalPair.Value)
				{
					AOctreeVoxelVolume* TargetVolume = Link.TargetVolume.Get();
					const int32* TargetChunkIdPtr = TargetVolume ? VolumeToChunkId.Find(TargetVolume) : nullptr;
					if (!TargetChunkIdPtr)
					{
						continue;
					}

					FNavigationChunkPortal& Portal = Manifest->Portals.AddDefaulted_GetRef();
					Portal.SourceChunkId = *SourceChunkIdPtr;
					Portal.SourceCellId = PortalPair.Key;
					Portal.TargetChunkId = *TargetChunkIdPtr;
					Portal.TargetCellId = Link.TargetNodeIndex;
					Portal.MaxAllowedRadius = Link.MaxAllowedRadius;
				}
			}
		}
		else
		{
			// Carried-over chunk: remap its previous portals onto the new chunk ids.
			const FBox VolumeBounds = Volume->GetVolumeBounds();
			const FVector VolumeCenter = VolumeBounds.GetCenter();
			const FVector VolumeExtent = VolumeBounds.GetExtent();
			const FNavigationChunkRecord* OldChunk = OldChunks.FindByPredicate(
				[&VolumeCenter, &VolumeExtent](const FNavigationChunkRecord& Chunk)
				{
					return Chunk.Bounds.IsInsideOrOn(VolumeCenter) && Chunk.Bounds.GetExtent().Equals(VolumeExtent, 1.0f);
				});

			if (OldChunk)
			{
				for (int32 PortalIndex = 0; PortalIndex < OldChunk->PortalCount; ++PortalIndex)
				{
					const int32 OldPortalIndex = OldChunk->PortalStartIndex + PortalIndex;
					if (!OldPortals.IsValidIndex(OldPortalIndex))
					{
						continue;
					}

					const FNavigationChunkPortal& OldPortal = OldPortals[OldPortalIndex];
					const int32* NewTargetId = OldToNewChunkId.Find(OldPortal.TargetChunkId);
					if (!NewTargetId)
					{
						continue;
					}

					FNavigationChunkPortal& Portal = Manifest->Portals.Add_GetRef(OldPortal);
					Portal.SourceChunkId = *SourceChunkIdPtr;
					Portal.TargetChunkId = *NewTargetId;
				}
			}
		}

		SourceChunk.PortalCount = Manifest->Portals.Num() - SourceChunk.PortalStartIndex;
	}

	Manifest->MarkPackageDirty();
	if (UPackage* ManifestPackage = Manifest->GetPackage())
	{
		TArray<UPackage*> PackagesToSave;
		PackagesToSave.Add(ManifestPackage);
		UEditorLoadingAndSavingUtils::SavePackages(PackagesToSave, true);
	}

	UE_LOG(LogTemp, Log, TEXT("Manifest bake saved with %d chunks, %d payloads, and %d portals."),
		Manifest->Chunks.Num(), Manifest->Payloads.Num(), Manifest->Portals.Num());
}

int32 UPathfindingEditorSubsystem::CountRelevantBakeComponents(UWorld* InWorld, const AOctreeVoxelVolume* Volume) const
{
	if (!InWorld || !IsValid(Volume))
	{
		return 0;
	}

	const FBox VolumeBounds = Volume->GetVolumeBounds();
	if (!VolumeBounds.IsValid)
	{
		return 0;
	}

	FCollisionObjectQueryParams ObjectQueryParams;
	bool bAddedObjectChannel = false;
	if (const UVoxelObstacleProfile* StaticProfile = Volume->GetStaticObstacleProfile())
	{
		if (StaticProfile->CollisionChannels.Num() > 0)
		{
			for (const TEnumAsByte<ECollisionChannel>& Channel : StaticProfile->CollisionChannels)
			{
				ObjectQueryParams.AddObjectTypesToQuery(Channel);
				bAddedObjectChannel = true;
			}
		}
	}
	else if (Volume->BakeCollisionChannels.Num() > 0)
	{
		for (const TEnumAsByte<ECollisionChannel>& Channel : Volume->BakeCollisionChannels)
		{
			ObjectQueryParams.AddObjectTypesToQuery(Channel);
			bAddedObjectChannel = true;
		}
	}

	if (!bAddedObjectChannel)
	{
		ObjectQueryParams.AddObjectTypesToQuery(ECC_WorldStatic);
	}

	FCollisionQueryParams QueryParams(SCENE_QUERY_STAT(NavBakeCostEstimateOverlap), false);
	QueryParams.AddIgnoredActor(Volume);

	TArray<FOverlapResult> OverlapResults;
	InWorld->OverlapMultiByObjectType(
		OverlapResults,
		VolumeBounds.GetCenter(),
		FQuat::Identity,
		ObjectQueryParams,
		FCollisionShape::MakeBox(VolumeBounds.GetExtent()),
		QueryParams);

	TSet<UPrimitiveComponent*> UniqueComponents;
	for (const FOverlapResult& Result : OverlapResults)
	{
		UPrimitiveComponent* PrimitiveComp = Result.GetComponent();
		if (!IsValid(PrimitiveComp) || !PrimitiveComp->IsQueryCollisionEnabled())
		{
			continue;
		}

		AActor* OwnerActor = PrimitiveComp->GetOwner();
		if (!OwnerActor)
		{
			continue;
		}

		if (OwnerActor->ActorHasTag(FName("NoNavBake")) || OwnerActor->FindComponentByClass<UNavBakeExcludeComponent>())
		{
			continue;
		}

#if WITH_EDITORONLY_DATA
		if (Volume->BakeExcludeActors.Contains(OwnerActor))
		{
			continue;
		}
#endif

		if (Volume->bRespectCanAffectNavigation && !PrimitiveComp->CanEverAffectNavigation())
		{
			continue;
		}

		if (const UVoxelObstacleProfile* StaticProfile = Volume->GetStaticObstacleProfile())
		{
			if (!StaticProfile->DoesActorMatch(OwnerActor, PrimitiveComp))
			{
				continue;
			}
		}

		UniqueComponents.Add(PrimitiveComp);
	}

	return UniqueComponents.Num();
}

bool UPathfindingEditorSubsystem::HasManifestPayloadForVolume(UWorld* InWorld, const AOctreeVoxelVolume* Volume) const
{
	if (!InWorld || !IsValid(Volume))
	{
		return false;
	}

	const UWorldNavigationDataAsset* Manifest = LoadManifestForWorld(InWorld, false);
	const FNavigationChunkRecord* Chunk = FindChunkRecordForVolume(Manifest, Volume);
	return Chunk && FindChunkPayload(Manifest, Chunk->ChunkId) != nullptr;
}

#endif // WITH_EDITOR

#undef LOCTEXT_NAMESPACE
