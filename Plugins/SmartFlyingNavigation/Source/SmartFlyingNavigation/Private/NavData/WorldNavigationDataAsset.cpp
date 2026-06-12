#include "NavData/WorldNavigationDataAsset.h"
#include "NavCore/NavDistanceField.h"
#include "Algo/Unique.h"
#include "Misc/Compression.h"
#include "Serialization/MemoryReader.h"
#include "Serialization/MemoryWriter.h"

const FName UWorldNavigationDataAsset::WorldNavigationAssetType = TEXT("WorldNavigationData");

namespace
{
	const FName GNavPayloadCompressionFormat(TEXT("Zlib"));

	bool CompressBuffer(const TArray<uint8>& RawData, TArray<uint8>& OutCompressed, int32& OutUncompressedSize)
	{
		OutCompressed.Empty();
		OutUncompressedSize = RawData.Num();

		if (RawData.Num() == 0)
		{
			return true;
		}

		int32 MaxCompressedSize = FCompression::CompressMemoryBound(GNavPayloadCompressionFormat, RawData.Num());
		OutCompressed.SetNumUninitialized(MaxCompressedSize);

		int32 ActualCompressedSize = MaxCompressedSize;
		if (!FCompression::CompressMemory(GNavPayloadCompressionFormat, OutCompressed.GetData(), ActualCompressedSize, RawData.GetData(), RawData.Num()))
		{
			OutCompressed.Empty();
			OutUncompressedSize = 0;
			return false;
		}

		OutCompressed.SetNum(ActualCompressedSize);
		return true;
	}

	bool DecompressBuffer(const TArray<uint8>& Compressed, int32 UncompressedSize, TArray<uint8>& OutRawData)
	{
		OutRawData.Reset();
		if (Compressed.Num() == 0 || UncompressedSize <= 0)
		{
			return false;
		}

		OutRawData.SetNumUninitialized(UncompressedSize);
		return FCompression::UncompressMemory(
			GNavPayloadCompressionFormat,
			OutRawData.GetData(),
			UncompressedSize,
			Compressed.GetData(),
			Compressed.Num());
	}
}

FPrimaryAssetId UWorldNavigationDataAsset::GetPrimaryAssetId() const
{
	return FPrimaryAssetId(WorldNavigationAssetType, GetFName());
}

bool UWorldNavigationDataAsset::IsFormatSupported() const
{
	if (FormatVersion < CurrentFormatVersion)
	{
		UE_LOG(LogTemp, Error,
			TEXT("WorldNavigationDataAsset '%s' uses format version %d but this plugin requires %d. Re-bake the navigation data."),
			*GetName(), FormatVersion, CurrentFormatVersion);
		return false;
	}
	return true;
}

bool UWorldNavigationDataAsset::DecompressChunkPayload(int32 ChunkId, TArray<FOctreeNode>& OutNodePool, FAirwayGraph& OutAirwayGraph,
                                                       FNavDistanceField* OutDistanceField) const
{
	OutNodePool.Reset();
	OutAirwayGraph.Reset();
	if (OutDistanceField)
	{
		OutDistanceField->Reset();
	}

	const FNavigationChunkPayload* Payload = Payloads.FindByPredicate(
		[ChunkId](const FNavigationChunkPayload& Candidate)
		{
			return Candidate.ChunkId == ChunkId;
		});

	if (!Payload)
	{
		return false;
	}

	if (Payload->CompressedNodePool.Num() > 0)
	{
		TArray<uint8> RawNodeData;
		if (!DecompressBuffer(Payload->CompressedNodePool, Payload->UncompressedNodePoolSize, RawNodeData))
		{
			UE_LOG(LogTemp, Error, TEXT("WorldNavigationDataAsset: Failed to decompress node payload for ChunkId=%d."), ChunkId);
			return false;
		}

		FMemoryReader NodeReader(RawNodeData, true);
		NodeReader << OutNodePool;
	}

	// v6 changed the airway stream to undirected half-edges; older streams would
	// decode into a silently corrupt graph, so they are skipped (node pools are
	// still readable — the editor reuses them for portal connections on rebake).
	if (Payload->CompressedAirwayGraph.Num() > 0 && FormatVersion >= CurrentFormatVersion)
	{
		TArray<uint8> RawAirwayData;
		if (!DecompressBuffer(Payload->CompressedAirwayGraph, Payload->UncompressedAirwayGraphSize, RawAirwayData))
		{
			UE_LOG(LogTemp, Error, TEXT("WorldNavigationDataAsset: Failed to decompress airway payload for ChunkId=%d."), ChunkId);
			return false;
		}

		FMemoryReader AirwayReader(RawAirwayData, true);
		AirwayReader << OutAirwayGraph;
	}

	if (OutDistanceField && Payload->CompressedDistanceField.Num() > 0)
	{
		TArray<uint8> RawDistanceData;
		if (DecompressBuffer(Payload->CompressedDistanceField, Payload->UncompressedDistanceFieldSize, RawDistanceData))
		{
			FMemoryReader DistanceReader(RawDistanceData, true);
			DistanceReader << *OutDistanceField;
		}
		else
		{
			UE_LOG(LogTemp, Error, TEXT("WorldNavigationDataAsset: Failed to decompress distance field for ChunkId=%d."), ChunkId);
			OutDistanceField->Reset();
		}
	}

	return OutNodePool.Num() > 0 || !OutAirwayGraph.IsEmpty();
}

#if WITH_EDITOR
bool UWorldNavigationDataAsset::SetChunkPayloadFromNodePool(int32 ChunkId, const TArray<FOctreeNode>& InNodePool, const FNavDistanceField* InDistanceField)
{
	FNavigationChunkPayload* Payload = Payloads.FindByPredicate(
		[ChunkId](const FNavigationChunkPayload& Candidate)
		{
			return Candidate.ChunkId == ChunkId;
		});

	if (!Payload)
	{
		Payload = &Payloads.AddDefaulted_GetRef();
		Payload->ChunkId = ChunkId;
	}

	TArray<uint8> RawNodeData;
	FMemoryWriter NodeWriter(RawNodeData, true);
	NodeWriter << const_cast<TArray<FOctreeNode>&>(InNodePool);

	if (!CompressBuffer(RawNodeData, Payload->CompressedNodePool, Payload->UncompressedNodePoolSize))
	{
		UE_LOG(LogTemp, Error, TEXT("WorldNavigationDataAsset: Failed to compress node payload for ChunkId=%d."), ChunkId);
		return false;
	}

	// --- Build the CSR airway graph (centers of empty leaves) from the node pool. ---
	FAirwayGraph BuiltAirwayGraph;
	{
		// Lattice: every leaf center lies on the half-voxel lattice anchored at the chunk origin.
		FVector OriginWS = InNodePool.Num() > 0 ? InNodePool[0].Bounds.Min : FVector::ZeroVector;
		float MinLeafSize = TNumericLimits<float>::Max();
		for (const FOctreeNode& Node : InNodePool)
		{
			if (Node.State != EOctreeNodeState::Mixed)
			{
				MinLeafSize = FMath::Min(MinLeafSize, static_cast<float>(Node.Bounds.GetSize().X));
			}
		}
		const float HalfVoxel = MinLeafSize < TNumericLimits<float>::Max() ? MinLeafSize * 0.5f : 50.f;

		BuiltAirwayGraph.OriginWS = FVector3f(OriginWS);
		BuiltAirwayGraph.HalfVoxelSize = HalfVoxel;

		TArray<int32> NodeToGraphIndex;
		NodeToGraphIndex.Init(-1, InNodePool.Num());

		for (int32 NodeIndex = 0; NodeIndex < InNodePool.Num(); ++NodeIndex)
		{
			const FOctreeNode& SourceNode = InNodePool[NodeIndex];
			if (SourceNode.State != EOctreeNodeState::Empty)
			{
				continue;
			}

			const FVector Local = (SourceNode.Bounds.GetCenter() - OriginWS) / HalfVoxel;
			FAirwayGraph::FCompactNode CompactNode;
			CompactNode.X = static_cast<uint16>(FMath::Clamp(FMath::RoundToInt(Local.X), 0, 65535));
			CompactNode.Y = static_cast<uint16>(FMath::Clamp(FMath::RoundToInt(Local.Y), 0, 65535));
			CompactNode.Z = static_cast<uint16>(FMath::Clamp(FMath::RoundToInt(Local.Z), 0, 65535));
			CompactNode.ClearanceQ = FAirwayGraph::QuantizeClearance(SourceNode.MaxAllowedRadius);
			NodeToGraphIndex[NodeIndex] = BuiltAirwayGraph.Nodes.Add(CompactNode);
		}

		// Octree neighbor links can be asymmetric across leaf sizes (a coarse leaf
		// stores only one neighbor per face), so adjacency is collected as unordered
		// pairs and both directions are emitted from each pair. This also makes the
		// graph safe to serialize as undirected half-edges.
		TArray<uint64> EdgePairs;
		for (int32 NodeIndex = 0; NodeIndex < InNodePool.Num(); ++NodeIndex)
		{
			const int32 SourceGraphIndex = NodeToGraphIndex[NodeIndex];
			if (SourceGraphIndex == -1)
			{
				continue;
			}

			const FOctreeNode& SourceNode = InNodePool[NodeIndex];
			for (int32 DirIndex = 0; DirIndex < static_cast<int32>(ENeighborDirection::MAX); ++DirIndex)
			{
				const int32 NeighborNodeIndex = SourceNode.Neighbors[DirIndex];
				if (!NodeToGraphIndex.IsValidIndex(NeighborNodeIndex) || NodeToGraphIndex[NeighborNodeIndex] == -1)
				{
					continue;
				}

				const int32 TargetGraphIndex = NodeToGraphIndex[NeighborNodeIndex];
				if (TargetGraphIndex == SourceGraphIndex)
				{
					continue;
				}

				const uint32 Low = static_cast<uint32>(FMath::Min(SourceGraphIndex, TargetGraphIndex));
				const uint32 High = static_cast<uint32>(FMath::Max(SourceGraphIndex, TargetGraphIndex));
				EdgePairs.Add((static_cast<uint64>(Low) << 32) | High);
			}
		}

		EdgePairs.Sort();
		EdgePairs.SetNum(Algo::Unique(EdgePairs));

		// Edge clearance is the min of both endpoint clearances; node ClearanceQ uses
		// the same floor quantizer, so min in quantized space matches the float min.
		TArray<uint32> Degree;
		Degree.SetNumZeroed(BuiltAirwayGraph.Nodes.Num());
		for (const uint64 Pair : EdgePairs)
		{
			++Degree[static_cast<int32>(Pair >> 32)];
			++Degree[static_cast<int32>(Pair & 0xFFFFFFFFu)];
		}

		BuiltAirwayGraph.AdjOffsets.SetNumUninitialized(BuiltAirwayGraph.Nodes.Num() + 1);
		BuiltAirwayGraph.AdjOffsets[0] = 0;
		for (int32 Index = 0; Index < BuiltAirwayGraph.Nodes.Num(); ++Index)
		{
			BuiltAirwayGraph.AdjOffsets[Index + 1] = BuiltAirwayGraph.AdjOffsets[Index] + Degree[Index];
		}

		TArray<uint32>& Cursor = Degree;
		FMemory::Memcpy(Cursor.GetData(), BuiltAirwayGraph.AdjOffsets.GetData(), BuiltAirwayGraph.Nodes.Num() * sizeof(uint32));

		BuiltAirwayGraph.Edges.SetNumUninitialized(2 * EdgePairs.Num());
		for (const uint64 Pair : EdgePairs)
		{
			const int32 Low = static_cast<int32>(Pair >> 32);
			const int32 High = static_cast<int32>(Pair & 0xFFFFFFFFu);
			const uint32 ClearanceBits = static_cast<uint32>(FMath::Min(
				BuiltAirwayGraph.Nodes[Low].ClearanceQ, BuiltAirwayGraph.Nodes[High].ClearanceQ)) << 24;
			BuiltAirwayGraph.Edges[Cursor[Low]++] = ClearanceBits | static_cast<uint32>(High);
			BuiltAirwayGraph.Edges[Cursor[High]++] = ClearanceBits | static_cast<uint32>(Low);
		}

		UE_LOG(LogTemp, Log, TEXT("WorldNavigationDataAsset: ChunkId=%d CSR airway graph: %d nodes, %d undirected edges (%d directed), %.2f MB in memory."),
			ChunkId, BuiltAirwayGraph.Num(), EdgePairs.Num(), BuiltAirwayGraph.Edges.Num(),
			BuiltAirwayGraph.GetAllocatedBytes() / (1024.0 * 1024.0));
	}
	TArray<uint8> RawAirwayData;
	FMemoryWriter AirwayWriter(RawAirwayData, true);
	AirwayWriter << BuiltAirwayGraph;

	if (!CompressBuffer(RawAirwayData, Payload->CompressedAirwayGraph, Payload->UncompressedAirwayGraphSize))
	{
		UE_LOG(LogTemp, Error, TEXT("WorldNavigationDataAsset: Failed to compress airway payload for ChunkId=%d."), ChunkId);
		return false;
	}

	Payload->CompressedDistanceField.Empty();
	Payload->UncompressedDistanceFieldSize = 0;

	if (InDistanceField && InDistanceField->IsValid())
	{
		TArray<uint8> RawDistanceData;
		FMemoryWriter DistanceWriter(RawDistanceData, true);
		DistanceWriter << const_cast<FNavDistanceField&>(*InDistanceField);

		if (!CompressBuffer(RawDistanceData, Payload->CompressedDistanceField, Payload->UncompressedDistanceFieldSize))
		{
			UE_LOG(LogTemp, Error, TEXT("WorldNavigationDataAsset: Failed to compress distance field for ChunkId=%d."), ChunkId);
			return false;
		}
	}

	return true;
}
#endif
