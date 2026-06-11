// WorldNavigationDataAsset.h

#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "NavCore/NavNodeTypes.h"
#include "WorldNavigationDataAsset.generated.h"

struct FNavDistanceField;

UENUM(BlueprintType)
enum class ENavChunkPayloadMode : uint8
{
	Octree,
	CellPortalGraph,
	AirwayGraph
};

USTRUCT(BlueprintType)
struct FNavigationChunkPortal
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, Category = "Navigation")
	int32 SourceChunkId = INDEX_NONE;

	UPROPERTY(VisibleAnywhere, Category = "Navigation")
	int32 SourceCellId = INDEX_NONE;

	UPROPERTY(VisibleAnywhere, Category = "Navigation")
	int32 TargetChunkId = INDEX_NONE;

	UPROPERTY(VisibleAnywhere, Category = "Navigation")
	int32 TargetCellId = INDEX_NONE;

	UPROPERTY(VisibleAnywhere, Category = "Navigation")
	float MaxAllowedRadius = 0.f;
};

USTRUCT(BlueprintType)
struct FNavigationChunkRecord
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, Category = "Navigation")
	int32 ChunkId = INDEX_NONE;

	UPROPERTY(VisibleAnywhere, Category = "Navigation")
	FIntVector ChunkCoord = FIntVector::ZeroValue;

	UPROPERTY(VisibleAnywhere, Category = "Navigation")
	FBox Bounds;

	UPROPERTY(VisibleAnywhere, Category = "Navigation")
	ENavChunkPayloadMode PayloadMode = ENavChunkPayloadMode::Octree;

	UPROPERTY(VisibleAnywhere, Category = "Navigation")
	FString SourceBuildHash;

	UPROPERTY(VisibleAnywhere, Category = "Navigation")
	int32 CellCount = 0;

	UPROPERTY(VisibleAnywhere, Category = "Navigation")
	int32 PortalStartIndex = 0;

	UPROPERTY(VisibleAnywhere, Category = "Navigation")
	int32 PortalCount = 0;

	UPROPERTY(VisibleAnywhere, Category = "Navigation")
	int64 PayloadOffset = 0;

	UPROPERTY(VisibleAnywhere, Category = "Navigation")
	int64 PayloadCompressedSize = 0;

	UPROPERTY(VisibleAnywhere, Category = "Navigation")
	int64 PayloadUncompressedSize = 0;

	UPROPERTY(VisibleAnywhere, Category = "Navigation")
	bool bHasDistanceField = false;

	UPROPERTY(VisibleAnywhere, Category = "Navigation")
	int32 DistanceFieldBandVoxels = 0;
};

USTRUCT(BlueprintType)
struct FNavigationChunkPayload
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, Category = "Navigation")
	int32 ChunkId = INDEX_NONE;

	UPROPERTY()
	TArray<uint8> CompressedNodePool;

	UPROPERTY()
	int32 UncompressedNodePoolSize = 0;

	UPROPERTY()
	TArray<uint8> CompressedAirwayGraph;

	UPROPERTY()
	int32 UncompressedAirwayGraphSize = 0;

	UPROPERTY()
	TArray<uint8> CompressedDistanceField;

	UPROPERTY()
	int32 UncompressedDistanceFieldSize = 0;
};

/**
 * The single baked navigation artifact: a world-wide manifest of navigation chunks,
 * their portals, and compressed payloads (octree node pools + airway graphs).
 */
UCLASS(BlueprintType)
class SMARTFLYINGNAVIGATION_API UWorldNavigationDataAsset : public UPrimaryDataAsset
{
	GENERATED_BODY()

public:
	static const FName WorldNavigationAssetType;

	/** Format version this plugin reads and writes. Older assets are rejected and must be re-baked.
	 *  v5: CSR airway graph (8 B nodes on the half-voxel lattice, 4 B edges), mixed 4/8-bit SDF bricks. */
	static constexpr int32 CurrentFormatVersion = 5;

	UPROPERTY(VisibleAnywhere, Category = "Navigation")
	int32 FormatVersion = CurrentFormatVersion;

	UPROPERTY(VisibleAnywhere, Category = "Navigation")
	FGuid BuildGuid;

	UPROPERTY(VisibleAnywhere, Category = "Navigation|Chunks")
	TArray<FNavigationChunkRecord> Chunks;

	UPROPERTY(VisibleAnywhere, Category = "Navigation|Chunks")
	TArray<FNavigationChunkPortal> Portals;

	UPROPERTY()
	TArray<FNavigationChunkPayload> Payloads;

	/** Returns false (with a log) when the asset was baked with an older, unsupported format. */
	bool IsFormatSupported() const;

	bool DecompressChunkPayload(int32 ChunkId, TArray<FOctreeNode>& OutNodePool, FAirwayGraph& OutAirwayGraph,
	                            FNavDistanceField* OutDistanceField = nullptr) const;

#if WITH_EDITOR
	bool SetChunkPayloadFromNodePool(int32 ChunkId, const TArray<FOctreeNode>& InNodePool, const FNavDistanceField* InDistanceField = nullptr);
#endif

	virtual FPrimaryAssetId GetPrimaryAssetId() const override;
};
