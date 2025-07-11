
#include "GameFramework/Actor.h"
#include "ChunkDataHolder.generated.h"

class UStaticVoxelChunkData;
class AFenwickVoxelVolume;

UCLASS()
class NAVIGATION3D_API AChunkDataHolder : public AActor
{
	GENERATED_BODY()
public:
	AChunkDataHolder();
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Voxel")
	FIntVector ChunkCoords;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Voxel")
	TObjectPtr<UStaticVoxelChunkData> VoxelChunkData;

	// 이 홀더가 속한 주 볼륨
	UPROPERTY()
	TWeakObjectPtr<AFenwickVoxelVolume> OwningVolume;

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
};