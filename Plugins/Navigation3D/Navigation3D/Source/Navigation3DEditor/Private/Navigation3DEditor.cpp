#include "Navigation3DEditor.h"
#include "PropertyEditorModule.h"
#include "Pathfinder/PathfindingComponent.h" 
#include "Pathfinder/NavVolume/OctreeVoxelVolume.h" 
#include "Nav3DPathfindingDetails.h"
#include "Nav3DVolumeDetails.h"

#define LOCTEXT_NAMESPACE "FNavigation3DEditorModule"

void FNavigation3DEditorModule::StartupModule()
{
	FPropertyEditorModule& PropertyModule = FModuleManager::LoadModuleChecked<FPropertyEditorModule>("PropertyEditor");

	PropertyModule.RegisterCustomClassLayout(
		UPathfindingComponent::StaticClass()->GetFName(),
		FOnGetDetailCustomizationInstance::CreateStatic(&FNav3DPathfindingDetails::MakeInstance)
	);

	PropertyModule.RegisterCustomClassLayout(
		AOctreeVoxelVolume::StaticClass()->GetFName(),
		FOnGetDetailCustomizationInstance::CreateStatic(&FNav3DVolumeDetails::MakeInstance)
	);
	
	UE_LOG(LogTemp, Log, TEXT("Navigation3DEditor Module has started. Custom details for UPathfindingComponent registered"));
}

void FNavigation3DEditorModule::ShutdownModule()
{
	if (FModuleManager::Get().IsModuleLoaded("PropertyEditor"))
	{
		FPropertyEditorModule& PropertyModule = FModuleManager::LoadModuleChecked<FPropertyEditorModule>("PropertyEditor");

		PropertyModule.UnregisterCustomClassLayout(UPathfindingComponent::StaticClass()->GetFName());
		PropertyModule.UnregisterCustomClassLayout(AOctreeVoxelVolume::StaticClass()->GetFName());
	}

	UE_LOG(LogTemp, Log, TEXT("Navigation3DEditor Module has shutdown"));
}
#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FNavigation3DEditorModule, Navigation3DEditor)