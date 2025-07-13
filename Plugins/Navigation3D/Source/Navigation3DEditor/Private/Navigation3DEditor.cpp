#include "Navigation3DEditor.h"
#include "PropertyEditorModule.h"
#include "Pathfinder/PathfindingComponent.h"
#include "Nav3DPathfindingDetails.h"
#include "Nav3DVolumeDetails.h"

#define LOCTEXT_NAMESPACE "FNavigation3DEditorModule"

void FNavigation3DEditorModule::StartupModule()
{
	//모듈 로드 실패시 크래시 발생시킴.
	FPropertyEditorModule& PropertyModule = FModuleManager::LoadModuleChecked<FPropertyEditorModule>("PropertyEditor");

	//커스텀 디테일 레이아웃 등록
	PropertyModule.RegisterCustomClassLayout(
		UPathfindingComponent::StaticClass()->GetFName(),
		FOnGetDetailCustomizationInstance::CreateStatic(&FNav3DPathfindingDetails::MakeInstance)
	);

	PropertyModule.RegisterCustomClassLayout(
		AFenwickVoxelVolume::StaticClass()->GetFName(),
		FOnGetDetailCustomizationInstance::CreateStatic(&FNav3DVolumeDetails::MakeInstance)
	);
	
	UE_LOG(LogTemp, Log, TEXT("Navigation3DEditor Module has started. Custom details for UPathfindingComponent registered"));
}

void FNavigation3DEditorModule::ShutdownModule()
{
	//언로드 하기 전 등록했던 커스터마이징을 반드시 해제
	if (FModuleManager::Get().IsModuleLoaded("PropertyEditor"))
	{
		FPropertyEditorModule& PropertyModule = FModuleManager::LoadModuleChecked<FPropertyEditorModule>("PropertyEditor");

		//클래스에 등록했던 커스텀 레이아웃 해제
		PropertyModule.UnregisterCustomClassLayout(UPathfindingComponent::StaticClass()->GetFName());
		PropertyModule.UnregisterCustomClassLayout(AFenwickVoxelVolume::StaticClass()->GetFName());
	}

	UE_LOG(LogTemp, Log, TEXT("Navigation3DEditor Module has shutdown"));
}
#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FNavigation3DEditorModule, Navigation3DEditor)