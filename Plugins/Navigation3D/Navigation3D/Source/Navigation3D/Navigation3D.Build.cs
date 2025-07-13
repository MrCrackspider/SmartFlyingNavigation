// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class Navigation3D : ModuleRules
{
	public Navigation3D(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;
		
		PublicIncludePaths.AddRange(
			new string[] {
				// ... add public include paths required here ...
			}
			);
				
		
		PrivateIncludePaths.AddRange(
			new string[] {
				// ... add other private include paths required here ...
			}
			);
			
		
		PublicDependencyModuleNames.AddRange(
			new string[]
			{
				"Core",
				// ... add other public dependencies that you statically link with here ...
			}
			);
			
		
		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				"CoreUObject",
				"Engine",
				"Slate",
				"SlateCore",
				"UnrealEd",
				"AssetTools",     // FAssetToolsModule, 에셋 생성/관리 기능
				// ... add private dependencies that you statically link with here ...	
				
				"LevelEditor",    // 메뉴 등록 및 탭 스포너에 필요
				"EditorStyle",
				// ▼▼▼▼▼ 이 모듈을 반드시 추가해야 합니다. ▼▼▼▼▼
				"WorkspaceMenuStructure",// FEditorStyle을 사용하기 위해 필요
				"InputCore"
			}
			);
		
		
		DynamicallyLoadedModuleNames.AddRange(
			new string[]
			{
				// ... add any modules that your module loads dynamically here ...
			}
			);
	}
}
