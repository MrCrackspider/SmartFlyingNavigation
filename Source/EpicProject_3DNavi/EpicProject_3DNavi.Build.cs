// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class EpicProject_3DNavi : ModuleRules
{
	public EpicProject_3DNavi(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
	
		PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore","AssetRegistry" });

		PrivateDependencyModuleNames.AddRange(new string[] {  });

		if (Target.bBuildEditor == true)
		{
			// 여기에 필요한 모듈들을 추가합니다.
			PrivateDependencyModuleNames.AddRange(
				new string[]
				{
					"UnrealEd",      // UEditorLoadingAndSavingUtils, UDataAssetFactory 등 에디터 핵심 기능
					"AssetTools"     // FAssetToolsModule, 에셋 생성/관리 기능
				}
			);
		}
		// Uncomment if you are using Slate UI
		// PrivateDependencyModuleNames.AddRange(new string[] { "Slate", "SlateCore" });
		
		if (Target.bBuildEditor == true)
		{
			// 여기에 필요한 모듈들을 추가합니다.
			PrivateDependencyModuleNames.AddRange(
				new string[]
				{
					"UnrealEd",      // UEditorLoadingAndSavingUtils, UDataAssetFactory 등 에디터 핵심 기능
					"AssetTools",    // FAssetToolsModule, 에셋 생성/관리 기능
					"Slate",         // Slate UI 프레임워크
					"SlateCore",     // Slate의 핵심 타입 및 구조
					"EditorStyle",   // 에디터의 기본 스타일(색상, 폰트 등)
					"LevelEditor"    // 메뉴 등록 및 탭 스포너에 필요
				}
			);
		}
		
		// Uncomment if you are using online features
		// PrivateDependencyModuleNames.Add("OnlineSubsystem");

		// To include OnlineSubsystemSteam, add it to the plugins section in your uproject file with the Enabled attribute set to true
	}
	
}
