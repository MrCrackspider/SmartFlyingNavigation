// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class EpicProject_3DNavi : ModuleRules
{
	public EpicProject_3DNavi(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
	
		PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore" });

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
		
		// Uncomment if you are using online features
		// PrivateDependencyModuleNames.Add("OnlineSubsystem");

		// To include OnlineSubsystemSteam, add it to the plugins section in your uproject file with the Enabled attribute set to true
	}
}
