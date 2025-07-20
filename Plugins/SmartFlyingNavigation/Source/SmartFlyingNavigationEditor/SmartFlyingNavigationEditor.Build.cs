using UnrealBuildTool;

public class SmartFlyingNavigationEditor : ModuleRules
{
	public SmartFlyingNavigationEditor(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
		
		PublicDependencyModuleNames.AddRange(
			new string[]
			{
				"Core",
			}
		);  
		
		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				"CoreUObject",
				"Engine",
				"UnrealEd",
				"Slate",
				"SlateCore",
				"AssetTools",
				"LevelEditor",    
				"EditorStyle",
				"WorkspaceMenuStructure",
				"EditorSubsystem",
				"LevelEditor",
				"PropertyEditor", 
				"DesktopWidgets",
				"DesktopPlatform",
				"EditorScriptingUtilities",
				"Projects", "SmartFlyingNavigation", 
				"SmartFlyingNavigation",
				"InputCore"    
			}
		);
	}
}