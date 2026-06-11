import unreal
les = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
assert les.load_level("/SmartFlyingNavigation/Maps/PathfinderExampleMap/MazeObstacle_Demo"), "load failed"
world = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
unreal.SystemLibrary.execute_console_command(world, "sfn.BakeNav")
print("BAKE_TRIGGERED")
