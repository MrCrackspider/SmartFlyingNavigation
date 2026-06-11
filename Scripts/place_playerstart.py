"""Places (or moves) a PlayerStart near the first agent in MazeObstacle_Demo.
Run with: UnrealEditor-Cmd.exe <project> -run=pythonscript -script=<this file>
"""
import unreal

LEVEL = "/SmartFlyingNavigation/Maps/PathfinderExampleMap/MazeObstacle_Demo"

les = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
if not les.load_level(LEVEL):
    raise SystemExit("PLAYERSTART_FAIL: could not load level")

actor_sub = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
actors = actor_sub.get_all_level_actors()

agent = None
volume = None
for a in actors:
    cls = a.get_class().get_name()
    if agent is None and cls.startswith("BP_Agent"):
        agent = a
    if volume is None and "OctreeVoxelVolume" in cls:
        volume = a

if agent:
    loc = agent.get_actor_location()
    target = unreal.Vector(loc.x - 800.0, loc.y, loc.z + 300.0)
    look = unreal.MathLibrary.find_look_at_rotation(target, loc)
elif volume:
    target = volume.get_actor_location()
    look = unreal.Rotator(0.0, 0.0, 0.0)
else:
    raise SystemExit("PLAYERSTART_FAIL: no agent or volume found")

starts = [a for a in actors if isinstance(a, unreal.PlayerStart)]
if starts:
    starts[0].set_actor_location(target, False, False)
    starts[0].set_actor_rotation(look, False)
    print("PLAYERSTART_MOVED to %s" % target)
else:
    spawned = actor_sub.spawn_actor_from_class(unreal.PlayerStart, target, look)
    print("PLAYERSTART_SPAWNED %s at %s" % (spawned.get_name(), target))

les.save_current_level()
print("PLAYERSTART_OK")
