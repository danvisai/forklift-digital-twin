using UnrealBuildTool;

public class ForkliftTwin : ModuleRules
{
	public ForkliftTwin(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(new string[]
		{
			"Core", "CoreUObject", "Engine", "InputCore", "PhysicsCore", "RenderCore", "Json", "HTTP"
		});
	}
}
