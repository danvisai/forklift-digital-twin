using UnrealBuildTool;

public class ForkliftTwinEditorTarget : TargetRules
{
	public ForkliftTwinEditorTarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Editor;
		DefaultBuildSettings = BuildSettingsVersion.Latest;
		IncludeOrderVersion = EngineIncludeOrderVersion.Latest;
		ExtraModuleNames.Add("ForkliftTwin");
	}
}
