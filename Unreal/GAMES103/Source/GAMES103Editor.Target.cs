using UnrealBuildTool;
using System.Collections.Generic;

public class GAMES103EditorTarget : TargetRules
{
	public GAMES103EditorTarget( TargetInfo Target) : base(Target)
	{
		Type = TargetType.Editor;
		DefaultBuildSettings = BuildSettingsVersion.V2;
		ExtraModuleNames.AddRange( new string[] { "GAMES103" } );
	}
}
