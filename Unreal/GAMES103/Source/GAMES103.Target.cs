using UnrealBuildTool;
using System.Collections.Generic;

public class GAMES103Target : TargetRules
{
	public GAMES103Target( TargetInfo Target) : base(Target)
	{
		Type = TargetType.Game;
		DefaultBuildSettings = BuildSettingsVersion.V2;
		ExtraModuleNames.AddRange( new string[] { "GAMES103" } );
	}
}
